import * as vscode from "vscode";
import { convertPointCloud } from "./converter";
import {
  encodeReviewShare,
  parseReviewShare,
  reviewShareState,
  validateReviewShare,
} from "./review-share";
import { createSequenceNameComparator } from "./sequence-order";
import type {
  ExtensionToWebviewMessage,
  ReviewShareDocument,
  ReviewShareState,
  WebviewToExtensionMessage,
} from "./protocol";
import {
  maximumCloudBytes,
  maximumLabelBytes,
  maximumNameBytes,
  maximumReviewShareBytes,
  maximumScreenshotBytes,
  maximumTransportBytes,
} from "./protocol";

const viewType = "kpt.pointCloudViewer";
const binaryViewType = "kpt.binaryPointCloudViewer";

interface HostReviewLayer {
  state: ReviewShareState["layers"][number];
  /** Host-only. Undefined keeps an imported source visibly unresolved. */
  uri?: vscode.Uri;
  /** Original v1 logical identity, including host-only native `path:` keys. */
  originalSourceKey: string;
}

interface HostReviewSession {
  /** Original portable document, retained only in the extension host. */
  original: ReviewShareDocument;
  state: ReviewShareState;
  layers: HostReviewLayer[];
  /** Webview-safe alias → logical source key, used for later re-export. */
  originalKeyForAlias: ReadonlyMap<string, string>;
}

let nextReviewRuntimeSession = 0;

export interface ExtensionRenderEvent {
  uri: string;
  pointCount?: number;
  error?: string;
}

export interface ExtensionApi {
  readonly onDidRender: vscode.Event<ExtensionRenderEvent>;
}

class PointCloudDocument implements vscode.CustomDocument {
  /**
   * Host-owned URI catalog. A reconstructed webview has no retained
   * ArrayBuffers, while this CustomDocument survives editor re-resolution.
   * URI strings never cross the webview boundary.
   */
  readonly overlayCatalog = new LayerReplayCatalog();
  primarySourceKey?: string;
  reviewSession?: HostReviewSession;

  constructor(readonly uri: vscode.Uri) {}
  dispose(): void {}
}

class PointCloudEditorProvider
  implements vscode.CustomReadonlyEditorProvider<PointCloudDocument>
{
  constructor(
    private readonly extensionUri: vscode.Uri,
    private readonly renderEvents:
      vscode.EventEmitter<ExtensionRenderEvent>,
  ) {}

  openCustomDocument(
    uri: vscode.Uri,
  ): PointCloudDocument {
    return new PointCloudDocument(uri);
  }

  async resolveCustomEditor(
    document: PointCloudDocument,
    panel: vscode.WebviewPanel,
    token: vscode.CancellationToken,
  ): Promise<void> {
    const dist = vscode.Uri.joinPath(this.extensionUri, "dist");
    panel.webview.options = {
      enableScripts: true,
      localResourceRoots: [dist],
    };
    panel.webview.html = this.html(panel.webview);

    let requestId = 0;
    // This namespace must stay independent from requestId above: requestId is
    // the document-load generation used to discard stale filesystem reads.
    // Add responses only need unique correlation IDs inside the webview.
    let nextLayerMessageId = 2_000_000_000;
    let disposed = false;
    let readInFlight = false;
    let reloadPending = false;
    let layerDialogOpen = false;
    let exportDialogOpen = false;
    let screenshotDialogOpen = false;
    let reviewShareDialogOpen = false;
    let layerQueue: LayerPayloadQueue | undefined;
    let replayAfterPrimary: QueuedLayerUri[] = [];
    let replayPrimaryRequest: number | undefined;
    let replayRequested = false;
    panel.onDidDispose(() => {
      disposed = true;
      ++requestId;
      layerQueue?.dispose();
    });
    const sendCloud = async (): Promise<void> => {
      const currentRequest = ++requestId;
      if (readInFlight) {
        reloadPending = true;
        return;
      }
      readInFlight = true;
      try {
        const source = await readLayerSource(document.uri);
        if (
          disposed ||
          token.isCancellationRequested ||
          currentRequest !== requestId
        ) {
          return;
        }
        const message: ExtensionToWebviewMessage = {
          type: "load",
          requestId: currentRequest,
          name: source.name,
          bytes: source.bytes,
          sourceKey: source.sourceKey,
        };
        document.primarySourceKey = source.sourceKey;
        if (!disposed) {
          await safePostMessage(panel.webview, message);
          if (replayRequested) {
            replayRequested = false;
            replayPrimaryRequest = currentRequest;
          }
        }
      } catch (error) {
        if (
          disposed ||
          token.isCancellationRequested ||
          currentRequest !== requestId
        ) {
          return;
        }
        const message: ExtensionToWebviewMessage = {
          type: "hostError",
          requestId: currentRequest,
          message: error instanceof Error ? error.message : String(error),
          primary: true,
        };
        if (!disposed) {
          await safePostMessage(panel.webview, message);
          this.renderEvents.fire({
            uri: document.uri.toString(),
            error: message.message,
          });
        }
      } finally {
        readInFlight = false;
        if (reloadPending && !disposed) {
          reloadPending = false;
          void sendCloud();
        }
      }
    };

    const postLayerError = async (
      currentRequest: number,
      message: string,
    ): Promise<void> => {
      if (disposed) return;
      try {
        await safePostMessage(panel.webview, {
          type: "hostError",
          requestId: currentRequest,
          message,
          primary: false,
        });
      } catch {
        // A webview may disappear while an asynchronous Remote filesystem
        // operation finishes. Nothing remains to notify in that case.
      }
    };

    layerQueue = new LayerPayloadQueue(
      readLayerSource,
      (message) => safePostMessage(panel.webview, message),
      (pending) => postLayerError(
        pending.requestId,
        vscode.l10n.t("Unable to add {0}.", basename(pending.uri)),
      ),
      () => ++nextLayerMessageId,
      () => disposed,
    );

    const replayReviewSession = async (shareRequestId: number): Promise<void> => {
      const session = document.reviewSession;
      if (!session || disposed) return;
      // Discard stale Remote payloads before replacing review state. Queue
      // cancellation is generation-based, so an old read cannot post later.
      layerQueue?.drainForReplay();
      await safePostMessage(panel.webview, {
        type: "reviewShareLoaded",
        requestId: shareRequestId,
        document: session.state,
      });
      const pending: QueuedLayerUri[] = session.layers.flatMap((layer) =>
        layer.uri ? [{
          uri: layer.uri,
          requestId: shareRequestId,
          reviewLayer: layer.state,
        }] : []);
      layerQueue?.enqueue(pending);
    };

    const addLayers = async (currentRequest: number): Promise<void> => {
      if (layerDialogOpen) {
        await postLayerError(
          currentRequest,
          vscode.l10n.t("Point-cloud selection is already open."),
        );
        return;
      }
      layerDialogOpen = true;
      try {
        // showOpenDialog executes in VS Code's Remote-aware UI flow and gives
        // this extension host URIs for the selected remote filesystem. Do not
        // accept a URI or path from the webview.
        const selected = await vscode.window.showOpenDialog({
          canSelectFiles: true,
          canSelectFolders: false,
          canSelectMany: true,
          // Keep the picker in the same extension-host filesystem as this
          // editor. In Remote windows this is a remote URI, not a client path.
          defaultUri: vscode.Uri.joinPath(document.uri, ".."),
          openLabel: vscode.l10n.t("Add point clouds"),
          filters: { [vscode.l10n.t("Point clouds")]: [...cloudExtensions] },
        });
        const pending: QueuedLayerUri[] = [];
        for (const uri of selected ?? []) {
          if (disposed) return;
          if (!cloudExtensions.has(extensionOf(uri))) {
            await postLayerError(
              currentRequest,
              vscode.l10n.t("Selected file is not a supported point cloud."),
            );
            continue;
          }
          pending.push({ uri, requestId: currentRequest });
        }
        layerQueue?.enqueue(pending);
      } catch {
        await postLayerError(
          currentRequest,
          vscode.l10n.t("Unable to select point clouds."),
        );
      } finally {
        layerDialogOpen = false;
      }
    };

    const exportPly = async (
      message: Extract<WebviewToExtensionMessage, { type: "exportPly" }>,
    ): Promise<void> => {
      if (exportDialogOpen) {
        await postLayerError(
          message.requestId,
          vscode.l10n.t("Point-cloud export is already open."),
        );
        return;
      }
      exportDialogOpen = true;
      try {
        const target = await vscode.window.showSaveDialog({
          // Preserve Remote URI scheme and authority. Never construct an OS
          // path or ask the webview to choose a destination.
          defaultUri: vscode.Uri.joinPath(
            parentUri(document.uri), message.suggestedName,
          ),
          saveLabel: vscode.l10n.t("Export PLY"),
          filters: { PLY: ["ply"] },
        });
        if (!target || disposed) return;
        if (target.toString() === document.uri.toString()) {
          await postLayerError(
            message.requestId,
            vscode.l10n.t("Output must not overwrite source file."),
          );
          return;
        }
        await vscode.workspace.fs.writeFile(target, new Uint8Array(message.bytes));
        await safePostMessage(panel.webview, {
          type: "exportedPly",
          requestId: message.requestId,
          name: basename(target),
          pointCount: message.pointCount,
        });
      } catch (error) {
        await postLayerError(
          message.requestId,
          vscode.l10n.t(
            "Unable to export PLY: {0}",
            error instanceof Error ? error.message : String(error),
          ),
        );
      } finally {
        exportDialogOpen = false;
      }
    };

    const saveScreenshot = async (
      message: Extract<WebviewToExtensionMessage, { type: "saveScreenshot" }>,
    ): Promise<void> => {
      if (screenshotDialogOpen) {
        await postLayerError(
          message.requestId,
          vscode.l10n.t("Screenshot save is already open."),
        );
        return;
      }
      screenshotDialogOpen = true;
      try {
        // The WebGL canvas lives inside a browser webview, but its output is
        // saved by the extension host. This preserves Remote SSH/container
        // URI scheme and never turns a browser download into a client path.
        const target = await vscode.window.showSaveDialog({
          defaultUri: vscode.Uri.joinPath(
            parentUri(document.uri), message.suggestedName,
          ),
          saveLabel: vscode.l10n.t("Save screenshot"),
          filters: { PNG: ["png"] },
        });
        if (!target || disposed) return;
        if (!await confirmRemoteOverwrite(target)) return;
        await vscode.workspace.fs.writeFile(target, new Uint8Array(message.bytes));
        await safePostMessage(panel.webview, {
          type: "screenshotSaved",
          requestId: message.requestId,
          name: basename(target),
        });
      } catch (error) {
        await postLayerError(
          message.requestId,
          vscode.l10n.t(
            "Unable to save screenshot: {0}",
            error instanceof Error ? error.message : String(error),
          ),
        );
      } finally {
        screenshotDialogOpen = false;
      }
    };

    const exportReviewShare = async (
      message: Extract<WebviewToExtensionMessage, { type: "exportReviewShare" }>,
    ): Promise<void> => {
      if (reviewShareDialogOpen) {
        await postLayerError(
          message.requestId,
          vscode.l10n.t("Review Share save is already open."),
        );
        return;
      }
      reviewShareDialogOpen = true;
      try {
        const target = await vscode.window.showSaveDialog({
          defaultUri: vscode.Uri.joinPath(
            parentUri(document.uri), message.suggestedName,
          ),
          saveLabel: vscode.l10n.t("Save Review Share"),
          filters: { [vscode.l10n.t("Review Share")]: ["json"] },
        });
        if (!target || disposed) return;
        if (!await confirmRemoteOverwrite(target)) return;
        const documentWithReferences = await attachReviewSourcePaths(
          message.document,
          target,
          document,
        );
        await vscode.workspace.fs.writeFile(
          target,
          encodeReviewShare(documentWithReferences),
        );
        await safePostMessage(panel.webview, {
          type: "reviewShareSaved",
          requestId: message.requestId,
          name: basename(target),
        });
      } catch (error) {
        await postLayerError(
          message.requestId,
          vscode.l10n.t(
            "Unable to save Review Share: {0}",
            error instanceof Error ? error.message : String(error),
          ),
        );
      } finally {
        reviewShareDialogOpen = false;
      }
    };

    const importReviewShare = async (
      message: Extract<WebviewToExtensionMessage, { type: "importReviewShare" }>,
    ): Promise<void> => {
      if (reviewShareDialogOpen) {
        await postLayerError(
          message.requestId,
          vscode.l10n.t("Review Share dialog is already open."),
        );
        return;
      }
      reviewShareDialogOpen = true;
      try {
        const selected = await vscode.window.showOpenDialog({
          canSelectFiles: true,
          canSelectFolders: false,
          canSelectMany: false,
          defaultUri: parentUri(document.uri),
          openLabel: vscode.l10n.t("Import Review Share"),
          filters: { [vscode.l10n.t("Review Share")]: ["json"] },
        });
        const shareFile = selected?.[0];
        if (!shareFile || disposed) return;
        // This is an extension-host read against the selected Remote URI. The
        // webview receives parsed semantic data, never the JSON URI/path.
        const bytes = await readReviewShareBounded(shareFile);
        const parsed = parseReviewShare(bytes);
        if (disposed) return;
        document.reviewSession = await createHostReviewSession(shareFile, parsed);
        document.overlayCatalog.clear();
        await replayReviewSession(message.requestId);
      } catch (error) {
        await postLayerError(
          message.requestId,
          vscode.l10n.t(
            "Unable to import Review Share: {0}",
            error instanceof Error ? error.message : String(error),
          ),
        );
      } finally {
        reviewShareDialogOpen = false;
      }
    };

    const replayCatalogAfterPrimary = (): void => {
      if (document.reviewSession) {
        void replayReviewSession(0);
        return;
      }
      const inFlight = layerQueue?.drainForReplay() ?? [];
      replayAfterPrimary = document.overlayCatalog.replay(
        inFlight,
        ++nextLayerMessageId,
      );
      replayRequested = true;
      void sendCloud();
    };

    panel.webview.onDidReceiveMessage(
      (value: unknown) => {
        const message = decodeWebviewMessage(value);
        if (!message) return;
        if (message.type === "ready") {
          // Recreate primary first, then replay host-owned Remote URIs. A
          // settled overlay never depends on a stale webview ArrayBuffer.
          replayCatalogAfterPrimary();
        } else if (message.type === "reload") {
          if (document.reviewSession) void replayReviewSession(0);
          else void sendCloud();
        } else if (message.type === "addLayers") {
          void addLayers(message.requestId);
        } else if (message.type === "removeLayer") {
          document.overlayCatalog.remove(message.sourceKey);
          removeReviewSessionLayer(document.reviewSession, message.sourceKey);
        } else if (message.type === "exportPly") {
          void exportPly(message);
        } else if (message.type === "saveScreenshot") {
          void saveScreenshot(message);
        } else if (message.type === "exportReviewShare") {
          void exportReviewShare(message);
        } else if (message.type === "importReviewShare") {
          void importReviewShare(message);
        } else if (message.type === "rendered") {
          const settled = layerQueue?.settle(message.requestId);
          if (settled) document.overlayCatalog.record(settled);
          if (message.requestId === replayPrimaryRequest) {
            replayPrimaryRequest = undefined;
            const replay = replayAfterPrimary;
            replayAfterPrimary = [];
            layerQueue?.enqueue(replay);
          }
          this.renderEvents.fire({
            uri: document.uri.toString(),
            pointCount: message.pointCount,
          });
        } else if (message.type === "renderError") {
          layerQueue?.settle(message.requestId);
          if (message.requestId === replayPrimaryRequest) {
            replayPrimaryRequest = undefined;
            replayAfterPrimary = [];
          }
          this.renderEvents.fire({
            uri: document.uri.toString(),
            error: message.message,
          });
        }
      },
    );
  }

  html(webview: vscode.Webview): string {
    const scriptUri = webview.asWebviewUri(
      vscode.Uri.joinPath(this.extensionUri, "dist", "webview.js"),
    );
    const nonce = randomNonce();
    const language = /^[a-z]{2,3}(?:-[a-z0-9]+)*$/iu.test(vscode.env.language)
      ? vscode.env.language
      : "en";
    const rawText = webviewStrings();
    const text = Object.fromEntries(
      Object.entries(rawText).map(([key, value]) => [
        key,
        escapeAttribute(value),
      ]),
    );
    const runtimeText = JSON.stringify(rawText).replaceAll("<", "\\u003c");

    return `<!doctype html>
<html lang="${escapeAttribute(language)}">
<head>
  <meta charset="UTF-8">
  <meta
    http-equiv="Content-Security-Policy"
    content="default-src 'none'; script-src 'nonce-${nonce}' 'wasm-unsafe-eval'; style-src ${webview.cspSource} 'unsafe-inline'; worker-src blob: ${webview.cspSource}; connect-src ${webview.cspSource} blob:; object-src 'none'; base-uri 'none'; form-action 'none';">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>${text.viewerTitle}</title>
  <style>
    :root { color-scheme: light dark; }
    * { box-sizing: border-box; }
    html, body, #viewer { width: 100%; height: 100%; margin: 0; overflow: hidden; }
    body { background: var(--vscode-editor-background); color: var(--vscode-editor-foreground);
      font: 13px/1.4 var(--vscode-font-family); }
    button, select, input { font: inherit; }
    .glass { border: 1px solid var(--vscode-panel-border);
      background: var(--vscode-editorWidget-background);
      border-color: color-mix(in srgb, var(--vscode-panel-border) 78%, transparent);
      background: color-mix(in srgb, var(--vscode-editorWidget-background) 90%, transparent);
      box-shadow: 0 8px 28px color-mix(in srgb, #000 22%, transparent);
      backdrop-filter: blur(14px); }
    #toolbar { position: fixed; z-index: 3; top: 14px; left: 50%; display: flex;
      max-width: calc(100vw - 28px); align-items: center; gap: 8px; padding: 7px;
      transform: translateX(-50%); border-radius: 12px; white-space: nowrap; }
    .drag-handle { flex: 0 0 auto; width: 18px; height: 30px; margin: 0 -3px 0 1px;
      border: 0; border-radius: 5px; color: var(--vscode-descriptionForeground); cursor: grab;
      touch-action: none; user-select: none; background: transparent; }
    .drag-handle::before { content: "⠿"; font-size: 17px; line-height: 1; }
    .drag-handle:hover { background: var(--vscode-toolbar-hoverBackground); }
    .drag-handle:active { cursor: grabbing; }
    .tool-group { display: flex; align-items: center; gap: 4px; }
    .tool-group + .tool-group { padding-left: 8px;
      border-left: 1px solid var(--vscode-panel-border); }
    #toolbar button, #toolbar select { height: 30px; color: inherit;
      border: 1px solid transparent; border-radius: 7px; outline: none;
      background: transparent; }
    #toolbar button { min-width: 32px; padding: 0 9px; cursor: pointer; }
    #toolbar button:hover { background: var(--vscode-toolbar-hoverBackground); }
    #toolbar button:focus-visible, #toolbar select:focus-visible, input:focus-visible {
      outline: 2px solid var(--vscode-focusBorder); outline-offset: 1px; }
    #toolbar select { padding: 0 28px 0 9px; cursor: pointer;
      background: var(--vscode-dropdown-background); color: var(--vscode-dropdown-foreground); }
    #point-size-control { display: grid; grid-template-columns: auto 94px 34px;
      align-items: center; gap: 7px; padding: 0 3px; color: var(--vscode-descriptionForeground); }
    #point-size { width: 94px; accent-color: var(--vscode-progressBar-background); }
    #point-size-value { color: var(--vscode-foreground); text-align: right;
      font-variant-numeric: tabular-nums; }
    #controls-help summary { cursor: pointer; user-select: none; list-style: none; }
    #controls-help summary::-webkit-details-marker { display: none; }
    #display-toggle[aria-expanded="true"], #details-toggle[aria-expanded="true"] {
      background: var(--vscode-toolbar-activeBackground);
    }
    #overlay-menu { position: fixed; z-index: 5; top: 62px; right: 14px; display: grid;
      gap: 9px; width: min(240px, calc(100vw - 28px)); padding: 12px; border-radius: 10px; }
    #overlay-menu[hidden] { display: none; }
    #overlay-menu label { display: flex; min-height: 25px; align-items: center;
      justify-content: space-between; gap: 16px; }
    #overlay-menu label.toggle { justify-content: flex-start; }
    #overlay-menu input[type="checkbox"] { width: 16px; height: 16px;
      accent-color: var(--vscode-progressBar-background); }
    #inspection-panel { position: fixed; z-index: 5; top: 62px; right: 14px; display: grid;
      gap: 10px; width: min(360px, calc(100vw - 28px)); padding: 12px; border-radius: 10px; }
    #inspection-panel[hidden] { display: none; }
    #inspection-panel fieldset { display: grid; gap: 7px; margin: 0; padding: 9px;
      border: 1px solid var(--vscode-panel-border); border-radius: 7px; }
    #inspection-panel legend { padding: 0 4px; color: var(--vscode-descriptionForeground); }
    #inspection-panel button, #inspection-panel select, #inspection-panel input { color: inherit;
      border: 1px solid var(--vscode-input-border); border-radius: 5px;
      background: var(--vscode-input-background); }
    #inspection-panel button { min-height: 27px; padding: 0 8px; cursor: pointer; }
    #inspection-panel button:hover:not(:disabled) { background: var(--vscode-toolbar-hoverBackground); }
    #inspection-panel button:disabled { opacity: .55; cursor: default; }
    #inspection-panel input, #inspection-panel select { min-height: 26px; padding: 2px 5px;
      background: var(--vscode-input-background); color: var(--vscode-input-foreground); }
    .roi-grid { display: grid; grid-template-columns: 18px minmax(0, 1fr) minmax(0, 1fr); gap: 5px; align-items: center; }
    .layer-grid { display: grid; grid-template-columns: auto minmax(0, 1fr); gap: 6px; align-items: center; }
    .layer-transform-grid { display: grid; grid-template-columns: 20px repeat(3, minmax(0, 1fr)); gap: 5px; align-items: center; }
    #layer-list { min-height: 76px; width: 100%; }
    #layer-opacity, #layer-size { width: 100%; accent-color: var(--vscode-progressBar-background); }
    .inspection-actions { display: flex; flex-wrap: wrap; gap: 6px; }
    .inspection-result { min-height: 19px; color: var(--vscode-descriptionForeground);
      font-variant-numeric: tabular-nums; word-break: break-word; }
    input[type="color"] { width: 38px; height: 24px; padding: 2px;
      border: 1px solid var(--vscode-input-border); border-radius: 6px; background: transparent; }
    #background-wrap { display: flex; align-items: center; gap: 7px;
      color: var(--vscode-descriptionForeground); }
    #information { position: fixed; z-index: 2; top: 70px; left: 14px; display: grid;
      gap: 8px; max-width: min(390px, calc(100vw - 28px)); }
    #information[hidden] { display: none; }
    .panel-drag-handle { position: absolute; z-index: 1; top: 6px; right: 6px; }
    #status, #cloud-info { padding: 9px 12px; border-radius: 9px; }
    #status { width: fit-content; color: var(--vscode-descriptionForeground); }
    #status[data-kind="ready"] { color: var(--vscode-foreground); }
    #status[data-kind="error"] { color: var(--vscode-errorForeground); }
    #cloud-info { display: grid; grid-template-columns: auto; gap: 2px;
      color: var(--vscode-descriptionForeground); font-size: 12px;
      font-variant-numeric: tabular-nums; }
    #cloud-info strong { margin-bottom: 2px; color: var(--vscode-foreground);
      font-size: 11px; letter-spacing: .08em; text-transform: uppercase; }
    #cloud-info[hidden] { display: none; }
    #controls-help { position: fixed; z-index: 3; left: 14px; bottom: 14px;
      max-width: min(300px, calc(100vw - 28px)); padding: 8px 11px; border-radius: 9px; }
    #controls-help summary { color: var(--vscode-descriptionForeground); }
    #controls-help summary::before { content: "?"; display: inline-grid; width: 18px;
      height: 18px; margin-right: 7px; place-items: center; border-radius: 50%;
      background: var(--vscode-badge-background); color: var(--vscode-badge-foreground); }
    #controls-help div { display: grid; gap: 4px; margin-top: 9px;
      color: var(--vscode-descriptionForeground); font-size: 12px; }
    #player { position: fixed; z-index: 3; left: 50%; bottom: 12px;
      transform: translateX(-50%); display: none; align-items: center; gap: 8px;
      padding: 8px 10px; border-radius: 10px; }
    #frame { width: min(50vw, 520px); }
    #player-options { position: relative; }
    #player-options summary { cursor: pointer; color: var(--vscode-descriptionForeground); }
    #player-options > div { position: absolute; right: 0; bottom: calc(100% + 8px);
      display: flex; align-items: center; gap: 6px; padding: 8px; border-radius: 9px;
      white-space: nowrap; border: 1px solid var(--vscode-panel-border);
      background: var(--vscode-editorWidget-background); box-shadow: 0 8px 28px rgba(0,0,0,.22); }
    #player-options:not([open]) > div { display: none; }
    #player-options button { height: 28px; min-width: 28px; color: inherit;
      border: 1px solid transparent; border-radius: 6px; background: transparent; cursor: pointer; }
    #player-options button:hover { background: var(--vscode-toolbar-hoverBackground); }
    @media (max-width: 900px) {
      #toolbar { left: 10px; right: 10px; max-width: none; overflow-x: auto;
        transform: none; border-radius: 10px; }
      #toolbar .tool-group:last-child { position: sticky; right: 0; padding-right: 3px;
        background: var(--vscode-editorWidget-background); }
      #toolbar .tool-group:nth-last-child(2) { position: sticky; right: 39px;
        background: var(--vscode-editorWidget-background); }
      #details-toggle { min-width: 32px; padding: 0 6px; }
      .details-label { display: none; }
      #information { top: 84px; }
      .view-label { display: none; }
      body.sequence #controls-help { bottom: 78px; }
    }
    .vscode-high-contrast .glass, .vscode-high-contrast-light .glass {
      border: 2px solid var(--vscode-contrastBorder); box-shadow: none;
      backdrop-filter: none; background: var(--vscode-editorWidget-background);
    }
  </style>
</head>
<body>
  <div id="viewer"></div>
  <div id="information">
    <button class="drag-handle panel-drag-handle" type="button" data-drag-handle aria-label="${text.dragDetails}"></button>
    <div id="status" class="glass" role="status" aria-live="polite">${text.loadingDecoder}</div>
    <section id="cloud-info" class="glass" aria-label="${text.pointCloudBounds}" hidden>
      <strong>${text.aabb}</strong>
      <span id="aabb-min"></span>
      <span id="aabb-max"></span>
      <span id="aabb-size"></span>
      <span id="noise-info"></span>
      <span id="grid-spacing"></span>
    </section>
  </div>
  <div id="toolbar" class="glass" role="toolbar" aria-label="${text.pointCloudControls}">
    <button class="drag-handle" type="button" data-drag-handle aria-label="${text.dragToolbar}"></button>
    <div class="tool-group">
    <select id="color-mode" aria-label="${text.colorMode}">
      <option value="rgb">RGB</option>
      <option value="intensity">${text.intensity}</option>
      <option value="height">${text.height}</option>
      <option value="fixed">${text.fixed}</option>
    </select>
    <label id="point-size-control" title="${text.pointSize}"><span>${text.size}</span><input id="point-size" type="range" min="0" max="5" step="0.05" value="1.5"><output id="point-size-value">1.50</output></label>
    </div>
    <div class="tool-group">
    <button data-view="fit" title="${text.fit}" aria-label="${text.fit}">⌗ <span class="view-label">${text.fitShort}</span></button>
    <button id="add-layers" title="${text.addLayers}" aria-label="${text.addLayers}">＋ ${text.add}</button>
    <button id="save-screenshot" title="${text.saveScreenshot}" aria-label="${text.saveScreenshot}">▣</button>
    <button id="reload" title="${text.reload}" aria-label="${text.reload}">↻</button>
    </div>
    <div class="tool-group">
    <button data-view="top" title="${text.topView}">${text.top}</button>
    <button data-view="front" title="${text.frontView}">${text.front}</button>
    <button data-view="left" title="${text.leftView}">${text.left}</button>
    <button data-view="right" title="${text.rightView}">${text.right}</button>
    <button data-view="iso" title="${text.isoView}">${text.iso}</button>
    </div>
    <div class="tool-group">
    <button id="display-toggle" aria-expanded="false" aria-controls="overlay-menu">◇ ${text.display}</button>
    <label id="background-wrap" title="${text.background}">${text.backgroundShort}<input id="background" type="color" aria-label="${text.background}" value="#1e1e1e"></label>
    </div>
    <div class="tool-group">
    <button id="inspection-toggle" aria-expanded="false" aria-controls="inspection-panel">⌖ ${text.inspect}</button>
    <button id="details-toggle" aria-expanded="true" aria-controls="information">ⓘ <span class="details-label">${text.details}</span></button>
    </div>
  </div>
  <div id="overlay-menu" class="glass" hidden>
    <button class="drag-handle panel-drag-handle" type="button" data-drag-handle aria-label="${text.dragDisplay}"></button>
    <label>${text.colormap}<select id="color-map" aria-label="${text.colormap}">
      <option value="turbo">Turbo</option><option value="viridis">Viridis</option>
      <option value="plasma">Plasma</option><option value="inferno">Inferno</option>
      <option value="magma">Magma</option><option value="grayscale">${text.grayscale}</option>
      <option value="hot">Hot</option><option value="jet">Jet</option>
      <option value="spring">Spring</option><option value="autumn">Autumn</option>
    </select></label>
    <label class="toggle"><input id="equalize-intensity" type="checkbox" checked> ${text.equalizeIntensity}</label>
    <label class="toggle"><input id="show-axes" type="checkbox"> ${text.axes}</label>
    <label class="toggle"><input id="show-grid" type="checkbox"> ${text.grid}</label>
    <label class="toggle"><input id="highlight-noise" type="checkbox" checked> ${text.highlightNoise}</label>
    <label>${text.fixedColor}<input id="fixed-color" type="color" value="#ffffff"></label>
    <label>${text.noiseColor}<input id="noise-color" type="color" value="#ff0000"></label>
  </div>
  <aside id="inspection-panel" class="glass" aria-label="${text.inspect}" hidden>
    <button class="drag-handle panel-drag-handle" type="button" data-drag-handle aria-label="${text.dragInspection}"></button>
    <fieldset>
      <legend>${text.layers}</legend>
      <select id="layer-list" size="4" aria-label="${text.layers}"></select>
      <div class="inspection-actions"><button id="fit-active" type="button">${text.fitActive}</button><button id="fit-visible" type="button">${text.fitVisible}</button><button id="remove-layer" type="button">${text.removeLayer}</button></div>
      <label class="toggle"><input id="layer-visible" type="checkbox" checked> ${text.layerVisible}</label>
      <div class="layer-grid"><label for="layer-opacity">${text.opacity}</label><input id="layer-opacity" type="range" min="0" max="1" step="0.05" value="1"></div>
      <div class="layer-grid"><label for="layer-size">${text.layerSize}</label><input id="layer-size" type="range" min="0" max="5" step="0.05" value="1.5"></div>
      <div class="layer-grid"><label for="layer-color">${text.fixedColor}</label><input id="layer-color" type="color" value="#ffffff"></div>
      <div class="layer-transform-grid"><span></span><span>X</span><span>Y</span><span>Z</span>
        <label>${text.translate}</label><input id="layer-pos-x" type="number" step="any"><input id="layer-pos-y" type="number" step="any"><input id="layer-pos-z" type="number" step="any">
        <label>${text.rotate}</label><input id="layer-rot-x" type="number" step="any"><input id="layer-rot-y" type="number" step="any"><input id="layer-rot-z" type="number" step="any">
        <label>${text.scale}</label><input id="layer-scale-x" type="number" step="any"><input id="layer-scale-y" type="number" step="any"><input id="layer-scale-z" type="number" step="any">
      </div>
      <div class="inspection-actions"><button id="apply-layer-transform" type="button">${text.applyTransform}</button></div>
      <output id="layer-transform-state" class="inspection-result" aria-live="polite"></output>
    </fieldset>
    <fieldset>
      <legend>${text.roi}</legend>
      <div class="roi-grid"><span></span><span>${text.minimum}</span><span>${text.maximum}</span>
        <label for="roi-min-x">X</label><input id="roi-min-x" type="number" step="any" aria-label="${text.roiMinX}"><input id="roi-max-x" type="number" step="any" aria-label="${text.roiMaxX}">
        <label for="roi-min-y">Y</label><input id="roi-min-y" type="number" step="any" aria-label="${text.roiMinY}"><input id="roi-max-y" type="number" step="any" aria-label="${text.roiMaxY}">
        <label for="roi-min-z">Z</label><input id="roi-min-z" type="number" step="any" aria-label="${text.roiMinZ}"><input id="roi-max-z" type="number" step="any" aria-label="${text.roiMaxZ}">
      </div>
      <div class="inspection-actions"><button id="apply-roi" type="button">${text.applyRoi}</button><button id="reset-roi" type="button">${text.clearRoi}</button><button id="export-roi" type="button">${text.exportPly}</button></div>
      <output id="roi-result" class="inspection-result" aria-live="polite"></output>
    </fieldset>
    <fieldset>
      <legend>${text.measurement}</legend>
      <div class="inspection-actions"><button id="measure-toggle" type="button" aria-pressed="false">${text.measurementStart}</button><button id="clear-measurement" type="button">${text.clearMeasurement}</button></div>
      <output id="measurement-result" class="inspection-result" aria-live="polite"></output>
      <output id="picking-scope" class="inspection-result" aria-live="polite"></output>
    </fieldset>
    <fieldset>
      <legend>${text.bookmarks}</legend>
      <select id="bookmark-list" aria-label="${text.bookmarks}"></select>
      <div class="inspection-actions"><button id="bookmark-save" type="button">${text.saveBookmark}</button><button id="bookmark-restore" type="button">${text.restoreBookmark}</button><button id="bookmark-remove" type="button">${text.removeBookmark}</button></div>
    </fieldset>
    <fieldset>
      <legend>${text.reviewShare}</legend>
      <div class="inspection-actions"><button id="export-review-share" type="button">${text.exportReviewShare}</button><button id="import-review-share" type="button">${text.importReviewShare}</button></div>
    </fieldset>
  </aside>
  <details id="controls-help" class="glass">
    <button class="drag-handle panel-drag-handle" type="button" data-drag-handle aria-label="${text.dragHelp}"></button>
    <summary>${text.mouseControls}</summary>
    <div>
      <span>${text.leftDrag}</span>
      <span>${text.middleDrag}</span>
      <span>${text.rightDrag}</span>
      <span>${text.shiftLeftDrag}</span>
    </div>
  </details>
  <div id="player" class="glass">
    <button class="drag-handle" type="button" data-drag-handle aria-label="${text.dragPlayer}"></button>
    <button id="play" title="${text.playPause}" aria-label="${text.playPause}">▶</button>
    <input id="frame" type="range" min="0" max="0" value="0" aria-label="${text.frame}">
    <span id="frame-label">1 / 1</span>
    <select id="rate" aria-label="${text.playbackRate}">
      <option value="2">2 fps</option><option value="5" selected>5 fps</option>
      <option value="10">10 fps</option><option value="20">20 fps</option>
    </select>
    <details id="player-options">
      <summary>${text.playbackOptions}</summary>
      <div>
        <button id="previous-frame" title="${text.previousFrame}" aria-label="${text.previousFrame}">⏮</button>
        <button id="next-frame" title="${text.nextFrame}" aria-label="${text.nextFrame}">⏭</button>
        <button id="reverse-play" title="${text.reversePlayback}" aria-label="${text.reversePlayback}">◀</button>
        <button id="reset-playback" title="${text.resetPlayback}" aria-label="${text.resetPlayback}">↺</button>
        <label class="toggle"><input id="loop-playback" type="checkbox"> ${text.loopPlayback}</label>
      </div>
    </details>
  </div>
  <script nonce="${nonce}">globalThis.kptStrings=${runtimeText};</script>
  <script nonce="${nonce}" src="${escapeAttribute(scriptUri.toString())}"></script>
</body>
</html>`;
  }
}

function webviewStrings(): Record<string, string> {
  const language = vscode.env.language.toLowerCase();
  const zh = language === "zh" || language === "zh-cn" ||
    language === "zh-hans";
  const en: Record<string, string> = {
    viewerTitle: "Point Cloud Viewer", loadingDecoder: "Starting viewer…",
    pointCloudBounds: "Point cloud bounds",
    aabb: "Bounds", pointCloudControls: "Point cloud controls",
    colorMode: "Color mode", intensity: "Intensity", height: "Height", fixed: "Fixed",
    pointSize: "Point size", size: "Size",
    fit: "Fit active layer", fitShort: "Fit", addLayers: "Add point-cloud layers",
    add: "Add", saveScreenshot: "Save screenshot", screenshotSaving: "Saving screenshot…",
    screenshotSaved: "Saved screenshot: {0}", reload: "Reload and cancel current decode",
    topView: "Top view", frontView: "Front view", leftView: "Left view",
    rightView: "Right view", isoView: "Isometric view", top: "Top",
    front: "Front", left: "Left", right: "Right", iso: "Iso",
    display: "Display", details: "Details", inspect: "Inspect", axes: "Coordinate axes", grid: "Scale grid",
    colormap: "Color map", grayscale: "Grayscale", equalizeIntensity: "Equalize intensity",
    dragToolbar: "Drag toolbar", dragDetails: "Drag details", dragDisplay: "Drag display settings",
    dragInspection: "Drag inspection panel",
    dragHelp: "Drag controls help", dragPlayer: "Drag sequence player",
    highlightNoise: "Highlight noise", fixedColor: "Fixed color",
    noiseColor: "Noise color", background: "Background color", backgroundShort: "BG",
    mouseControls: "Controls", leftDrag: "Left drag · Rotate",
    middleDrag: "Middle drag / wheel · Zoom", rightDrag: "Right drag · Pan",
    shiftLeftDrag: "Shift + left drag · Roll", playPause: "Play or pause",
    playbackRate: "Playback rate", frame: "Sequence frame", playbackOptions: "Playback options",
    previousFrame: "Previous frame", nextFrame: "Next frame", reversePlayback: "Play reverse",
    resetPlayback: "Reset playback", loopPlayback: "Loop",
    pointsStatus: "{0} points · {1} ms decode · {2} ms index",
    loadingCloud: "Loading {0}…", reloading: "Reloading…",
    noiseAvailable: "Noise: {0} / {1}", noiseUnavailable: "Noise: unavailable",
    minimumUnavailable: "Min: unavailable", maximumUnavailable: "Max: unavailable",
    sizeUnavailable: "Size: unavailable", minimumValue: "Min: {0}",
    maximumValue: "Max: {0}", sizeValue: "Size: {0}",
    gridValue: "Grid: {0} units / division",
    layers: "Layers", fitActive: "Fit active", fitVisible: "Fit visible",
    removeLayer: "Remove", layerVisible: "Visible", opacity: "Opacity",
    layerSize: "Point size", translate: "Move", rotate: "Rotate°", scale: "Scale",
    applyTransform: "Apply transform",
    affineTransformReadOnly: "Exact affine matrix (shear/reflection) is preserved and read-only here.",
    invalidLayerTransform: "Transform is invalid for this layer.",
    roi: "ROI crop", minimum: "Min", maximum: "Max",
    roiMinX: "ROI minimum X", roiMaxX: "ROI maximum X",
    roiMinY: "ROI minimum Y", roiMaxY: "ROI maximum Y",
    roiMinZ: "ROI minimum Z", roiMaxZ: "ROI maximum Z",
    applyRoi: "Apply ROI", clearRoi: "Clear ROI", exportPly: "Export PLY",
    roiCount: "ROI: {0} points", roiInactive: "Full cloud: {0} points",
    roiFiltering: "ROI: filtering…",
    roiInvalid: "ROI needs finite min ≤ max on every axis.",
    roiExported: "Downloaded {0} points as PLY.", roiEmpty: "No finite point in ROI.",
    measurement: "Measure", measurementStart: "Measure", measurementStop: "Stop measuring",
    clearMeasurement: "Clear measurement", measurementEmpty: "Click two points to measure.",
    measurementFirst: "First: {0}. Pick second point.",
    measurementDistance: "Distance: {0} · {1} → {2}",
    measurementMiss: "No sampled point near cursor.",
    measurementDetached: "Detached source",
    pickingAll: "Picking: all {0} visible layers",
    pickingActiveOnly: "Picking: active layer {0} only ({1} visible layers)",
    pickingActiveHidden: "Picking unavailable: active layer is hidden ({0} visible layers)",
    bookmarks: "View bookmarks", bookmarkChoose: "Choose bookmark…",
    bookmarkDefault: "View {0}", bookmarkName: "Bookmark name",
    saveBookmark: "Save", restoreBookmark: "Restore", removeBookmark: "Remove",
    reviewShare: "Review Share", exportReviewShare: "Export", importReviewShare: "Import",
    reviewShareSaving: "Saving Review Share…", reviewShareImporting: "Choosing Review Share…",
    reviewShareLoading: "Loading Review Share ({0} layers)…",
    reviewShareUnresolved: "Unresolved", reviewShareSaved: "Saved Review Share: {0}",
    sequenceTitle: "Point Cloud Sequence · {0} frames",
  };
  if (!zh) return en;
  return {
    ...en,
    viewerTitle: "点云查看器", loadingDecoder: "正在启动查看器…",
    pointCloudBounds: "点云边界", aabb: "边界",
    pointCloudControls: "点云控件", colorMode: "着色模式", intensity: "强度",
    height: "高度", fixed: "固定色", pointSize: "点大小",
    size: "点径", fit: "适配活动图层", fitShort: "适配", addLayers: "添加点云图层",
    add: "添加", saveScreenshot: "保存截图", screenshotSaving: "正在保存截图…",
    screenshotSaved: "已保存截图：{0}", reload: "重新加载并取消当前解码",
    topView: "顶视图", frontView: "前视图", leftView: "左视图", rightView: "右视图",
    isoView: "等轴视图", top: "顶", front: "前", left: "左", right: "右",
    iso: "等轴", display: "显示", details: "详情", inspect: "检查", axes: "坐标轴", grid: "比例网格",
    colormap: "色图", grayscale: "灰度", equalizeIntensity: "强度均衡",
    dragToolbar: "拖动工具栏", dragDetails: "拖动详情", dragDisplay: "拖动显示设置",
    dragInspection: "拖动检查面板",
    dragHelp: "拖动操作帮助", dragPlayer: "拖动序列播放器",
    highlightNoise: "突出噪声", fixedColor: "固定色", noiseColor: "噪声色",
    background: "背景色", backgroundShort: "背景", mouseControls: "操作帮助",
    leftDrag: "左键拖拽 · 旋转", middleDrag: "中键拖拽 / 滚轮 · 缩放",
    rightDrag: "右键拖拽 · 平移", shiftLeftDrag: "Shift + 左键拖拽 · 翻滚",
    playPause: "播放或暂停", playbackRate: "播放速率", frame: "序列帧", playbackOptions: "播放选项",
    previousFrame: "上一帧", nextFrame: "下一帧", reversePlayback: "反向播放",
    resetPlayback: "重置播放", loopPlayback: "循环",
    pointsStatus: "{0} 点 · 解码 {1} ms · 索引 {2} ms",
    loadingCloud: "正在加载 {0}…", reloading: "正在重新加载…",
    noiseAvailable: "噪声：{0} / {1}", noiseUnavailable: "噪声：不可用",
    minimumUnavailable: "最小值：不可用", maximumUnavailable: "最大值：不可用",
    sizeUnavailable: "尺寸：不可用", minimumValue: "最小值：{0}",
    maximumValue: "最大值：{0}", sizeValue: "尺寸：{0}",
    gridValue: "网格：{0} 单位 / 分格",
    layers: "图层", fitActive: "适配活动层", fitVisible: "适配可见层",
    removeLayer: "删除", layerVisible: "可见", opacity: "不透明度",
    layerSize: "点大小", translate: "平移", rotate: "旋转°", scale: "缩放",
    applyTransform: "应用变换",
    affineTransformReadOnly: "精确仿射矩阵（剪切/镜像）已保留，此处不可编辑。",
    invalidLayerTransform: "此图层的变换无效。",
    roi: "ROI 裁剪", minimum: "最小", maximum: "最大",
    roiMinX: "ROI 最小 X", roiMaxX: "ROI 最大 X",
    roiMinY: "ROI 最小 Y", roiMaxY: "ROI 最大 Y",
    roiMinZ: "ROI 最小 Z", roiMaxZ: "ROI 最大 Z",
    applyRoi: "应用 ROI", clearRoi: "清除 ROI", exportPly: "导出 PLY",
    roiCount: "ROI：{0} 点", roiInactive: "完整点云：{0} 点",
    roiFiltering: "ROI：正在筛选…",
    roiInvalid: "ROI 每轴需有限且最小值 ≤ 最大值。",
    roiExported: "已下载 {0} 点 PLY。", roiEmpty: "ROI 内无有限点。",
    measurement: "量测", measurementStart: "量测", measurementStop: "停止量测",
    clearMeasurement: "清除量测", measurementEmpty: "点击两点开始量测。",
    measurementFirst: "第一点：{0}。请选择第二点。",
    measurementDistance: "距离：{0} · {1} → {2}",
    measurementMiss: "光标附近没有抽样点。",
    measurementDetached: "已脱离图层",
    pickingAll: "拾取：全部 {0} 个可见图层",
    pickingActiveOnly: "拾取：仅活动图层 {0}（{1} 个可见图层）",
    pickingActiveHidden: "拾取不可用：活动图层已隐藏（{0} 个可见图层）",
    bookmarks: "视角书签", bookmarkChoose: "选择书签…",
    bookmarkDefault: "视角 {0}", bookmarkName: "书签名称",
    saveBookmark: "保存", restoreBookmark: "恢复", removeBookmark: "删除",
    reviewShare: "审阅分享", exportReviewShare: "导出", importReviewShare: "导入",
    reviewShareSaving: "正在保存审阅分享…", reviewShareImporting: "正在选择审阅分享…",
    reviewShareLoading: "正在加载审阅分享（{0} 个图层）…",
    reviewShareUnresolved: "未解析", reviewShareSaved: "已保存审阅分享：{0}",
    sequenceTitle: "点云序列 · {0} 帧",
  };
}

function randomNonce(): string {
  const bytes = new Uint8Array(18);
  globalThis.crypto.getRandomValues(bytes);
  return Array.from(bytes, (byte) => byte.toString(16).padStart(2, "0")).join("");
}

function escapeAttribute(value: string): string {
  return value
    .replaceAll("&", "&amp;")
    .replaceAll('"', "&quot;")
    .replaceAll("<", "&lt;")
    .replaceAll(">", "&gt;");
}

export function activate(context: vscode.ExtensionContext): ExtensionApi {
  const renderEvents = new vscode.EventEmitter<ExtensionRenderEvent>();
  const provider = new PointCloudEditorProvider(
    context.extensionUri,
    renderEvents,
  );
  context.subscriptions.push(
    renderEvents,
    vscode.window.registerCustomEditorProvider(viewType, provider, {
      supportsMultipleEditorsPerDocument: false,
      // Layer bytes, transforms, ROI, and measurements live in the webview.
      // Retaining it while hidden avoids losing an in-progress review merely
      // because the user switches editors.
      webviewOptions: { retainContextWhenHidden: true },
    }),
    vscode.window.registerCustomEditorProvider(binaryViewType, provider, {
      supportsMultipleEditorsPerDocument: false,
      webviewOptions: { retainContextWhenHidden: true },
    }),
    vscode.commands.registerCommand("kpt.openSequence", async () => {
      await openSequence(context.extensionUri, provider, renderEvents);
    }),
    vscode.commands.registerCommand("kpt.openPointCloud", async (uri?: unknown) => {
      await openPointCloud(uri);
    }),
    vscode.commands.registerCommand("kpt.convertPointCloud", async (uri?: unknown) => {
      await exportPointCloud(context.extensionUri, uri);
    }),
  );
  return { onDidRender: renderEvents.event };
}

const cloudExtensions = new Set([
  "bin", "pcd", "ply", "las", "pts", "obj", "npy",
  "xyz", "xyzi", "xyzrgb", "xyzrgbi",
]);

async function openPointCloud(candidate?: unknown): Promise<void> {
  const uri = await choosePointCloud(candidate, vscode.l10n.t("Open point cloud"));
  if (!uri) return;
  await vscode.commands.executeCommand(
    "vscode.openWith",
    uri,
    ["bin", "npy"].includes(extensionOf(uri)) ? binaryViewType : viewType,
  );
}

interface ExportFormatItem extends vscode.QuickPickItem {
  extension: string;
}

const exportFormats: ExportFormatItem[] = [
  { label: "PCD", description: vscode.l10n.t("Point Cloud Data"), extension: "pcd" },
  { label: "PLY", description: vscode.l10n.t("Polygon File Format"), extension: "ply" },
  { label: "LAS", description: "LAS 1.2", extension: "las" },
  { label: "PTS", description: "Leica Cyclone PTS", extension: "pts" },
  { label: "OBJ", description: "Wavefront OBJ", extension: "obj" },
  { label: "NPY", description: "NumPy array", extension: "npy" },
  { label: "KITTI BIN", description: vscode.l10n.t("XYZ + intensity"), extension: "bin" },
  { label: "XYZ", description: vscode.l10n.t("ASCII XYZ"), extension: "xyz" },
  { label: "XYZI", description: vscode.l10n.t("ASCII XYZ + intensity"), extension: "xyzi" },
  { label: "XYZRGB", description: vscode.l10n.t("ASCII XYZ + RGB"), extension: "xyzrgb" },
  {
    label: "XYZRGBI",
    description: vscode.l10n.t("ASCII XYZ + RGB + intensity"),
    extension: "xyzrgbi",
  },
];

async function exportPointCloud(
  extensionUri: vscode.Uri,
  candidate?: unknown,
): Promise<void> {
  const source = await choosePointCloud(candidate, vscode.l10n.t("Select point cloud to convert"));
  if (!source) return;
  const format = await vscode.window.showQuickPick(exportFormats, {
    placeHolder: vscode.l10n.t("Select output format"),
  });
  if (!format) return;
  const target = await vscode.window.showSaveDialog({
    defaultUri: convertedUri(source, format.extension),
    saveLabel: vscode.l10n.t("Export {0}", format.label),
    filters: { [format.label]: [format.extension] },
  });
  if (!target) return;
  if (target.toString() === source.toString()) {
    void vscode.window.showErrorMessage(vscode.l10n.t("Output must not overwrite source file."));
    return;
  }
  await vscode.window.withProgress(
    {
      location: vscode.ProgressLocation.Notification,
      title: vscode.l10n.t("Converting {0} to {1}", basename(source), format.label),
      cancellable: true,
    },
    async (_progress, token) => {
      try {
        const sourceBytes = await readBounded(source);
        const outputBytes = await convertPointCloud(
          extensionUri,
          basename(source),
          basename(target),
          sourceBytes,
          token,
        );
        await vscode.workspace.fs.writeFile(target, outputBytes);
        void vscode.window.showInformationMessage(
          vscode.l10n.t("Exported {0} ({1} MiB).", basename(target), (outputBytes.byteLength / 1024 / 1024).toFixed(1)),
        );
      } catch (error) {
        void vscode.window.showErrorMessage(
          vscode.l10n.t("Point-cloud conversion failed: {0}", error instanceof Error ? error.message : String(error)),
        );
      }
    },
  );
}

async function choosePointCloud(
  candidate: unknown,
  openLabel: string,
): Promise<vscode.Uri | undefined> {
  if (candidate instanceof vscode.Uri && cloudExtensions.has(extensionOf(candidate))) {
    return candidate;
  }
  const selected = await vscode.window.showOpenDialog({
    canSelectMany: false,
    openLabel,
    filters: { [vscode.l10n.t("Point clouds")]: [...cloudExtensions] },
  });
  return selected?.[0];
}

function convertedUri(source: vscode.Uri, extension: string): vscode.Uri {
  const sourceExtension = extensionOf(source);
  const suffix = sourceExtension === extension ? `.converted.${extension}` : `.${extension}`;
  const dot = source.path.lastIndexOf(".");
  const base = dot > source.path.lastIndexOf("/") ? source.path.slice(0, dot) : source.path;
  return source.with({ path: `${base}${suffix}` });
}

async function openSequence(
  extensionUri: vscode.Uri,
  provider: PointCloudEditorProvider,
  renderEvents: vscode.EventEmitter<ExtensionRenderEvent>,
): Promise<void> {
  const selected = await vscode.window.showOpenDialog({
    canSelectMany: true,
    openLabel: vscode.l10n.t("Open point-cloud sequence"),
    filters: {
      [vscode.l10n.t("Point clouds, labels, and poses")]: [
        ...cloudExtensions, "label", "csv", "txt",
      ],
    },
  });
  if (!selected?.length) return;
  const clouds = selected.filter((uri) => cloudExtensions.has(extensionOf(uri)));
  if (clouds.length > 100_000) {
    void vscode.window.showErrorMessage("Point-cloud sequence exceeds 100,000 frames.");
    return;
  }
  const compareNames = createSequenceNameComparator(clouds.map(basename));
  clouds.sort((left, right) => compareNames(basename(left), basename(right)));
  if (!clouds.length) {
    void vscode.window.showErrorMessage(vscode.l10n.t("No supported point-cloud files selected."));
    return;
  }
  const labelUris = selected.filter((uri) => extensionOf(uri) === "label");
  if (hasDuplicateStem(labelUris) ||
      (labelUris.length > 0 && hasDuplicateStem(clouds))) {
    void vscode.window.showErrorMessage(
      vscode.l10n.t("Duplicate stems make cloud/label pairing ambiguous; select one sequence."),
    );
    return;
  }
  const labels = new Map(labelUris.map((uri) => [stem(uri), uri]));
  if (labels.size && clouds.some((uri) => !labels.has(stem(uri)))) {
    void vscode.window.showErrorMessage(
      vscode.l10n.t("Labels selected, but one or more point-cloud stems have no matching .label file."),
    );
    return;
  }
  const poseUris = selected.filter((uri) =>
    extensionOf(uri) === "csv" || extensionOf(uri) === "txt"
  ).slice(0, 2);
  const poseSequences: PoseSequence[] = [];
  try {
    for (const uri of poseUris) poseSequences.push(await readPoses(uri));
  } catch (error) {
    void vscode.window.showErrorMessage(
      error instanceof Error ? error.message : String(error),
    );
    return;
  }
  const trajectories = poseSequences.map(
    (sequence) => sequence.positions.slice(0, clouds.length),
  );
  const framePoses = poseSequences[0]?.matrices.slice(0, clouds.length) ?? [];
  try {
    validateTransportName(basename(clouds[0]));
  } catch (error) {
    void vscode.window.showErrorMessage(
      error instanceof Error ? error.message : String(error),
    );
    return;
  }
  const panel = vscode.window.createWebviewPanel(
    "kpt.sequencePlayer",
    webviewStrings().sequenceTitle.replace("{0}", String(clouds.length)),
    vscode.ViewColumn.Active,
    { enableScripts: true, retainContextWhenHidden: true },
  );
  const dist = vscode.Uri.joinPath(extensionUri, "dist");
  panel.webview.options = { enableScripts: true, localResourceRoots: [dist] };
  panel.webview.html = provider.html(panel.webview);
  let disposed = false;
  let currentGeneration = 1;
  let reading = false;
  let pendingRequest: Extract<WebviewToExtensionMessage, {
    type: "requestFrame";
  }> | undefined;
  const requestFrames = new Map<number, number>();
  panel.onDidDispose(() => {
    disposed = true;
    pendingRequest = undefined;
    requestFrames.clear();
  });

  const sendFrameError = async (
    message: Extract<WebviewToExtensionMessage, { type: "requestFrame" }>,
    error: unknown,
  ): Promise<void> => {
    requestFrames.delete(message.requestId);
    if (disposed || message.generation !== currentGeneration) return;
    try {
      await safePostMessage(panel.webview, {
        type: "hostError",
        requestId: message.requestId,
        frameIndex: message.frameIndex,
        generation: message.generation,
        message: error instanceof Error ? error.message : String(error),
      } satisfies ExtensionToWebviewMessage);
    } catch {
      // Closing a panel while a remote read completes is expected. There is no
      // receiver left to report to, but the queue must remain usable.
    }
  };

  const pumpFrames = async (): Promise<void> => {
    if (reading || disposed) return;
    reading = true;
    try {
      while (!disposed && pendingRequest) {
        const message = pendingRequest;
        pendingRequest = undefined;
        if (message.generation !== currentGeneration) {
          requestFrames.delete(message.requestId);
          continue;
        }
        const uri = clouds[message.frameIndex];
        if (!uri) {
          await sendFrameError(message, new Error("frame index is out of range"));
          continue;
        }
        try {
          const labelUri = labels.get(stem(uri));
          const [bytes, sourceKey, labelBytes] = await Promise.all([
            readBounded(uri, maximumCloudBytes),
            sourceKeyForUri(uri),
            labelUri
              ? readBounded(labelUri, maximumLabelBytes)
              : Promise.resolve(undefined),
          ]);
          if (bytes.byteLength + (labelBytes?.byteLength ?? 0) >
              maximumTransportBytes) {
            throw new Error("cloud and label exceed transport memory limit");
          }
          if (disposed || message.generation !== currentGeneration) {
            requestFrames.delete(message.requestId);
            continue;
          }
          validateTransportName(basename(uri));
          await safePostMessage(panel.webview, {
            type: "load",
            requestId: message.requestId,
            frameIndex: message.frameIndex,
            generation: message.generation,
            name: basename(uri),
            bytes,
            sourceKey,
            labelBytes,
          } satisfies ExtensionToWebviewMessage);
        } catch (error) {
          await sendFrameError(message, error);
        }
      }
    } finally {
      reading = false;
      if (pendingRequest && !disposed) void pumpFrames();
    }
  };
  panel.webview.onDidReceiveMessage(async (
    value: unknown,
  ) => {
    const message = decodeWebviewMessage(value);
    if (!message) return;
    if (message.type === "ready") {
      if (!disposed) {
        await safePostMessage(panel.webview, {
          type: "sequenceCatalog",
          frameCount: clouds.length,
          name: basename(clouds[0]),
          trajectories,
          framePoses,
        } satisfies ExtensionToWebviewMessage);
      }
    } else if (message.type === "requestFrame") {
      if (message.frameIndex >= clouds.length ||
          message.generation < currentGeneration) return;
      if (message.generation > currentGeneration) {
        currentGeneration = message.generation;
        pendingRequest = undefined;
        requestFrames.clear();
      }
      if (requestFrames.has(message.requestId)) return;
      if (pendingRequest) requestFrames.delete(pendingRequest.requestId);
      requestFrames.set(message.requestId, message.frameIndex);
      pendingRequest = message;
      void pumpFrames();
    } else if (message.type === "rendered") {
      const frameIndex = requestFrames.get(message.requestId);
      renderEvents.fire({
        uri: frameIndex === undefined
          ? "sequence"
          : clouds[frameIndex]?.toString() ?? "sequence",
        pointCount: message.pointCount,
      });
      requestFrames.delete(message.requestId);
    } else if (message.type === "renderError") {
      const frameIndex = requestFrames.get(message.requestId);
      requestFrames.delete(message.requestId);
      renderEvents.fire({
        uri: frameIndex === undefined
          ? "sequence"
          : clouds[frameIndex]?.toString() ?? "sequence",
        error: message.message,
      });
    }
  });
}

/**
 * Serialize a source URI only inside the extension host. Remote files must
 * retain their scheme and authority; converting to a platform-local path
 * would target a client-local file or lose virtual filesystem semantics.
 */
export function serializeSourceUri(uri: vscode.Uri): string {
  const serialized = uri.toString();
  if (new TextEncoder().encode(serialized).byteLength > 16 * 1024) {
    throw new Error("source URI exceeds identity limit");
  }
  return serialized;
}

/**
 * Stable, opaque identity for a source URI. The serialized URI stays in the
 * extension host; the webview receives only this SHA-256 key.
 */
export async function sourceKeyForUri(uri: vscode.Uri): Promise<string> {
  const source = new TextEncoder().encode(serializeSourceUri(uri));
  const digest = await globalThis.crypto.subtle.digest("SHA-256", source);
  return `sha256:${Array.from(new Uint8Array(digest), (byte) =>
    byte.toString(16).padStart(2, "0")).join("")}`;
}

export interface LayerSource {
  sourceKey: string;
  name: string;
  bytes: ArrayBuffer;
}

export interface QueuedLayerUri {
  uri: vscode.Uri;
  requestId: number;
  /** Sanitized import metadata; source paths never enter this payload. */
  reviewLayer?: ReviewShareState["layers"][number];
}

export interface SettledLayerPayload {
  pending: QueuedLayerUri;
  sourceKey: string;
}

/**
 * Host-only catalog of successfully rendered review layers. It deliberately
 * stores VS Code URIs rather than paths so Remote SSH, dev-container, WSL,
 * and virtual filesystems retain their provider/authority on replay.
 */
export class LayerReplayCatalog {
  private readonly sources = new Map<string, vscode.Uri>();

  record(payload: SettledLayerPayload): void {
    this.sources.set(payload.sourceKey, payload.pending.uri);
  }

  remove(sourceKey: string): void {
    this.sources.delete(sourceKey);
  }

  clear(): void {
    this.sources.clear();
  }

  uriFor(sourceKey: string): vscode.Uri | undefined {
    return this.sources.get(sourceKey);
  }

  replay(
    inFlight: readonly QueuedLayerUri[],
    requestId: number,
  ): QueuedLayerUri[] {
    const result: QueuedLayerUri[] = [];
    const seen = new Set<string>();
    const append = (uri: vscode.Uri): void => {
      const serialized = serializeSourceUri(uri);
      if (seen.has(serialized)) return;
      seen.add(serialized);
      result.push({ uri, requestId });
    };
    for (const pending of inFlight) append(pending.uri);
    for (const uri of this.sources.values()) append(uri);
    return result;
  }
}

/**
 * Serialize Remote layer payloads. Only the selected URI metadata may queue;
 * a source ArrayBuffer is read and posted only after the prior layer settles.
 * `settle()` is intentionally shared by successful rendering, decode errors,
 * and reload cancellation acknowledgements.
 */
export class LayerPayloadQueue {
  private readonly pending: QueuedLayerUri[] = [];
  private inFlight?: SettledLayerPayload & { requestId: number };
  private reading?: QueuedLayerUri;
  private pumping = false;
  private disposed = false;
  private replayGeneration = 0;

  constructor(
    private readonly readSource: (uri: vscode.Uri) => Promise<LayerSource>,
    private readonly postLayer: (
      message: Extract<ExtensionToWebviewMessage, { type: "addLayer" }>,
    ) => Promise<void>,
    private readonly postError: (pending: QueuedLayerUri) => Promise<void>,
    private readonly allocateRequestId: () => number,
    private readonly externallyDisposed: () => boolean,
  ) {}

  enqueue(pending: readonly QueuedLayerUri[]): void {
    if (this.disposed || this.externallyDisposed()) return;
    this.pending.push(...pending);
    void this.pump();
  }

  settle(requestId: number): SettledLayerPayload | undefined {
    if (requestId !== this.inFlight?.requestId) return;
    const settled: SettledLayerPayload = {
      pending: this.inFlight.pending,
      sourceKey: this.inFlight.sourceKey,
    };
    this.inFlight = undefined;
    void this.pump();
    return settled;
  }

  /**
   * A reconstructed webview cannot acknowledge an old postMessage payload.
   * Requeue its URI, not its ArrayBuffer, so a fresh Remote read gets a new
   * correlation ID and stale acknowledgements cannot release the next layer.
   */
  retryInFlight(): void {
    if (this.disposed || this.externallyDisposed()) return;
    const replay = this.drainForReplay();
    if (replay.length === 0) return;
    this.pending.unshift(...replay);
    void this.pump();
  }

  /**
   * Stop waiting for old webview acknowledgements and return URI metadata for
   * replay after a freshly rendered primary layer. No ArrayBuffer survives.
   */
  drainForReplay(): QueuedLayerUri[] {
    if (this.disposed || this.externallyDisposed()) return [];
    ++this.replayGeneration;
    const replay = [...this.pending];
    this.pending.length = 0;
    if (this.inFlight) replay.unshift(this.inFlight.pending);
    else if (this.reading) replay.unshift(this.reading);
    this.inFlight = undefined;
    this.reading = undefined;
    return replay;
  }

  dispose(): void {
    this.disposed = true;
    this.pending.length = 0;
    this.inFlight = undefined;
    this.reading = undefined;
  }

  private async pump(): Promise<void> {
    if (this.disposed || this.externallyDisposed() || this.pumping ||
        this.inFlight !== undefined) return;
    const pending = this.pending.shift();
    if (!pending) return;
    this.pumping = true;
    this.reading = pending;
    const replayGeneration = this.replayGeneration;
    try {
      const source = await this.readSource(pending.uri);
      if (this.disposed || this.externallyDisposed() ||
          replayGeneration !== this.replayGeneration) return;
      const requestId = this.allocateRequestId();
      // A portable share owns its stable semantic key. Its resolved URI can
      // change when the JSON moves, so do not silently rewrite measurements
      // to a fresh URI hash on import.
      const sourceKey = pending.reviewLayer?.source_key ?? source.sourceKey;
      this.inFlight = { requestId, pending, sourceKey };
      this.reading = undefined;
      await this.postLayer({
        type: "addLayer",
        requestId,
        sourceKey,
        name: source.name,
        bytes: source.bytes,
        reviewLayer: pending.reviewLayer,
      });
    } catch {
      if (replayGeneration !== this.replayGeneration) return;
      this.inFlight = undefined;
      if (!this.disposed && !this.externallyDisposed()) {
        await this.postError(pending);
      }
    } finally {
      if (this.reading === pending) this.reading = undefined;
      this.pumping = false;
      if (!this.disposed && !this.externallyDisposed() &&
          this.inFlight === undefined) void this.pump();
    }
  }
}

/**
 * Read a selected source exclusively through the VS Code filesystem API. This
 * works unchanged for local, Remote SSH, container, WSL, and virtual URIs.
 */
export async function readLayerSource(uri: vscode.Uri): Promise<LayerSource> {
  const name = basename(uri);
  validateTransportName(name);
  const [bytes, sourceKey] = await Promise.all([
    readBounded(uri),
    sourceKeyForUri(uri),
  ]);
  return { sourceKey, name, bytes };
}

/** Read a Review Share through Remote-aware VS Code FS without allocating an
 * unbounded selected JSON file before its 4 MiB protocol limit is checked. */
export async function readReviewShareBounded(uri: vscode.Uri): Promise<Uint8Array> {
  let stat: vscode.FileStat;
  try {
    stat = await vscode.workspace.fs.stat(uri);
  } catch {
    throw new Error(vscode.l10n.t("Unable to read Review Share: {0}", basename(uri)));
  }
  if (stat.size > maximumReviewShareBytes) {
    throw new Error(vscode.l10n.t("Review Share exceeds 4 MiB limit"));
  }
  let bytes: Uint8Array;
  try {
    bytes = await vscode.workspace.fs.readFile(uri);
  } catch {
    throw new Error(vscode.l10n.t("Unable to read Review Share: {0}", basename(uri)));
  }
  if (bytes.byteLength > maximumReviewShareBytes) {
    throw new Error(vscode.l10n.t("Review Share exceeds 4 MiB limit"));
  }
  return bytes;
}

async function readBounded(
  uri: vscode.Uri,
  hardMaximum = maximumCloudBytes,
): Promise<ArrayBuffer> {
  const configured = vscode.workspace
    .getConfiguration("kpt").get<number>("maxFileSizeMiB", 64);
  const maximumMiB = Number.isFinite(configured)
    ? Math.min(Math.max(configured, 1), 128) : 64;
  const maximum = Math.min(maximumMiB * 1024 * 1024, hardMaximum);
  const effectiveMaximumMiB = maximum / 1024 / 1024;
  let stat: vscode.FileStat;
  try {
    stat = await vscode.workspace.fs.stat(uri);
  } catch {
    throw new Error(vscode.l10n.t("Unable to read {0}.", basename(uri)));
  }
  if (stat.size > maximum) {
    throw new Error(vscode.l10n.t(
      "{0} exceeds {1} MiB limit", basename(uri), effectiveMaximumMiB,
    ));
  }
  let bytes: Uint8Array;
  try {
    bytes = await vscode.workspace.fs.readFile(uri);
  } catch {
    throw new Error(vscode.l10n.t("Unable to read {0}.", basename(uri)));
  }
  if (bytes.byteLength === 0) {
    throw new Error(vscode.l10n.t("{0} is empty", basename(uri)));
  }
  if (bytes.byteLength > maximum) {
    throw new Error(vscode.l10n.t(
      "{0} exceeds {1} MiB limit", basename(uri), effectiveMaximumMiB,
    ));
  }
  return exactArrayBuffer(bytes);
}

interface PoseSequence {
  positions: Array<[number, number, number]>;
  matrices: number[][];
}

async function readPoses(
  uri: vscode.Uri,
): Promise<PoseSequence> {
  const maximumBytes = 16 * 1024 * 1024;
  const metadata = await vscode.workspace.fs.stat(uri);
  if (metadata.size > maximumBytes)
    throw new Error(vscode.l10n.t("{0} exceeds pose-file size limit", basename(uri)));
  const bytes = await vscode.workspace.fs.readFile(uri);
  if (bytes.byteLength > maximumBytes)
    throw new Error(vscode.l10n.t("{0} exceeds pose-file size limit", basename(uri)));
  let rows = 1;
  let rowBytes = 0;
  for (const byte of bytes) {
    if (byte === 10) {
      ++rows;
      rowBytes = 0;
      if (rows > 100_000)
        throw new Error(vscode.l10n.t("{0} exceeds pose-row limit", basename(uri)));
    } else if (++rowBytes > 4096) {
      throw new Error(vscode.l10n.t("{0} contains an overlong pose row", basename(uri)));
    }
  }
  const text = new TextDecoder().decode(bytes);
  const positions: Array<[number, number, number]> = [];
  const matrices: number[][] = [];
  const lines = text.split(/\r?\n/u);
  for (const line of lines) {
    const values = line.trim().split(/[\s,]+/u).map(Number);
    if (values.some((value) => !Number.isFinite(value))) continue;
    if (values.length >= 12) {
      positions.push([values[3], values[7], values[11]]);
      matrices.push([
        values[0], values[4], values[8], 0,
        values[1], values[5], values[9], 0,
        values[2], values[6], values[10], 0,
        values[3], values[7], values[11], 1,
      ]);
    } else if (values.length >= 3) {
      positions.push([values[0], values[1], values[2]]);
      matrices.push([
        1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0,
        values[0], values[1], values[2], 1,
      ]);
    }
  }
  return { positions, matrices };
}

function extensionOf(uri: vscode.Uri): string {
  return basename(uri).split(".").pop()?.toLowerCase() ?? "";
}

function basename(uri: vscode.Uri): string {
  return uri.path.split("/").pop() ?? uri.path;
}

/** Keep scheme/authority while selecting a sibling in a Remote filesystem. */
function parentUri(uri: vscode.Uri): vscode.Uri {
  const slash = uri.path.lastIndexOf("/");
  return uri.with({ path: slash > 0 ? uri.path.slice(0, slash) : "/" });
}

/**
 * Save dialogs are UI hints, not an overwrite guarantee for every Remote or
 * virtual filesystem provider. Check through the extension-host URI API so a
 * screenshot/share can never silently replace an existing Remote artifact.
 */
async function confirmRemoteOverwrite(target: vscode.Uri): Promise<boolean> {
  try {
    await vscode.workspace.fs.stat(target);
  } catch (error) {
    if (error instanceof vscode.FileSystemError && error.code === "FileNotFound") {
      return true;
    }
    throw error;
  }
  const replace = vscode.l10n.t("Replace");
  return await vscode.window.showWarningMessage(
    vscode.l10n.t("{0} already exists. Replace it?", basename(target)),
    { modal: true },
    replace,
  ) === replace;
}

function stem(uri: vscode.Uri): string {
  const name = basename(uri);
  return name.slice(0, Math.max(name.lastIndexOf("."), 0));
}

function hasDuplicateStem(uris: readonly vscode.Uri[]): boolean {
  const seen = new Set<string>();
  for (const uri of uris) {
    const value = stem(uri);
    if (seen.has(value)) return true;
    seen.add(value);
  }
  return false;
}

function validateTransportName(name: string): void {
  if (name.length === 0 ||
      new TextEncoder().encode(name).byteLength > maximumNameBytes ||
      /[\\/\u0000-\u001f\u007f-\u009f]/u.test(name)) {
    throw new Error("point cloud filename is invalid");
  }
}

function exactArrayBuffer(bytes: Uint8Array): ArrayBuffer {
  if (bytes.byteOffset === 0 && bytes.byteLength === bytes.buffer.byteLength &&
      bytes.buffer instanceof ArrayBuffer) return bytes.buffer;
  return bytes.buffer.slice(
    bytes.byteOffset, bytes.byteOffset + bytes.byteLength,
  ) as ArrayBuffer;
}

async function safePostMessage(
  webview: vscode.Webview,
  message: ExtensionToWebviewMessage,
): Promise<void> {
  if (!await webview.postMessage(message)) {
    throw new Error("webview is no longer available");
  }
}

export function decodeWebviewMessage(
  value: unknown,
): WebviewToExtensionMessage | undefined {
  if (!value || typeof value !== "object") return undefined;
  const message = value as Record<string, unknown>;
  if (typeof message.type !== "string") return undefined;
  switch (message.type) {
  case "ready":
  case "reload":
    return { type: message.type };
  case "addLayers":
    if (!validMessageInteger(message.requestId)) return undefined;
    return { type: "addLayers", requestId: message.requestId };
  case "removeLayer":
    if (!validSourceKey(message.sourceKey)) return undefined;
    return { type: "removeLayer", sourceKey: message.sourceKey };
  case "exportPly":
    if (!validMessageInteger(message.requestId) ||
        !validMessageInteger(message.pointCount, 20_000_000) ||
        !validExportName(message.suggestedName) ||
        !validMessageArrayBuffer(message.bytes, maximumTransportBytes)) {
      return undefined;
    }
    return {
      type: "exportPly",
      requestId: message.requestId,
      pointCount: message.pointCount,
      suggestedName: message.suggestedName,
      bytes: message.bytes,
    };
  case "saveScreenshot":
    if (!validMessageInteger(message.requestId) ||
        !validScreenshotName(message.suggestedName) ||
        !validMessageArrayBuffer(message.bytes, maximumScreenshotBytes)) {
      return undefined;
    }
    return {
      type: "saveScreenshot",
      requestId: message.requestId,
      suggestedName: message.suggestedName,
      bytes: message.bytes,
    };
  case "exportReviewShare":
    if (!validMessageInteger(message.requestId) ||
        !validShareExportName(message.suggestedName) ||
        !validWebviewReviewShare(message.document)) return undefined;
    return {
      type: "exportReviewShare",
      requestId: message.requestId,
      suggestedName: message.suggestedName,
      document: message.document,
    };
  case "importReviewShare":
    if (!validMessageInteger(message.requestId)) return undefined;
    return { type: "importReviewShare", requestId: message.requestId };
  case "requestFrame":
    if (!validMessageInteger(message.requestId) ||
        !validMessageInteger(message.frameIndex) ||
        !validMessageInteger(message.generation)) return undefined;
    return {
      type: "requestFrame",
      requestId: message.requestId,
      frameIndex: message.frameIndex,
      generation: message.generation,
    };
  case "rendered":
    if (!validMessageInteger(message.requestId) ||
        !validMessageInteger(message.pointCount, 20_000_000)) return undefined;
    return {
      type: "rendered",
      requestId: message.requestId,
      pointCount: message.pointCount,
    };
  case "renderError":
    if (!validMessageInteger(message.requestId) ||
        typeof message.message !== "string" || message.message.length > 16_384) {
      return undefined;
    }
    return {
      type: "renderError",
      requestId: message.requestId,
      message: message.message,
    };
  default:
    return undefined;
  }
}

function validMessageInteger(value: unknown, maximum = 0xffffffff): value is number {
  return typeof value === "number" && Number.isSafeInteger(value) &&
    value >= 0 && value <= maximum;
}

function validMessageArrayBuffer(
  value: unknown,
  maximum: number,
): value is ArrayBuffer {
  return value instanceof ArrayBuffer && value.byteLength > 0 &&
    value.byteLength <= maximum;
}

function validSourceKey(value: unknown): value is string {
  return typeof value === "string" && /^sha256:[a-f0-9]{64}$/u.test(value);
}

function validExportName(value: unknown): value is string {
  return typeof value === "string" && value.toLowerCase().endsWith(".ply") &&
    new TextEncoder().encode(value).byteLength <= maximumNameBytes &&
    !/[\\/\u0000-\u001f\u007f-\u009f]/u.test(value);
}

function validScreenshotName(value: unknown): value is string {
  return typeof value === "string" && value.toLowerCase().endsWith(".png") &&
    new TextEncoder().encode(value).byteLength <= maximumNameBytes &&
    !/[\\/\u0000-\u001f\u007f-\u009f]/u.test(value);
}

function validShareExportName(value: unknown): value is string {
  return typeof value === "string" && value.toLowerCase().endsWith(".json") &&
    new TextEncoder().encode(value).byteLength <= maximumNameBytes &&
    !/[\\/\u0000-\u001f\u007f-\u009f]/u.test(value);
}

/** Webview is never allowed to submit a host-path logical identity. */
function validWebviewReviewShare(value: unknown): value is ReviewShareDocument {
  return validateReviewShare(value) && value.layers.every((layer) =>
    validSourceKey(layer.source_key) && layer.source_path === null) &&
    value.measurements.every((measurement) =>
      validSourceKey(measurement.first_source_key) &&
      (measurement.second_source_key === null ||
        validSourceKey(measurement.second_source_key)));
}

/**
 * The host alone knows source URIs. Add a portable relative reference only
 * when source and share target share the same Remote filesystem authority.
 */
async function attachReviewSourcePaths(
  review: ReviewShareDocument,
  target: vscode.Uri,
  document: PointCloudDocument,
): Promise<ReviewShareDocument> {
  const primaryKey = document.primarySourceKey ?? await sourceKeyForUri(document.uri);
  const imported = document.reviewSession;
  const importedLayers = new Map(
    imported?.layers.map((layer) => [layer.state.source_key, layer]) ?? [],
  );
  const originalKey = (key: string): string =>
    imported?.originalKeyForAlias.get(key) ?? key;
  return {
    ...review,
    layers: await Promise.all(review.layers.map(async (layer) => {
      const importedLayer = importedLayers.get(layer.source_key);
      const source = importedLayer?.uri ?? (layer.source_key === primaryKey
        ? document.uri
        : document.overlayCatalog.uriFor(layer.source_key));
      return {
        ...layer,
        source_key: originalKey(layer.source_key),
        source_path: source ? relativeUriPath(parentUri(target), source) ?? null : null,
      };
    })),
    measurements: review.measurements.map((measurement) => ({
      ...measurement,
      first_source_key: originalKey(measurement.first_source_key),
      second_source_key: measurement.second_source_key === null
        ? null : originalKey(measurement.second_source_key),
    })),
  };
}

export function relativeUriPath(directory: vscode.Uri, source: vscode.Uri): string | undefined {
  if (directory.scheme !== source.scheme || directory.authority !== source.authority)
    return undefined;
  const base = directory.path.split("/").filter(Boolean);
  const target = source.path.split("/").filter(Boolean);
  let shared = 0;
  while (shared < base.length && shared < target.length &&
         base[shared] === target[shared]) ++shared;
  // v1 share references must remain below their JSON directory. Do not emit
  // `..`, even for a same-authority Remote URI, because a moved share would
  // otherwise escape its review bundle.
  if (shared !== base.length) return undefined;
  const relative = target.slice(shared).join("/");
  return relative.length > 0 ? relative : undefined;
}

/**
 * Resolve share sources solely against the JSON's own Remote-aware URI.
 * There is intentionally no workspace/cwd fallback. Unreadable entries stay
 * in the state with no URI, so webview measurements remain detached rather
 * than being rebound to an unrelated file.
 */
export async function createHostReviewSession(
  shareFile: vscode.Uri,
  document: ReviewShareDocument,
): Promise<HostReviewSession> {
  const sources = await Promise.all(document.layers.map(async (layer) =>
    resolveReviewSourceUri(shareFile, layer.source_path)));
  const aliases = new Map<string, string>();
  const sourceKeys = new Set<string>();
  for (const layer of document.layers) {
    if (!aliases.has(layer.source_key)) {
      let alias = layer.source_key.startsWith("sha256:")
        ? layer.source_key : await opaqueSourceAlias(layer.source_key);
      while (sourceKeys.has(alias)) alias = await opaqueSourceAlias(
        `${layer.source_key}:${sourceKeys.size}`,
      );
      aliases.set(layer.source_key, alias);
      sourceKeys.add(alias);
    }
  }
  // Detached endpoints may name a native path/opaque source absent from
  // layers. Hash all non-transport logical identities too; browser state
  // never receives a host path or arbitrary opaque payload.
  for (const measurement of document.measurements) {
    for (const key of [measurement.first_source_key, measurement.second_source_key]) {
      if (key && !key.startsWith("sha256:") && !aliases.has(key)) {
        let alias = await opaqueSourceAlias(key);
        while (sourceKeys.has(alias)) alias = await opaqueSourceAlias(
          `${key}:${sourceKeys.size}`,
        );
        aliases.set(key, alias);
        sourceKeys.add(alias);
      }
    }
  }
  const alias = (sourceKey: string): string => aliases.get(sourceKey) ?? sourceKey;
  const sanitized: ReviewShareDocument = {
    ...document,
    layers: document.layers.map((layer) => ({
      ...layer,
      source_key: alias(layer.source_key),
    })),
    measurements: document.measurements.map((measurement) => ({
      ...measurement,
      first_source_key: alias(measurement.first_source_key),
      second_source_key: measurement.second_source_key === null
        ? null : alias(measurement.second_source_key),
    })),
  };
  const sessionId = ++nextReviewRuntimeSession;
  const state = reviewShareState(
    sanitized,
    (_layer, index) => `review-${sessionId}-${index + 1}`,
    (layer, index) => sources[index]
      ? basename(sources[index])
      : unresolvedShareName(layer.source_path, index),
  );
  return {
    original: document,
    state,
    layers: state.layers.map((layer, index) => ({
      state: layer,
      uri: sources[index],
      originalSourceKey: document.layers[index].source_key,
    })),
    originalKeyForAlias: new Map([...aliases].map(([original, safe]) =>
      [safe, original])),
  };
}

async function opaqueSourceAlias(identity: string): Promise<string> {
  const bytes = new TextEncoder().encode(identity);
  const digest = await globalThis.crypto.subtle.digest("SHA-256", bytes);
  return `sha256:${Array.from(new Uint8Array(digest), (byte) =>
    byte.toString(16).padStart(2, "0")).join("")}`;
}

async function resolveReviewSourceUri(
  shareFile: vscode.Uri,
  relativePath: string | null,
): Promise<vscode.Uri | undefined> {
  if (!relativePath) return undefined;
  const candidate = vscode.Uri.joinPath(
    parentUri(shareFile),
    ...relativePath.split("/"),
  );
  if (!cloudExtensions.has(extensionOf(candidate))) return undefined;
  try {
    const stat = await vscode.workspace.fs.stat(candidate);
    return stat.type === vscode.FileType.File ? candidate : undefined;
  } catch {
    return undefined;
  }
}

function unresolvedShareName(relativePath: string | null, index: number): string {
  const last = relativePath?.split("/").at(-1);
  return last && last.length > 0 ? last : `Unresolved layer ${index + 1}`;
}

function removeReviewSessionLayer(
  session: HostReviewSession | undefined,
  transportSourceKey: string,
): void {
  if (!session) return;
  const index = session.layers.findIndex((layer) =>
    layer.state.source_key === transportSourceKey);
  if (index < 0) return;
  session.layers.splice(index, 1);
  session.state.layers.splice(index, 1);
  session.original.layers.splice(index, 1);
}

export function deactivate(): void {}
