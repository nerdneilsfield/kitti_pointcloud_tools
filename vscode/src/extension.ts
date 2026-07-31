import * as vscode from "vscode";
import { randomBytes } from "node:crypto";
import type {
  ExtensionToWebviewMessage,
  WebviewToExtensionMessage,
} from "./protocol";

const viewType = "kpt.pointCloudViewer";

export interface ExtensionRenderEvent {
  uri: string;
  pointCount?: number;
  error?: string;
}

export interface ExtensionApi {
  readonly onDidRender: vscode.Event<ExtensionRenderEvent>;
}

class PointCloudDocument implements vscode.CustomDocument {
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
    let disposed = false;
    let readInFlight = false;
    let reloadPending = false;
    panel.onDidDispose(() => {
      disposed = true;
      ++requestId;
    });
    const sendCloud = async (): Promise<void> => {
      const currentRequest = ++requestId;
      if (readInFlight) {
        reloadPending = true;
        return;
      }
      readInFlight = true;
      try {
        const configuredMiB = vscode.workspace
          .getConfiguration("kpt")
          .get<number>("maxFileSizeMiB", 256);
        const maximumMiB = Number.isFinite(configuredMiB)
          ? Math.min(Math.max(configuredMiB, 1), 512)
          : 256;
        const maximumBytes = maximumMiB * 1024 * 1024;
        const metadata = await vscode.workspace.fs.stat(document.uri);
        if (metadata.size > maximumBytes) {
          throw new Error(
            `File is ${(metadata.size / 1024 / 1024).toFixed(1)} MiB; ` +
              `configured limit is ${maximumMiB} MiB`,
          );
        }
        const bytes = await vscode.workspace.fs.readFile(document.uri);
        if (bytes.byteLength > maximumBytes) {
          throw new Error(
            `Read ${(bytes.byteLength / 1024 / 1024).toFixed(1)} MiB; ` +
              `configured limit is ${maximumMiB} MiB`,
          );
        }
        if (
          disposed ||
          token.isCancellationRequested ||
          currentRequest !== requestId
        ) {
          return;
        }
        const exactBytes = bytes.buffer.slice(
          bytes.byteOffset,
          bytes.byteOffset + bytes.byteLength,
        ) as ArrayBuffer;
        const message: ExtensionToWebviewMessage = {
          type: "load",
          requestId: currentRequest,
          name: document.uri.path.split("/").pop() ?? "cloud.bin",
          bytes: exactBytes,
        };
        if (!disposed) await panel.webview.postMessage(message);
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
        };
        if (!disposed) await panel.webview.postMessage(message);
      } finally {
        readInFlight = false;
        if (reloadPending && !disposed) {
          reloadPending = false;
          void sendCloud();
        }
      }
    };

    panel.webview.onDidReceiveMessage(
      (message: WebviewToExtensionMessage) => {
        if (message.type === "ready" || message.type === "reload") {
          void sendCloud();
        } else if (message.type === "rendered") {
          this.renderEvents.fire({
            uri: document.uri.toString(),
            pointCount: message.pointCount,
          });
        } else if (message.type === "renderError") {
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
    const workerUri = webview.asWebviewUri(
      vscode.Uri.joinPath(this.extensionUri, "dist", "decoder.worker.js"),
    );
    const wasmUri = webview.asWebviewUri(
      vscode.Uri.joinPath(this.extensionUri, "dist", "kpt_decoder.wasm"),
    );
    const nonce = randomNonce();
    const worker = escapeAttribute(workerUri.toString());
    const wasm = escapeAttribute(wasmUri.toString());

    return `<!doctype html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta
    http-equiv="Content-Security-Policy"
    content="default-src 'none'; script-src 'nonce-${nonce}' 'wasm-unsafe-eval'; style-src ${webview.cspSource} 'unsafe-inline'; worker-src blob:; connect-src ${webview.cspSource} blob:;">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Point Cloud Viewer</title>
  <style>
    html, body, #viewer { width: 100%; height: 100%; margin: 0; overflow: hidden; }
    body { background: var(--vscode-editor-background); color: var(--vscode-editor-foreground); }
    #toolbar { position: fixed; z-index: 3; top: 10px; right: 10px; display: flex;
      box-sizing: border-box;
      max-width: calc(100vw - 20px); flex-wrap: wrap; justify-content: flex-end;
      align-items: center; gap: 5px; padding: 6px; border: 1px solid var(--vscode-panel-border);
      border-radius: 4px; background: var(--vscode-editorWidget-background); }
    #toolbar button, #toolbar select, #toolbar input {
      color: var(--vscode-input-foreground); background: var(--vscode-input-background);
      border: 1px solid var(--vscode-input-border); }
    #toolbar button { min-width: 28px; padding: 3px 6px; cursor: pointer; }
    #toolbar button:hover { background: var(--vscode-toolbar-hoverBackground); }
    #toolbar details { position: relative; }
    #toolbar summary, #controls-help summary { cursor: pointer; user-select: none; }
    #overlay-menu { position: absolute; top: calc(100% + 8px); right: 0;
      display: grid; gap: 7px; min-width: 125px; padding: 8px;
      border: 1px solid var(--vscode-panel-border); border-radius: 4px;
      background: var(--vscode-editorWidget-background); }
    #overlay-menu label { display: flex; align-items: center; gap: 7px; }
    #point-size { width: 80px; }
    #information { position: fixed; z-index: 2; top: 12px; left: 12px;
      display: grid; gap: 6px; max-width: min(520px, calc(50vw - 24px)); }
    #status, #cloud-info { padding: 6px 9px;
      border-radius: 3px; background: color-mix(in srgb, var(--vscode-editor-background) 85%, transparent); }
    #status[data-kind="error"] { color: var(--vscode-errorForeground); }
    #cloud-info { display: grid; gap: 2px; font-variant-numeric: tabular-nums; }
    #cloud-info[hidden] { display: none; }
    #controls-help { position: fixed; z-index: 3; left: 12px; bottom: 64px;
      max-width: min(340px, calc(100vw - 24px)); padding: 6px 9px;
      border: 1px solid var(--vscode-panel-border); border-radius: 4px;
      background: var(--vscode-editorWidget-background); }
    #controls-help div { display: grid; gap: 3px; margin-top: 6px; }
    #player { position: fixed; z-index: 3; left: 50%; bottom: 12px;
      transform: translateX(-50%); display: none; align-items: center; gap: 8px;
      padding: 7px; border: 1px solid var(--vscode-panel-border);
      border-radius: 4px; background: var(--vscode-editorWidget-background); }
    #frame { width: min(50vw, 520px); }
    @media (max-width: 1200px) {
      #toolbar { left: 10px; justify-content: flex-start; }
      #information { top: 152px; max-width: calc(100vw - 24px); }
    }
  </style>
</head>
<body data-worker-uri="${worker}" data-wasm-uri="${wasm}">
  <div id="viewer"></div>
  <div id="information">
    <div id="status">Loading decoder…</div>
    <section id="cloud-info" aria-label="Point cloud bounds" hidden>
      <strong>AABB</strong>
      <span id="aabb-min"></span>
      <span id="aabb-max"></span>
      <span id="aabb-size"></span>
      <span id="grid-spacing"></span>
    </section>
  </div>
  <div id="toolbar" aria-label="Point cloud controls">
    <select id="color-mode" aria-label="Color mode">
      <option value="rgb">RGB</option>
      <option value="intensity">Intensity</option>
      <option value="height">Height</option>
    </select>
    <label title="Point size">Size <input id="point-size" type="range" min="1" max="8" step="0.25" value="1.5"></label>
    <button data-view="fit" title="Fit cloud">Fit</button>
    <button id="reload" title="Reload and cancel current decode">↻</button>
    <button data-view="top" title="Top view">T</button>
    <button data-view="front" title="Front view">F</button>
    <button data-view="left" title="Left view">L</button>
    <button data-view="right" title="Right view">R</button>
    <button data-view="iso" title="Isometric view">Iso</button>
    <details id="overlays" open>
      <summary>Overlays</summary>
      <div id="overlay-menu">
        <label><input id="show-axes" type="checkbox"> Axes</label>
        <label><input id="show-grid" type="checkbox"> 3-plane scale grid</label>
      </div>
    </details>
    <input id="background" type="color" aria-label="Background color" value="#1e1e1e">
  </div>
  <details id="controls-help">
    <summary>Mouse controls</summary>
    <div>
      <span>Left drag — rotate</span>
      <span>Middle drag or wheel — zoom</span>
      <span>Right drag — pan</span>
      <span>Shift + left drag — roll</span>
    </div>
  </details>
  <div id="player">
    <button id="play" title="Play or pause">▶</button>
    <input id="frame" type="range" min="0" max="0" value="0">
    <span id="frame-label">1 / 1</span>
    <select id="rate" aria-label="Playback rate">
      <option value="2">2 fps</option><option value="5" selected>5 fps</option>
      <option value="10">10 fps</option><option value="20">20 fps</option>
    </select>
  </div>
  <script nonce="${nonce}" src="${scriptUri}"></script>
</body>
</html>`;
  }
}

function randomNonce(): string {
  return randomBytes(18).toString("base64");
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
      webviewOptions: { retainContextWhenHidden: false },
    }),
    vscode.commands.registerCommand("kpt.openSequence", async () => {
      await openSequence(context.extensionUri, provider, renderEvents);
    }),
  );
  return { onDidRender: renderEvents.event };
}

const cloudExtensions = new Set([
  "bin", "pcd", "ply", "xyz", "xyzi", "xyzrgb", "xyzrgbi",
]);

async function openSequence(
  extensionUri: vscode.Uri,
  provider: PointCloudEditorProvider,
  renderEvents: vscode.EventEmitter<ExtensionRenderEvent>,
): Promise<void> {
  const selected = await vscode.window.showOpenDialog({
    canSelectMany: true,
    openLabel: "Open point-cloud sequence",
    filters: {
      "Point clouds, labels, and poses": [
        ...cloudExtensions, "label", "csv", "txt",
      ],
    },
  });
  if (!selected?.length) return;
  const clouds = selected
    .filter((uri) => cloudExtensions.has(extensionOf(uri)))
    .sort((left, right) => basename(left).localeCompare(basename(right), undefined, {
      numeric: true,
    }));
  if (!clouds.length) {
    void vscode.window.showErrorMessage("No supported point-cloud files selected.");
    return;
  }
  const labelUris = selected.filter((uri) => extensionOf(uri) === "label");
  const duplicateLabelStem = labelUris.find((uri, index) =>
    labelUris.findIndex((candidate) => stem(candidate) === stem(uri)) !== index
  );
  const duplicateCloudStem = clouds.find((uri, index) =>
    clouds.findIndex((candidate) => stem(candidate) === stem(uri)) !== index
  );
  if (duplicateLabelStem || (labelUris.length && duplicateCloudStem)) {
    void vscode.window.showErrorMessage(
      "Duplicate stems make cloud/label pairing ambiguous; select one sequence.",
    );
    return;
  }
  const labels = new Map(labelUris.map((uri) => [stem(uri), uri]));
  if (labels.size && clouds.some((uri) => !labels.has(stem(uri)))) {
    void vscode.window.showErrorMessage(
      "Labels selected, but one or more point-cloud stems have no matching .label file.",
    );
    return;
  }
  const poseUris = selected.filter((uri) =>
    extensionOf(uri) === "csv" || extensionOf(uri) === "txt"
  ).slice(0, 2);
  const poseSequences = await Promise.all(poseUris.map(readPoses));
  const trajectories = poseSequences.map((sequence) => sequence.positions);
  const framePoses = poseSequences[0]?.matrices ?? [];
  const panel = vscode.window.createWebviewPanel(
    "kpt.sequencePlayer",
    `Point Cloud Sequence · ${clouds.length} frames`,
    vscode.ViewColumn.Active,
    { enableScripts: true, retainContextWhenHidden: true },
  );
  const dist = vscode.Uri.joinPath(extensionUri, "dist");
  panel.webview.options = { enableScripts: true, localResourceRoots: [dist] };
  panel.webview.html = provider.html(panel.webview);
  let disposed = false;
  let latestGeneration = 0;
  let readQueue = Promise.resolve();
  const requestFrames = new Map<number, number>();
  panel.onDidDispose(() => { disposed = true; });
  panel.webview.onDidReceiveMessage(async (
    message: WebviewToExtensionMessage,
  ) => {
    if (message.type === "ready") {
      await panel.webview.postMessage({
        type: "sequenceCatalog",
        frameCount: clouds.length,
        name: basename(clouds[0]),
        trajectories,
        framePoses,
      } satisfies ExtensionToWebviewMessage);
    } else if (message.type === "requestFrame") {
      latestGeneration = Math.max(latestGeneration, message.generation);
      requestFrames.set(message.requestId, message.frameIndex);
      readQueue = readQueue.then(async () => {
        if (message.generation !== latestGeneration || disposed) return;
        const uri = clouds[message.frameIndex];
        if (!uri) return;
        try {
          const bytes = await readBounded(uri);
          const labelUri = labels.get(stem(uri));
          const labelBytes = labelUri ? await readBounded(labelUri) : undefined;
          if (disposed || message.generation !== latestGeneration) return;
          await panel.webview.postMessage({
            type: "load",
            requestId: message.requestId,
            frameIndex: message.frameIndex,
            generation: message.generation,
            name: basename(uri),
            bytes,
            labelBytes,
          } satisfies ExtensionToWebviewMessage);
        } catch (error) {
          if (!disposed && message.generation === latestGeneration) {
            await panel.webview.postMessage({
              type: "hostError",
              requestId: message.requestId,
              frameIndex: message.frameIndex,
              generation: message.generation,
              message: error instanceof Error ? error.message : String(error),
            } satisfies ExtensionToWebviewMessage);
          }
        }
      });
    } else if (message.type === "rendered") {
      const frameIndex = requestFrames.get(message.requestId);
      renderEvents.fire({
        uri: frameIndex === undefined
          ? "sequence"
          : clouds[frameIndex]?.toString() ?? "sequence",
        pointCount: message.pointCount,
      });
      requestFrames.delete(message.requestId);
    }
  });
}

async function readBounded(uri: vscode.Uri): Promise<ArrayBuffer> {
  const configured = vscode.workspace
    .getConfiguration("kpt").get<number>("maxFileSizeMiB", 256);
  const maximumMiB = Number.isFinite(configured)
    ? Math.min(Math.max(configured, 1), 512) : 256;
  const maximum = maximumMiB * 1024 * 1024;
  const stat = await vscode.workspace.fs.stat(uri);
  if (stat.size > maximum) {
    throw new Error(`${basename(uri)} exceeds ${maximumMiB} MiB limit`);
  }
  const bytes = await vscode.workspace.fs.readFile(uri);
  if (bytes.byteLength > maximum) {
    throw new Error(`${basename(uri)} exceeds ${maximumMiB} MiB limit`);
  }
  return bytes.buffer.slice(
    bytes.byteOffset, bytes.byteOffset + bytes.byteLength,
  ) as ArrayBuffer;
}

interface PoseSequence {
  positions: Array<[number, number, number]>;
  matrices: number[][];
}

async function readPoses(
  uri: vscode.Uri,
): Promise<PoseSequence> {
  const text = new TextDecoder().decode(await vscode.workspace.fs.readFile(uri));
  const positions: Array<[number, number, number]> = [];
  const matrices: number[][] = [];
  for (const line of text.split(/\r?\n/u)) {
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

function stem(uri: vscode.Uri): string {
  const name = basename(uri);
  return name.slice(0, Math.max(name.lastIndexOf("."), 0));
}

export function deactivate(): void {}
