import type {
  AddLayerMessage,
  DecodedCloudMessage,
  ExtensionToWebviewMessage,
  LoadCloudMessage,
  ReviewShareDocument,
  ReviewShareState,
  WorkerRequest,
  WorkerResponse,
} from "../src/protocol";
import {
  validateReviewShare,
  validateReviewShareStateLayer,
} from "../src/review-share";
import {
  hasPortableCameraFov,
  hasValidSourceKeyByteLength,
  maximumCloudBytes,
  maximumLabelBytes,
  maximumNameBytes,
  maximumScreenshotBytes,
  maximumTransportBytes,
} from "../src/protocol";
import {
  decoderWasmBase64,
  decoderWorkerBase64,
} from "kpt-decoder-resources";
import { PointCloudViewer } from "./viewer";
import type {
  CameraBookmark,
  ColorMap,
  ColorMode,
  LayerSummary,
  LayerTransform,
  PickingScope,
  PointPick,
  RoiBox,
  StandardView,
} from "./viewer";

declare function acquireVsCodeApi(): {
  postMessage(message: unknown): void;
  getState?(): unknown;
  setState?(state: unknown): void;
};

const vscode = acquireVsCodeApi();
const localizedText = (
  globalThis as typeof globalThis & { kptStrings?: Record<string, string> }
).kptStrings ?? {};

const maximumWasmBytes = 64 * 1024 * 1024;
const maximumDecodedBytes = 256 * 1024 * 1024;
const maximumChunks = 2048;

void bootstrap(vscode).catch((error: unknown) => {
  const detail = error instanceof Error ? error.message : String(error);
  const status = document.getElementById("status");
  if (status) {
    status.textContent = detail;
    status.dataset.kind = "error";
  }
  vscode.postMessage({
    type: "renderError",
    requestId: 0,
    message: detail,
  });
});

async function bootstrap(vscode: ReturnType<typeof acquireVsCodeApi>): Promise<void> {
  const container = requiredElement("viewer");
  const workerUri = document.body.dataset.workerUri;
  const wasmUri = document.body.dataset.wasmUri;
  if ((workerUri && !wasmUri) || (!workerUri && wasmUri)) {
    throw new Error("point cloud viewer resources are missing");
  }
  const viewer = new PointCloudViewer(container);
  requiredInput<HTMLInputElement>("background").value =
    viewer.useThemeBackground();
  // Production resources are embedded in this bundle. Explicit URIs remain
  // available to smoke tests, but decoder bytes never cross the extension-host
  // message boundary and production startup cannot stall on resource fetches.
  const [workerSource, wasmBinary] = workerUri && wasmUri
    ? await Promise.all([
        fetchBounded(workerUri, 16 * 1024 * 1024, "decoder worker"),
        fetchBounded(wasmUri, maximumWasmBytes, "decoder WASM"),
      ])
    : [
        decodeBase64Bounded(
          decoderWorkerBase64,
          16 * 1024 * 1024,
          "decoder worker",
        ),
        decodeBase64Bounded(
          decoderWasmBase64,
          maximumWasmBytes,
          "decoder WASM",
        ),
      ];
  const workerBlobUri = URL.createObjectURL(new Blob([workerSource]));
  let worker: Worker | undefined;
  let workerNeedsWasm = true;
  let workerBusy = false;
  const pendingLoads: LoadCloudMessage[] = [];
  let activeWorkerRequest: WorkerRequest | undefined;
  const decodeTimeouts = new Map<number, number>();
  const configuredTimeout = Number(document.body.dataset.decodeTimeoutMs);
  const configuredDecodeTimeout =
    Number.isFinite(configuredTimeout) && configuredTimeout > 0
      ? configuredTimeout
      : undefined;
  let activeRequest = 0;
  let nextRequest = 0;
  // Keep layer-selection request IDs disjoint from host document loads and
  // sequence frame requests. A user can click Add while initial load is live.
  let nextLayerRequest = 1_000_000_000;
  let nextExportRequest = 1_500_000_000;
  let nextScreenshotRequest = 1_600_000_000;
  let nextShareRequest = 1_700_000_000;
  let exportRequestId: number | undefined;
  let screenshotRequestId: number | undefined;
  let shareRequestId: number | undefined;
  // A primary `show()` replaces the renderer's entire layer map. Do not let a
  // faster Add decode render before the initial/reloaded primary cloud, or
  // that later replacement would silently erase it.
  let primaryReady = false;
  let primarySourceKey: string | undefined;
  let preserveLayersOnNextPrimaryLoad = false;
  // Host-side generation cancellation is the primary defense. Keep a local
  // gate too: a message already queued by VS Code before import may still
  // arrive after the Review Share state has replaced the primary scene.
  let reviewSessionActive = false;
  // A stale host replay can be delivered after a newer import while a panel
  // reconstructs. Session generation is host-owned and monotonically grows
  // for the document lifetime, unlike request IDs used for UI correlation.
  let latestReviewSessionGeneration = 0;
  // Reload does not replace a share session. A separate epoch rejects an old
  // asynchronous host replay that belongs to the same session generation.
  let latestReviewReplayEpoch = 0;
  type ReviewActionIdentity = {
    sessionGeneration: number;
    replayEpoch: number;
  };
  const reviewActionIdentity = (): ReviewActionIdentity | undefined =>
    reviewSessionActive && latestReviewSessionGeneration > 0 &&
      latestReviewReplayEpoch > 0
      ? {
        sessionGeneration: latestReviewSessionGeneration,
        replayEpoch: latestReviewReplayEpoch,
      }
      : undefined;
  // Async browser work (PNG/Blob creation) may finish after the host replaces
  // Review Share A with B. Do not post an unacknowledgeable stale request and
  // leave its UI button permanently busy.
  const reviewActionStillCurrent = (
    identity: ReviewActionIdentity | undefined,
  ): boolean => identity
    ? reviewSessionActive &&
      identity.sessionGeneration === latestReviewSessionGeneration &&
      identity.replayEpoch === latestReviewReplayEpoch
    : !reviewSessionActive;
  interface LayerRequest {
    sourceKey: string;
    runtimeId: string;
    name: string;
    append: boolean;
    reviewLayer?: ReviewShareState["layers"][number];
    /** Host review-session generation for a layer replay/add. */
    sessionGeneration?: number;
    /** Host replay epoch paired with `sessionGeneration`. */
    replayEpoch?: number;
    /** Per-source queue revision used to discard a removed decode. */
    sourceRevision?: number;
    /** A primary reload replaces only its own source, retaining review layers. */
    primary?: boolean;
    resetInspection?: boolean;
  }
  const deferredLayerMessages: AddLayerMessage[] = [];
  const deferredLayerDecodes: Array<{
    message: DecodedCloudMessage;
    request: LayerRequest;
  }> = [];
  let frameCount = 0;
  let currentFrame = 0;
  let sequenceGeneration = 1;
  let playback: number | undefined;
  let playbackDirection: 1 | -1 = 1;
  let trajectories: Array<Array<[number, number, number]>> = [];
  let framePoses: number[][] = [];
  const frameCache = new Map<number, DecodedCloudMessage>();
  const requestedFrames = new Set<number>();
  const requestNames = new Map<number, string>();
  const layerRequests = new Map<number, LayerRequest>();
  const cacheBudget = 192 * 1024 * 1024;
  const restoredInspection = readInspectionState(vscode.getState?.());
  let bookmarks = restoredInspection.bookmarks;
  let measurements: PointPick[] = [];
  let currentCloudName = "point-cloud";
  let importedReviewLayers = new Map<string, ReviewShareState["layers"][number]>();
  let selectedUnresolvedSourceKey: string | undefined;
  // Source revisions make an in-flight Remote payload distinguishable from a
  // later deliberate re-add of the same URI. A tombstoned revision may never
  // materialize in this webview after the user has removed its layer.
  const reviewSourceRevisions = new Map<string, number>();
  const removedReviewSourceRevisions = new Map<string, number>();
  let importedReviewRoi: RoiBox | undefined;
  let importedReviewMeasurements: ReviewShareDocument["measurements"] = [];
  // A Review Share can contain more saved measurements than the compact UI
  // displays. Treat them as a read-only preserved list; any new picking is a
  // separate transient measurement, never a destructive replacement.
  let importedReviewMeasurementsDirty = false;

  const persistInspection = (): void => {
    vscode.setState?.({ version: 1, bookmarks });
  };

  /**
   * Full semantic state mirrored to the host before Reload. It has no source
   * URI/path: host owns those bindings and validates every runtime identity.
   */
  const captureReviewShareState = (): ReviewShareState => {
    // An imported share may have no resolvable layer yet, in which case the
    // renderer has no ROI object to own. Its world-space ROI remains review
    // state and must survive a portable re-export.
    const roi = viewer.getRoi() ?? importedReviewRoi;
    const first = measurements[0];
    const second = measurements[1];
    const liveLayers = viewer.getLayers().flatMap((layer) => {
      const state = viewer.getReviewShareLayerState(layer.runtimeId);
      return state ? [state] : [];
    });
    const liveSourceKeys = new Set(liveLayers.map((layer) => layer.source_key));
    const unresolvedLayers = [...importedReviewLayers.values()]
      .filter((layer) => !liveSourceKeys.has(layer.source_key))
      .map((layer) => ({
        ...layer,
        local_to_world: layer.local_to_world.map((row) => [...row]),
        style: {
          ...layer.style,
          fixed_color: [...layer.style.fixed_color] as [number, number, number],
          noise_color: [...layer.style.noise_color] as [number, number, number],
        },
      }));
    return {
      schema_version: 3,
      intensity_scale_mode: viewer.getIntensityScaleMode(),
      layers: [...liveLayers, ...unresolvedLayers],
      roi: roi ? {
        minimum: [...roi.min] as [number, number, number],
        maximum: [...roi.max] as [number, number, number],
      } : null,
      measurements: [
        ...importedReviewMeasurements.map((measurement) => ({
          ...measurement,
          first_world: [...measurement.first_world] as [number, number, number],
          second_world: measurement.second_world &&
            [...measurement.second_world] as [number, number, number] | null,
        })),
        ...((importedReviewMeasurements.length === 0 ||
            importedReviewMeasurementsDirty) && first ? [{
            first_source_key: first.sourceKey,
            first_world: [...first.point] as [number, number, number],
            second_source_key: second?.sourceKey ?? null,
            second_world: second ? [...second.point] as [number, number, number] : null,
          }] : []),
      ],
      bookmarks: bookmarks.map((bookmark) => ({
        name: bookmark.name,
        camera: viewer.getReviewShareCamera(bookmark.camera),
      })),
    };
  };

  const captureReviewShare = (): ReviewShareDocument => {
    const state = captureReviewShareState();
    return {
      ...state,
      layers: state.layers.map(({ name: _name, runtime_id: _runtimeId, ...layer }) => ({
        ...layer,
        source_path: null,
      })),
    };
  };

  /**
   * A Review Share may be entirely unresolved. Its world-space ROI remains
   * editable evidence even while no decoded layer exists to draw a Box3Helper.
   * Once a layer arrives `showDecoded` installs this pending value in Viewer.
   */
  const setReviewRoi = (roi: RoiBox | undefined): boolean => {
    if (viewer.getLayers().length === 0) {
      importedReviewRoi = roi && {
        min: [...roi.min] as RoiBox["min"],
        max: [...roi.max] as RoiBox["max"],
      };
      return true;
    }
    if (!viewer.setRoi(roi)) return false;
    importedReviewRoi = roi && {
      min: [...roi.min] as RoiBox["min"],
      max: [...roi.max] as RoiBox["max"],
    };
    return true;
  };

  /** Mark a live or unresolved source as removed before notifying the host. */
  const tombstoneReviewSource = (sourceKey: string): void => {
    if (!reviewSessionActive) return;
    removedReviewSourceRevisions.set(
      sourceKey,
      reviewSourceRevisions.get(sourceKey) ?? 0,
    );
  };

  /**
   * Host queue invalidation is primary, but a postMessage already in transit
   * can still arrive after Remove. Accept imported state only while it remains
   * in current share state; catalog state is valid only with its manual
   * runtime-ID convention. A higher source revision is a deliberate re-add.
   */
  const acceptsReviewLayerPayload = (message: AddLayerMessage): boolean => {
    if (!reviewSessionActive) return true;
    const revision = message.sourceRevision ?? 0;
    const currentRevision = reviewSourceRevisions.get(message.sourceKey) ?? 0;
    // A re-add has a newer revision than the removed source. Never let a
    // delayed older post lower the local generation after that re-add.
    if (revision < currentRevision) return false;
    const removedRevision = removedReviewSourceRevisions.get(message.sourceKey);
    if (removedRevision !== undefined && revision <= removedRevision) return false;
    const imported = importedReviewLayers.get(message.sourceKey);
    if (message.reviewLayer) {
      if (imported) {
        if (message.reviewLayer.runtime_id !== imported.runtime_id ||
            message.reviewLayer.source_key !== imported.source_key) return false;
      } else if (message.reviewLayer.runtime_id !== message.sourceKey) {
        // Runtime IDs `review-N-M` belong solely to current imported state.
        return false;
      }
    }
    reviewSourceRevisions.set(message.sourceKey, revision);
    if (removedRevision !== undefined) removedReviewSourceRevisions.delete(message.sourceKey);
    return true;
  };

  /**
   * Persist current manual/imported layer semantics in the host catalog. URI
   * ownership remains host-only; this message has no path or raw source URI.
   */
  const publishActiveReviewLayerState = (runtimeId = viewer.getActiveLayerKey()): void => {
    if (!reviewSessionActive || latestReviewSessionGeneration === 0 ||
        latestReviewReplayEpoch === 0 || !runtimeId)
      return;
    const layer = viewer.getReviewShareLayerState(runtimeId);
    if (!layer) return;
    vscode.postMessage({
      type: "reviewLayerState",
      sessionGeneration: latestReviewSessionGeneration,
      replayEpoch: latestReviewReplayEpoch,
      layer,
    });
    publishReviewShareState();
  };

  /** Host snapshot is source of truth across a same-session Reload. */
  const publishReviewShareState = (): void => {
    if (!reviewSessionActive || latestReviewSessionGeneration === 0 ||
        latestReviewReplayEpoch === 0) return;
    vscode.postMessage({
      type: "reviewShareState",
      sessionGeneration: latestReviewSessionGeneration,
      replayEpoch: latestReviewReplayEpoch,
      state: captureReviewShareState(),
    });
  };

  const renderBookmarks = (): void => {
    const select = document.getElementById("bookmark-list") as HTMLSelectElement | null;
    const restore = document.getElementById("bookmark-restore") as HTMLButtonElement | null;
    const remove = document.getElementById("bookmark-remove") as HTMLButtonElement | null;
    if (!select) return;
    const selected = select.value;
    select.replaceChildren();
    const placeholder = document.createElement("option");
    placeholder.value = "";
    placeholder.textContent = localized("bookmarkChoose", "Choose bookmark…");
    select.append(placeholder);
    bookmarks.forEach((bookmark, index) => {
      const option = document.createElement("option");
      option.value = String(index);
      option.textContent = bookmark.name;
      select.append(option);
    });
    select.value = [...select.options].some((option) => option.value === selected)
      ? selected : "";
    const enabled = select.value !== "";
    if (restore) restore.disabled = !enabled;
    if (remove) remove.disabled = !enabled;
  };

  const renderMeasurement = (message?: string): void => {
    const value = document.getElementById("measurement-result");
    const clear = document.getElementById("clear-measurement") as HTMLButtonElement | null;
    if (clear) clear.disabled = measurements.length === 0;
    if (!value) return;
    const preserved = importedReviewMeasurements.length > 0
      ? " · " + formatLocalized(
        "measurementImportedReadOnly",
        [String(importedReviewMeasurements.length)],
        "{0} imported measurement(s) preserved read-only",
      )
      : "";
    if (message) {
      value.textContent = message + preserved;
      return;
    }
    if (measurements.length === 0) {
      value.textContent = localized(
        "measurementEmpty", "Click two points to measure.",
      ) + preserved;
      return;
    }
    const sourceState = (pick: PointPick): string => viewer.hasLayer(pick.sourceKey)
      ? ""
      : ` · ${localized("measurementDetached", "Detached source")}`;
    if (measurements.length === 1) {
      value.textContent = formatLocalized(
        "measurementFirst", [formatVector(measurements[0].point)],
        "First: {0}. Pick second point.",
      ) + sourceState(measurements[0]) + preserved;
      return;
    }
    const [first, second] = measurements;
    const distance = Math.hypot(
      first.point[0] - second.point[0],
      first.point[1] - second.point[1],
      first.point[2] - second.point[2],
    );
    const result = formatLocalized(
      "measurementDistance", [
        formatCoordinate(distance),
        formatVector(first.point),
        formatVector(second.point),
      ],
      "Distance: {0} · {1} → {2}",
    );
    const detached = [first, second].filter(
      (pick) => !viewer.hasLayer(pick.sourceKey),
    );
    value.textContent = detached.length > 0
      ? `${result} · ${localized("measurementDetached", "Detached source")}: ${
          detached.map((pick) => pick.layerName).join(", ")
        }`
      : result;
    value.textContent += preserved;
  };

  const renderRoi = (resultMessage?: string): void => {
    const output = document.getElementById("roi-result");
    const exportButton = document.getElementById("export-roi") as HTMLButtonElement | null;
    const count = viewer.getVisiblePointCount();
    const roi = viewer.getRoi() ?? importedReviewRoi;
    if (output) {
      if (resultMessage) {
        output.textContent = resultMessage;
      } else if (viewer.isRoiFiltering()) {
        output.textContent = localized("roiFiltering", "ROI: filtering…");
      } else if (roi && count > viewer.getBrowserExportPointLimit()) {
        output.textContent = `ROI: ${count.toLocaleString()} points · Browser export limit ${
          viewer.getBrowserExportPointLimit().toLocaleString()
        }`;
      } else {
        output.textContent = roi
          ? formatLocalized("roiCount", [count.toLocaleString()], "ROI: {0} points")
          : formatLocalized("roiInactive", [count.toLocaleString()], "Full cloud: {0} points");
      }
    }
    if (exportButton) {
      exportButton.disabled = exportRequestId !== undefined ||
        !viewer.canDownloadVisiblePly();
    }
  };

  const renderPickingScope = (): void => {
    const output = document.getElementById("picking-scope");
    if (!output) return;
    const scope: PickingScope = viewer.getPickingScope();
    if (!scope.degraded) {
      output.textContent = formatLocalized(
        "pickingAll", [String(scope.visibleLayerCount)],
        "Picking: all {0} visible layers",
      );
      return;
    }
    output.textContent = scope.enabled && scope.activeLayerName
      ? formatLocalized(
        "pickingActiveOnly", [scope.activeLayerName, String(scope.visibleLayerCount)],
        "Picking: active layer {0} only ({1} visible layers)",
      )
      : formatLocalized(
        "pickingActiveHidden", [String(scope.visibleLayerCount)],
        "Picking unavailable: active layer is hidden ({0} visible layers)",
      );
  };

  const renderLayers = (): void => {
    const list = document.getElementById("layer-list") as HTMLSelectElement | null;
    const remove = document.getElementById("remove-layer") as HTMLButtonElement | null;
    const locate = document.getElementById("locate-review-source") as HTMLButtonElement | null;
    const activeKey = viewer.getActiveLayerKey();
    const layers = viewer.getLayers();
    const resolved = new Set(layers.map((layer) => layer.sourceKey));
    if (selectedUnresolvedSourceKey && !importedReviewLayers.has(selectedUnresolvedSourceKey))
      selectedUnresolvedSourceKey = undefined;
    if (list) {
      list.replaceChildren();
      for (const layer of layers) {
        const option = document.createElement("option");
        option.value = layer.runtimeId;
        option.textContent = `${layer.visible ? "●" : "○"} ${layer.name} · ${
          layer.pointCount.toLocaleString()
        }${layer.renderQuality === "lod" ? " · LOD" : ""}${
          layer.transformEditable ? "" : " · affine"
        }`;
        list.append(option);
      }
      for (const layer of importedReviewLayers.values()) {
        if (resolved.has(layer.source_key)) continue;
        const option = document.createElement("option");
        // A selected unresolved row names only a host-sanitized transport
        // key. "Locate source" can therefore ask the Remote extension host
        // for a URI without exposing a path to this webview.
        option.value = layer.runtime_id;
        option.textContent = `○ ${layer.name} · ${localized(
          "reviewShareUnresolved", "Unresolved",
        )}`;
        list.append(option);
      }
      const unresolved = selectedUnresolvedSourceKey
        ? importedReviewLayers.get(selectedUnresolvedSourceKey)
        : undefined;
      list.value = unresolved?.runtime_id ?? activeKey ?? "";
      list.disabled = layers.length === 0 && importedReviewLayers.size === 0;
    }
    if (remove) remove.disabled = !activeKey && selectedUnresolvedSourceKey === undefined;
    if (locate) locate.disabled = selectedUnresolvedSourceKey === undefined;
    const active = selectedUnresolvedSourceKey === undefined
      ? layers.find((layer) => layer.runtimeId === activeKey)
      : undefined;
    const intensityScale = document.getElementById("intensity-scale") as HTMLSelectElement | null;
    const intensityScaleNote = document.getElementById("intensity-scale-note");
    const sharedIntensity = viewer.getIntensityScaleMode() === "shared_visible";
    if (intensityScale) intensityScale.value = viewer.getIntensityScaleMode();
    if (intensityScaleNote) intensityScaleNote.textContent = sharedIntensity
      ? localized("sharedLinearScale", "Shared linear scale; layer range and equalization are retained but inactive.")
      : "";
    syncLayerControls(active);
    renderPickingScope();
  };

  const syncLayerControls = (layer: LayerSummary | undefined): void => {
    const styleElements = [
      document.getElementById("layer-visible"),
      document.getElementById("layer-opacity"),
      document.getElementById("layer-size"),
      document.getElementById("layer-color"),
    ];
    const intensityRange = document.getElementById("intensity-range") as HTMLSelectElement | null;
    const intensityMinimum = document.getElementById("intensity-min") as HTMLInputElement | null;
    const intensityMaximum = document.getElementById("intensity-max") as HTMLInputElement | null;
    const sharedIntensity = viewer.getIntensityScaleMode() === "shared_visible";
    const transformElements = [
      document.getElementById("apply-layer-transform"),
      ...["pos", "rot", "scale"].flatMap((kind) => ["x", "y", "z"].map(
        (axis) => document.getElementById(`layer-${kind}-${axis}`),
      )),
    ];
    for (const element of styleElements) {
      if (element instanceof HTMLInputElement || element instanceof HTMLButtonElement)
        element.disabled = !layer;
    }
    const intensityEnabled = !!layer && layer.colorMode === "intensity" && !sharedIntensity;
    for (const element of [intensityRange, intensityMinimum, intensityMaximum]) {
      if (element) element.disabled = !intensityEnabled;
    }
    for (const element of transformElements) {
      if (element instanceof HTMLInputElement || element instanceof HTMLButtonElement)
        element.disabled = !layer || !layer.transformEditable;
    }
    const transformState = document.getElementById("layer-transform-state");
    if (transformState) transformState.textContent = !layer || layer.transformEditable
      ? ""
      : localized(
        "affineTransformReadOnly",
        "Exact affine matrix (shear/reflection) is preserved and read-only here.",
      );
    if (!layer) return;
    const visible = document.getElementById("layer-visible") as HTMLInputElement | null;
    const opacity = document.getElementById("layer-opacity") as HTMLInputElement | null;
    const size = document.getElementById("layer-size") as HTMLInputElement | null;
    const color = document.getElementById("layer-color") as HTMLInputElement | null;
    if (visible) visible.checked = layer.visible;
    if (opacity) opacity.value = String(layer.opacity);
    if (size) size.value = String(layer.pointSize);
    if (color) color.value = layer.fixedColor;
    if (intensityRange) intensityRange.value = layer.intensityRangeMode;
    if (intensityMinimum) intensityMinimum.value = String(layer.scalarMin);
    if (intensityMaximum) intensityMaximum.value = String(layer.scalarMax);
    const setVector = (kind: "pos" | "rot" | "scale", values: readonly number[]) => {
      for (const [index, axis] of ["x", "y", "z"].entries()) {
        const field = document.getElementById(`layer-${kind}-${axis}`) as HTMLInputElement | null;
        if (field) field.value = String(values[index]);
      }
    };
    setVector("pos", layer.transform.position);
    setVector("rot", layer.transform.rotation);
    setVector("scale", layer.transform.scale);
    syncActiveToolbar(layer);
    showCloudInfo(layer, viewer.getGridSpacing());
  };

  const syncActiveToolbar = (layer: LayerSummary): void => {
    const mode = requiredInput<HTMLSelectElement>("color-mode");
    mode.value = layer.colorMode;
    const modeState = document.getElementById("color-mode-state");
    if (modeState) {
      modeState.textContent = layer.labelColorFallback
        ? localized(
          "labelColorFallback",
          "Label color is unavailable in this viewer; showing Fixed color.",
        )
        : "";
    }
    for (const option of mode.options) {
      option.disabled =
        (option.value === "rgb" && !layer.hasColor) ||
        (option.value === "intensity" && !layer.hasIntensity);
    }
    const colorMap = document.getElementById("color-map") as HTMLSelectElement | null;
    if (colorMap) colorMap.value = layer.colorMap;
    const equalize = document.getElementById("equalize-intensity") as HTMLInputElement | null;
    if (equalize) {
      equalize.checked = layer.intensityEqualize;
      equalize.disabled = viewer.getIntensityScaleMode() === "shared_visible" ||
        layer.colorMode !== "intensity";
    }
    const pointSize = document.getElementById("point-size") as HTMLInputElement | null;
    const pointSizeValue = document.getElementById("point-size-value");
    if (pointSize) pointSize.value = String(layer.pointSize);
    if (pointSizeValue) pointSizeValue.textContent = layer.pointSize.toFixed(2);
    const fixedColor = document.getElementById("fixed-color") as HTMLInputElement | null;
    const noiseColor = document.getElementById("noise-color") as HTMLInputElement | null;
    const highlightNoise = document.getElementById("highlight-noise") as HTMLInputElement | null;
    if (fixedColor) fixedColor.value = layer.fixedColor;
    if (noiseColor) noiseColor.value = layer.noiseColor;
    if (highlightNoise) {
      highlightNoise.checked = layer.highlightNoise;
      highlightNoise.disabled = !layer.hasNoise;
    }
  };

  const resetInspectionForCloud = (message: DecodedCloudMessage): void => {
    measurements = [];
    importedReviewMeasurements = [];
    importedReviewMeasurementsDirty = false;
    importedReviewLayers.clear();
    selectedUnresolvedSourceKey = undefined;
    reviewSourceRevisions.clear();
    removedReviewSourceRevisions.clear();
    importedReviewRoi = undefined;
    viewer.setMeasurement(measurements);
    renderMeasurement();
    const bounds = message.bounds;
    const fields = roiFields();
    for (let axis = 0; axis < 3; ++axis) {
      const minimum = fields.minimum[axis];
      const maximum = fields.maximum[axis];
      if (minimum) {
        minimum.value = bounds ? String(bounds.min[axis]) : "";
        minimum.disabled = !bounds;
      }
      if (maximum) {
        maximum.value = bounds ? String(bounds.max[axis]) : "";
        maximum.disabled = !bounds;
      }
    }
    const apply = document.getElementById("apply-roi") as HTMLButtonElement | null;
    const reset = document.getElementById("reset-roi") as HTMLButtonElement | null;
    if (apply) apply.disabled = !bounds;
    if (reset) reset.disabled = !bounds;
    renderRoi();
  };

  const applyImportedReviewShare = (state: ReviewShareState): void => {
    // Import replaces review state atomically. Terminate a previous primary
    // decode as well as queued Add payloads, otherwise its later `show()`
    // could erase a just-imported Remote review.
    reviewSessionActive = true;
    cancelLayerDecodes("Review Share replaced");
    clearDecodeTimeouts();
    worker?.terminate();
    worker = undefined;
    workerNeedsWasm = true;
    workerBusy = false;
    activeWorkerRequest = undefined;
    pendingLoads.length = 0;
    viewer.resetReview();
    viewer.setIntensityScaleMode(state.intensity_scale_mode);
    importedReviewLayers = new Map(
      state.layers.map((layer) => [layer.source_key, layer]),
    );
    reviewSourceRevisions.clear();
    removedReviewSourceRevisions.clear();
    for (const layer of state.layers) reviewSourceRevisions.set(layer.source_key, 0);
    selectedUnresolvedSourceKey = undefined;
    importedReviewRoi = state.roi ? {
      min: [...state.roi.minimum] as RoiBox["min"],
      max: [...state.roi.maximum] as RoiBox["max"],
    } : undefined;
    importedReviewMeasurements = state.measurements.map((measurement) => ({
      ...measurement,
      first_world: [...measurement.first_world] as [number, number, number],
      second_world: measurement.second_world &&
        [...measurement.second_world] as [number, number, number] | null,
    }));
    importedReviewMeasurementsDirty = false;
    const layerBySource = new Map(state.layers.map((layer) =>
      [layer.source_key, layer]));
    const first = state.measurements[0];
    const second = first?.second_world;
    measurements = first ? [{
      index: -1,
      point: [...first.first_world] as [number, number, number],
      sourceKey: first.first_source_key,
      runtimeId: layerBySource.get(first.first_source_key)?.runtime_id,
      layerName: layerBySource.get(first.first_source_key)?.name ??
        localized("measurementDetached", "Detached source"),
    }, ...(second ? [{
      index: -1,
      point: [...second] as [number, number, number],
      sourceKey: first.second_source_key ?? first.first_source_key,
      runtimeId: first.second_source_key
        ? layerBySource.get(first.second_source_key)?.runtime_id : undefined,
      layerName: first.second_source_key
        ? layerBySource.get(first.second_source_key)?.name ??
          localized("measurementDetached", "Detached source")
        : localized("measurementDetached", "Detached source"),
    }] : [])] : [];
    viewer.setMeasurement(measurements);
    const importedBookmarks = state.bookmarks.flatMap((bookmark) => {
      const camera = cameraBookmarkFromReview(bookmark.camera);
      return camera ? [{ name: bookmark.name, camera }] : [];
    });
    // Review import is additive for personal bookmarks. An imported bookmark
    // with the same name is authoritative; unrelated local views survive.
    bookmarks = mergeBookmarks(bookmarks, importedBookmarks);
    persistInspection();
    const fields = roiFields();
    for (let axis = 0; axis < 3; ++axis) {
      const minimum = fields.minimum[axis];
      const maximum = fields.maximum[axis];
      if (minimum) {
        minimum.value = importedReviewRoi ? String(importedReviewRoi.min[axis]) : "";
        minimum.disabled = false;
      }
      if (maximum) {
        maximum.value = importedReviewRoi ? String(importedReviewRoi.max[axis]) : "";
        maximum.disabled = false;
      }
    }
    primaryReady = true;
    primarySourceKey = undefined;
    preserveLayersOnNextPrimaryLoad = false;
    currentCloudName = "review";
    renderBookmarks();
    renderLayers();
    renderMeasurement();
    renderRoi();
  };

  viewer.setMeasurementPickHandler((pick) => {
    if (!pick) {
      renderMeasurement(localized("measurementMiss", "No sampled point near cursor."));
      return;
    }
    if (!importedReviewMeasurementsDirty &&
        importedReviewMeasurements.length > 0) measurements = [];
    if (measurements.length >= 2) measurements = [];
    importedReviewMeasurementsDirty = true;
    measurements.push(pick);
    viewer.setMeasurement(measurements);
    renderMeasurement();
    publishReviewShareState();
  });
  viewer.setRoiChangeHandler(() => {
    renderRoi();
    publishReviewShareState();
  });
  viewer.setRendererWarningHandler((message) => {
    showStatus(message, "error");
    renderLayers();
    renderRoi();
    renderMeasurement();
  });

  type ReviewMessageIdentity = Pick<
    LayerRequest, "sessionGeneration" | "replayEpoch"
  >;
  const postRenderError = (
    requestId: number,
    message: string,
    identity: ReviewMessageIdentity | undefined = layerRequests.get(requestId),
    cancelled = false,
  ): void => {
    vscode.postMessage({
      type: "renderError",
      requestId,
      message,
      ...(identity?.sessionGeneration === undefined ? {} : {
        sessionGeneration: identity.sessionGeneration,
        replayEpoch: identity.replayEpoch,
      }),
      ...(cancelled ? { cancelled: true } : {}),
    });
  };

  const showDecoded = (message: DecodedCloudMessage): void => {
    const layerRequest = layerRequests.get(message.requestId);
    try {
      // Remove may race a worker result already decoded in this webview. The
      // host tombstones its queue too; this local gate keeps an in-transit
      // payload from recreating a layer before that acknowledgement arrives.
      if (!reviewLayerRequestIsCurrent(layerRequest)) {
        layerRequests.delete(message.requestId);
        return;
      }
      currentCloudName = requestNames.get(message.requestId) ?? currentCloudName;
      requestNames.delete(message.requestId);
      if (message.frameIndex !== undefined) {
        viewer.showTrajectories(trajectories, framePoses[message.frameIndex]);
      }
      // A layer already in the worker when Reload is clicked must wait too.
      // Keep its decoded data until the replacement primary owns the scene.
      if (layerRequest?.append && !layerRequest.primary && !primaryReady) {
        layerRequests.delete(message.requestId);
        deferredLayerDecodes.push({ message, request: layerRequest });
        return;
      }
      layerRequests.delete(message.requestId);
      if (layerRequest?.append) {
        if (layerRequest.primary) {
          // Reload replaces the primary source in place. Do not treat it as a
          // newly-added review layer: preserving the user camera is part of a
          // reload, alongside ROI, measurements, and overlay layers.
          viewer.replaceLayer(
            message, layerRequest.sourceKey, layerRequest.name,
            layerRequest.runtimeId,
          );
        } else {
          viewer.addLayer(
            message, layerRequest.sourceKey, layerRequest.name,
            layerRequest.runtimeId,
          );
        }
      } else {
        viewer.show(
          message,
          layerRequest?.sourceKey ?? `primary:${message.requestId}`,
          layerRequest?.name ?? currentCloudName,
        );
      }
      if (layerRequest?.reviewLayer) {
        if (!viewer.setLayerReviewState(
          layerRequest.runtimeId,
          layerRequest.reviewLayer,
        )) {
          viewer.removeLayer(layerRequest.runtimeId);
          throw new Error("Review Share layer state is invalid");
        }
        importedReviewLayers.delete(layerRequest.reviewLayer.source_key);
      }
      // A manual Add has no imported layer envelope, yet an imported
      // world-space ROI still governs every later review layer. Install that
      // pending ROI as soon as the first decoded layer exists.
      if (importedReviewRoi && !viewer.getRoi()) {
        viewer.setRoi(importedReviewRoi);
      }
      const replacedPrimary = layerRequest?.primary ?? !layerRequest?.append;
      if (layerRequest?.resetInspection ?? !layerRequest?.append) {
        resetInspectionForCloud(message);
      }
      renderLayers();
      if (replacedPrimary) {
        primarySourceKey = layerRequest?.sourceKey ?? primarySourceKey;
        primaryReady = true;
        flushDeferredLayers();
      }
      showStatus(formatLocalized("pointsStatus", [
        message.pointCount.toLocaleString(),
        message.decodeMilliseconds.toFixed(0),
        message.indexMilliseconds.toFixed(0),
      ], "{0} points · {1} ms decode · {2} ms index"), "ready");
      const renderedReviewLayer = reviewSessionActive &&
        layerRequest?.append &&
        layerRequest.sessionGeneration === latestReviewSessionGeneration &&
        layerRequest.replayEpoch === latestReviewReplayEpoch
        ? viewer.getReviewShareLayerState(layerRequest.runtimeId)
        : undefined;
      vscode.postMessage({
        type: "rendered",
        requestId: message.requestId,
        pointCount: message.pointCount,
        ...(renderedReviewLayer === undefined ? {} : {
          reviewLayer: renderedReviewLayer,
          sessionGeneration: latestReviewSessionGeneration,
          replayEpoch: latestReviewReplayEpoch,
        }),
      });
      if (message.frameIndex === currentFrame &&
          frameBytes(message) <= cacheBudget / 3) {
        requestFrame(currentFrame - 1);
        requestFrame(currentFrame + 1);
      }
    } catch (error) {
      const detail = error instanceof Error ? error.message : String(error);
      showStatus(detail, "error");
      postRenderError(message.requestId, detail, layerRequest);
      if (layerRequest?.primary) {
        openPrimaryGateAfterFailure(message.requestId, layerRequest);
      }
    }
  };

  const evictFrameCache = (): void => {
    for (const index of frameCache.keys()) {
      if (Math.abs(index - currentFrame) > 1) frameCache.delete(index);
    }
    let retained = [...frameCache.values()].reduce(
      (total, frame) => total + frameBytes(frame),
      0,
    );
    for (const [index, frame] of frameCache) {
      if (retained <= cacheBudget) break;
      if (index === currentFrame) continue;
      retained -= frameBytes(frame);
      frameCache.delete(index);
    }
  };

  const restartWorker = (): Worker => {
    worker?.terminate();
    clearDecodeTimeouts();
    const next = new Worker(workerBlobUri);
    next.onmessage = (event: MessageEvent<WorkerResponse>) => {
      if (worker !== next) return;
      if (!validWorkerResponse(event.data)) return;
      const message = event.data;
      if (message.type === "decodeStarted") {
        armDecodeTimeout(message, next);
        return;
      }
      clearDecodeTimeout(message.requestId);
      if (activeWorkerRequest?.requestId === message.requestId) {
        workerBusy = false;
        activeWorkerRequest = undefined;
      }
      if (message.type === "decodeError") {
        // The worker resets its decoder promise after any initialization or
        // internal failure. Always resend WASM on next request so recovery is
        // one-shot rather than depending on brittle error-text matching.
        if (message.code === "internal-error") workerNeedsWasm = true;
        if (message.frameIndex !== undefined &&
            message.generation !== sequenceGeneration) {
          dispatchPendingLoad();
          return;
        }
        if (message.frameIndex !== undefined) {
          requestedFrames.delete(message.frameIndex);
          if (message.frameIndex !== currentFrame) {
            dispatchPendingLoad();
            return;
          }
        }
        if (frameCount === 0 && message.requestId !== activeRequest &&
            !layerRequests.has(message.requestId)) return;
        const failedLayerRequest = layerRequests.get(message.requestId);
        layerRequests.delete(message.requestId);
        showStatus(message.message, "error");
        postRenderError(message.requestId, message.message, failedLayerRequest);
        if (failedLayerRequest?.primary) {
          openPrimaryGateAfterFailure(message.requestId, failedLayerRequest);
        }
        dispatchPendingLoad();
        return;
      }
      if (message.frameIndex !== undefined) {
        if (message.generation !== sequenceGeneration) {
          dispatchPendingLoad();
          return;
        }
        requestedFrames.delete(message.frameIndex);
        frameCache.set(message.frameIndex, message);
        evictFrameCache();
        if (message.frameIndex === currentFrame) showDecoded(message);
        dispatchPendingLoad();
        return;
      }
      if (message.requestId === activeRequest ||
          layerRequests.has(message.requestId)) showDecoded(message);
      dispatchPendingLoad();
    };
    const fail = (message: string): void => {
      if (worker !== next) return;
      const failedRequest = activeWorkerRequest;
      clearDecodeTimeouts();
      next.terminate();
      worker = undefined;
      workerNeedsWasm = true;
      workerBusy = false;
      activeWorkerRequest = undefined;
      pendingLoads.length = 0;
      requestedFrames.clear();
      ++sequenceGeneration;
      showStatus(message, "error");
      postRenderError(
        failedRequest?.requestId ?? activeRequest,
        message,
        failedRequest ? layerRequests.get(failedRequest.requestId) : undefined,
      );
      if (failedRequest && layerRequests.get(failedRequest.requestId)?.primary) {
        openPrimaryGateAfterFailure(failedRequest.requestId);
      }
    };
    next.onerror = (event) => fail(event.message || "decoder worker failed");
    next.onmessageerror = () => fail("decoder worker returned unreadable data");
    worker = next;
    workerNeedsWasm = true;
    return next;
  };

  const clearDecodeTimeout = (requestId: number): void => {
    const timeout = decodeTimeouts.get(requestId);
    if (timeout !== undefined) window.clearTimeout(timeout);
    decodeTimeouts.delete(requestId);
  };

  const clearDecodeTimeouts = (): void => {
    for (const timeout of decodeTimeouts.values()) {
      window.clearTimeout(timeout);
    }
    decodeTimeouts.clear();
  };

  const armDecodeTimeout = (request: {
    requestId: number;
    frameIndex?: number;
    generation?: number;
  }, owner: Worker): void => {
    clearDecodeTimeout(request.requestId);
    const timeoutMilliseconds = decodeDeadline(activeWorkerRequest);
    const timeout = window.setTimeout(() => {
      if (!decodeTimeouts.has(request.requestId)) return;
      if (worker !== owner) {
        clearDecodeTimeout(request.requestId);
        return;
      }
      if (request.frameIndex === undefined) {
        if (request.requestId !== activeRequest &&
            !layerRequests.has(request.requestId)) {
          clearDecodeTimeout(request.requestId);
          return;
        }
      } else if (request.generation !== sequenceGeneration) {
        clearDecodeTimeout(request.requestId);
        return;
      }
      clearDecodeTimeouts();
      owner.terminate();
      worker = undefined;
      workerNeedsWasm = true;
      workerBusy = false;
      activeWorkerRequest = undefined;
      pendingLoads.length = 0;
      if (request.frameIndex === undefined &&
          request.requestId === activeRequest) {
        activeRequest = Math.max(activeRequest, request.requestId) + 1;
      } else {
        ++sequenceGeneration;
        requestedFrames.clear();
      }
      const detail = `decoder worker timed out after ${
        timeoutMilliseconds.toLocaleString()
      } ms`;
      showStatus(detail, "error");
      postRenderError(request.requestId, detail);
      if (layerRequests.get(request.requestId)?.primary) {
        openPrimaryGateAfterFailure(request.requestId);
      }
    }, timeoutMilliseconds);
    decodeTimeouts.set(request.requestId, timeout);
  };

  const decodeDeadline = (request?: WorkerRequest): number => {
    if (configuredDecodeTimeout !== undefined) return configuredDecodeTimeout;
    const bytes = request ? transportBytes(request) : 0;
    return Math.min(120_000, 10_000 + Math.ceil(bytes / 1024 / 1024) * 250);
  };

  const dispatchLoad = (message: LoadCloudMessage): void => {
    requestNames.set(message.requestId, message.name);
    if (workerBusy) {
      // Explicit Add selections queue losslessly. Sequence prefetches are
      // replaceable, so retain only the newest queued frame.
      if (message.frameIndex !== undefined) {
        for (let index = pendingLoads.length - 1; index >= 0; --index) {
          const pending = pendingLoads[index];
          if (pending.frameIndex !== undefined) {
            requestedFrames.delete(pending.frameIndex);
            pendingLoads.splice(index, 1);
          }
        }
      }
      pendingLoads.push(message);
      return;
    }
    let decoder: Worker;
    try {
      decoder = worker ?? restartWorker();
      const request: WorkerRequest = workerNeedsWasm
        ? { ...message, wasmBinary: wasmBinary.slice(0) }
        : { ...message };
      const transfers: Transferable[] = [request.bytes];
      if (request.wasmBinary) transfers.push(request.wasmBinary);
      if (request.labelBytes) transfers.push(request.labelBytes);
      workerBusy = true;
      activeWorkerRequest = request;
      armDecodeTimeout(request, decoder);
      decoder.postMessage(request, transfers);
      workerNeedsWasm = false;
    } catch (error) {
      clearDecodeTimeout(message.requestId);
      workerBusy = false;
      activeWorkerRequest = undefined;
      worker?.terminate();
      worker = undefined;
      workerNeedsWasm = true;
      if (message.frameIndex !== undefined) requestedFrames.delete(message.frameIndex);
      const detail = error instanceof Error ? error.message : String(error);
      showStatus(detail, "error");
      postRenderError(message.requestId, detail);
      if (layerRequests.get(message.requestId)?.primary) {
        openPrimaryGateAfterFailure(message.requestId);
      }
    }
  };

  const dispatchPendingLoad = (): void => {
    while (pendingLoads.length > 0) {
      const next = pendingLoads.shift()!;
      if (next.frameIndex !== undefined && next.generation !== sequenceGeneration) {
        requestedFrames.delete(next.frameIndex);
        continue;
      }
      dispatchLoad(next);
      return;
    }
  };

  const reviewLayerRequestIsCurrent = (request: LayerRequest | undefined): boolean => {
    if (!request?.sessionGeneration) return true;
    if (!reviewSessionActive ||
        request.sessionGeneration !== latestReviewSessionGeneration ||
        request.replayEpoch !== latestReviewReplayEpoch) return false;
    const revision = request.sourceRevision ?? 0;
    if (revision !== (reviewSourceRevisions.get(request.sourceKey) ?? 0)) return false;
    const removedRevision = removedReviewSourceRevisions.get(request.sourceKey);
    if (removedRevision !== undefined && revision <= removedRevision) return false;
    if (!request.reviewLayer) return true;
    const imported = importedReviewLayers.get(request.sourceKey);
    return imported
      ? imported.runtime_id === request.reviewLayer.runtime_id
      : request.reviewLayer.runtime_id === request.sourceKey;
  };

  function dispatchLayerMessage(message: AddLayerMessage): void {
    // A Remote read can have begun for session A before session B took over.
    // Do this again after primary gating too, because an already-deferred A
    // payload may otherwise be appended after B's state replacement.
    if (reviewSessionActive &&
        (message.sessionGeneration !== latestReviewSessionGeneration ||
         message.replayEpoch !== latestReviewReplayEpoch)) return;
    // A share can remain unresolved until user chooses its source through the
    // Remote-aware Add dialog. The host then posts a normal addLayer payload,
    // without replay metadata. Bind it to current imported state by stable
    // source key before constructing a runtime layer; otherwise defaults
    // overwrite its affine/style/scalar/visibility during re-export.
    if (!acceptsReviewLayerPayload(message)) return;
    const imported = importedReviewLayers.get(message.sourceKey);
    const catalog = message.reviewLayer?.runtime_id === message.sourceKey
      ? message.reviewLayer
      : undefined;
    const importedState = imported ?? catalog;
    const reviewLayer = importedState?.source_key === message.sourceKey
      ? importedState
      : undefined;
    const load: LoadCloudMessage = {
      type: "load",
      requestId: message.requestId,
      name: message.name,
      bytes: message.bytes,
      sourceKey: message.sourceKey,
    };
    layerRequests.set(message.requestId, {
      sourceKey: message.sourceKey,
      runtimeId: reviewLayer?.runtime_id ?? message.sourceKey,
      name: message.name,
      append: true,
      reviewLayer,
      sessionGeneration: message.sessionGeneration,
      replayEpoch: message.replayEpoch,
      sourceRevision: message.sourceRevision,
    });
    showStatus(formatLocalized(
      "loadingCloud", [message.name], "Loading {0}…",
    ), "loading");
    dispatchLoad(load);
  }

  function flushDeferredLayers(): void {
    // Decodes that completed during a reload can be appended immediately.
    // Requests not sent to the worker yet retain their original FIFO order.
    const decoded = deferredLayerDecodes.splice(0);
    for (const deferred of decoded) {
      layerRequests.set(deferred.message.requestId, deferred.request);
      showDecoded(deferred.message);
    }
    for (const deferred of deferredLayerMessages.splice(0)) {
      dispatchLayerMessage(deferred);
    }
  }

  /** A failed primary must not strand a queued Remote Add payload forever. */
  function openPrimaryGateAfterFailure(
    requestId: number,
    knownRequest?: LayerRequest,
  ): void {
    const request = knownRequest ?? layerRequests.get(requestId);
    if (!request?.primary) return;
    layerRequests.delete(requestId);
    primaryReady = true;
    flushDeferredLayers();
  }

  /**
   * A Reload terminates the decoder worker. A host Add payload may be waiting
   * for its rendered/renderError acknowledgement, so explicitly settle every
   * layer request before dropping local decode state. Without this, a slow
   * Remote primary followed by Reload could leave the host's one-payload
   * backpressure gate closed forever.
  */
  function cancelLayerDecodes(detail: string): void {
    const cancelled = new Map<number, ReviewMessageIdentity | undefined>();
    for (const [id, request] of layerRequests) {
      if (!request.primary) cancelled.set(id, request);
      requestNames.delete(id);
    }
    for (const deferred of deferredLayerDecodes) {
      cancelled.set(deferred.message.requestId, deferred.request);
      requestNames.delete(deferred.message.requestId);
    }
    for (const deferred of deferredLayerMessages) {
      cancelled.set(deferred.requestId, {
        sessionGeneration: deferred.sessionGeneration,
        replayEpoch: deferred.replayEpoch,
      });
      requestNames.delete(deferred.requestId);
    }
    layerRequests.clear();
    deferredLayerDecodes.length = 0;
    deferredLayerMessages.length = 0;
    for (let index = pendingLoads.length - 1; index >= 0; --index) {
      const request = pendingLoads[index];
      if (!cancelled.has(request.requestId)) continue;
      pendingLoads.splice(index, 1);
      clearDecodeTimeout(request.requestId);
    }
    for (const [requestId, identity] of cancelled) {
      clearDecodeTimeout(requestId);
      postRenderError(requestId, detail, identity, true);
    }
  }

  const requestFrame = (index: number): void => {
    if (index < 0 || index >= frameCount || requestedFrames.has(index)) return;
    const cached = frameCache.get(index);
    if (cached) {
      if (index === currentFrame) showDecoded(cached);
      return;
    }
    requestedFrames.add(index);
    vscode.postMessage({
      type: "requestFrame",
      requestId: ++nextRequest,
      frameIndex: index,
      generation: sequenceGeneration,
    });
  };

  const updateFrameLabel = (): void => {
    requiredElement("frame-label").textContent =
      `${currentFrame + 1} / ${Math.max(frameCount, 1)}`;
  };

  const selectFrame = (index: number): void => {
    currentFrame = Math.max(0, Math.min(index, frameCount - 1));
    requiredInput<HTMLInputElement>("frame").value = String(currentFrame);
    updateFrameLabel();
    evictFrameCache();
    if ([...requestedFrames].some(
      (requested) => Math.abs(requested - currentFrame) > 1,
    )) {
      ++sequenceGeneration;
      requestedFrames.clear();
    }
    requestFrame(currentFrame);
  };

  window.addEventListener(
    "message",
    (event: MessageEvent<unknown>) => {
      if (!validExtensionMessage(event.data)) return;
      const message = event.data;
      if (message.type === "sequenceCatalog") {
        frameCount = message.frameCount;
        document.body.classList.add("sequence");
        // The sequence host owns a frame stream, not a layer-source map.
        // Do not expose an Add button whose request it cannot fulfill.
        const addLayers = document.getElementById("add-layers");
        if (addLayers) addLayers.hidden = true;
        currentFrame = 0;
        frameCache.clear();
        requestedFrames.clear();
        trajectories = message.trajectories;
        framePoses = message.framePoses;
        viewer.showTrajectories(trajectories, framePoses[0]);
        requiredElement("player").style.display = "flex";
        const frame = requiredInput<HTMLInputElement>("frame");
        frame.max = String(Math.max(frameCount - 1, 0));
        frame.value = "0";
        updateFrameLabel();
        requestFrame(0);
        return;
      }
      if (message.type === "hostError") {
        if (message.generation !== undefined &&
            message.generation !== sequenceGeneration) return;
        if (message.frameIndex !== undefined) {
          requestedFrames.delete(message.frameIndex);
          if (message.frameIndex !== currentFrame) return;
        }
        if (frameCount === 0 && message.requestId < activeRequest) return;
        if (message.requestId === exportRequestId) {
          exportRequestId = undefined;
          renderRoi();
        }
        if (message.requestId === screenshotRequestId) {
          screenshotRequestId = undefined;
        }
        if (message.requestId === shareRequestId) {
          shareRequestId = undefined;
        }
        showStatus(message.message, "error");
        // Document read failures originate in the extension host before a
        // LoadCloud message can create a LayerRequest. The host marks them
        // explicitly; request ID ranges are not stable across repeated Add.
        if (frameCount === 0 && !primaryReady && message.primary === true) {
          primaryReady = true;
          flushDeferredLayers();
        }
        return;
      }
      if (message.type === "exportedPly") {
        if (message.requestId !== exportRequestId) return;
        exportRequestId = undefined;
        renderRoi(formatLocalized(
          "roiExported", [message.pointCount.toLocaleString()],
          "Exported {0} points as PLY.",
        ));
        return;
      }
      if (message.type === "screenshotSaved") {
        if (message.requestId !== screenshotRequestId) return;
        screenshotRequestId = undefined;
        showStatus(formatLocalized(
          "screenshotSaved", [message.name], "Saved screenshot: {0}"),
          "ready",
        );
        return;
      }
      if (message.type === "reviewShareSaved") {
        if (message.requestId !== shareRequestId) return;
        shareRequestId = undefined;
        showStatus(formatLocalized(
          "reviewShareSaved", [message.name], "Saved Review Share: {0}"),
          "ready",
        );
        return;
      }
      if (message.type === "reviewShareLoaded") {
        if (message.sessionGeneration < latestReviewSessionGeneration ||
            (message.sessionGeneration === latestReviewSessionGeneration &&
             message.replayEpoch <= latestReviewReplayEpoch)) return;
        latestReviewSessionGeneration = message.sessionGeneration;
        latestReviewReplayEpoch = message.replayEpoch;
        // A save/capture begun under a superseded epoch is deliberately
        // ignored by the host. Release its local spinner too, otherwise the
        // new review is left unable to request its own export or screenshot.
        exportRequestId = undefined;
        screenshotRequestId = undefined;
        shareRequestId = undefined;
        applyImportedReviewShare(message.document);
        showStatus(formatLocalized(
          "reviewShareLoading", [String(message.document.layers.length)],
          "Loading Review Share ({0} layers)…",
        ), "loading");
        return;
      }
      if (message.type === "addLayer") {
        if (reviewSessionActive &&
            (message.sessionGeneration !== latestReviewSessionGeneration ||
             message.replayEpoch !== latestReviewReplayEpoch)) return;
        if (!primaryReady) {
          deferredLayerMessages.push(message);
          showStatus(formatLocalized(
            "loadingCloud", [message.name], "Waiting for {0}…",
          ), "loading");
          return;
        }
        dispatchLayerMessage(message);
        return;
      }
      if (message.type !== "load") return;
      if (reviewSessionActive && message.frameIndex === undefined) {
        // Primary document payload predates an imported session. Do not decode
        // or acknowledge it as a render: the host invalidated its generation
        // and no queue slot depends on this document-only message.
        return;
      }
      if (message.frameIndex !== undefined &&
          message.generation !== sequenceGeneration) return;
      if (message.frameIndex === undefined) {
        primaryReady = false;
        activeRequest = message.requestId;
        layerRequests.set(message.requestId, {
          sourceKey: message.sourceKey ?? `primary:${message.requestId}`,
          runtimeId: message.sourceKey ?? `primary:${message.requestId}`,
          name: message.name,
          append: preserveLayersOnNextPrimaryLoad &&
            message.sourceKey !== undefined && message.sourceKey === primarySourceKey,
          primary: true,
          resetInspection: !(preserveLayersOnNextPrimaryLoad &&
            message.sourceKey !== undefined && message.sourceKey === primarySourceKey),
        });
        preserveLayersOnNextPrimaryLoad = false;
      }
      if (message.frameIndex === undefined ||
          message.frameIndex === currentFrame) {
        showStatus(formatLocalized(
          "loadingCloud", [message.name], "Loading {0}…",
        ), "loading");
      }
      dispatchLoad(message);
    },
  );

  const themeObserver = new MutationObserver(() => {
    const color = viewer.syncThemeBackground();
    if (color) requiredInput<HTMLInputElement>("background").value = color;
  });
  themeObserver.observe(document.documentElement, {
    attributes: true,
    attributeFilter: ["class", "style"],
  });
  window.addEventListener("unload", () => {
    clearDecodeTimeouts();
    if (playback !== undefined) window.clearInterval(playback);
    themeObserver.disconnect();
    worker?.terminate();
    URL.revokeObjectURL(workerBlobUri);
    viewer.dispose();
  });

  requiredInput<HTMLSelectElement>("color-mode").addEventListener(
    "change",
    (event) => {
      viewer.setColorMode(
        (event.currentTarget as HTMLSelectElement).value as ColorMode,
      );
      publishActiveReviewLayerState();
    },
  );
  const colorMap = document.getElementById("color-map") as HTMLSelectElement | null;
  const equalizeIntensity = document.getElementById("equalize-intensity") as HTMLInputElement | null;
  if (colorMap) {
    colorMap.addEventListener("change", () => {
      viewer.setColorMap(colorMap.value as ColorMap);
      publishActiveReviewLayerState();
    });
  }
  if (equalizeIntensity) {
    equalizeIntensity.addEventListener("change", () => {
      viewer.setIntensityEqualization(equalizeIntensity.checked);
      publishActiveReviewLayerState();
    });
  }
  const intensityScale = document.getElementById("intensity-scale") as HTMLSelectElement | null;
  if (intensityScale) {
    intensityScale.addEventListener("change", () => {
      if (!viewer.setIntensityScaleMode(
        intensityScale.value as "per_layer" | "shared_visible",
      )) return;
      renderLayers();
      publishReviewShareState();
    });
  }
  const intensityRange = document.getElementById("intensity-range") as HTMLSelectElement | null;
  if (intensityRange) {
    intensityRange.addEventListener("change", () => {
      if (!viewer.setIntensityRangeMode(intensityRange.value as "auto" | "manual")) return;
      renderLayers();
      publishActiveReviewLayerState();
    });
  }
  const applyIntensityRange = (): void => {
    const minimum = Number((document.getElementById("intensity-min") as HTMLInputElement | null)?.value);
    const maximum = Number((document.getElementById("intensity-max") as HTMLInputElement | null)?.value);
    if (!viewer.setIntensityRange(minimum, maximum)) return;
    publishActiveReviewLayerState();
  };
  document.getElementById("intensity-min")?.addEventListener("change", applyIntensityRange);
  document.getElementById("intensity-max")?.addEventListener("change", applyIntensityRange);
  requiredInput<HTMLInputElement>("point-size").addEventListener(
    "input",
    (event) => {
      const input = event.currentTarget as HTMLInputElement;
      const pointSize = Number.isFinite(Number(input.value))
        ? Math.max(0.05, Math.min(Number(input.value), 5))
        : 1.5;
      input.value = pointSize.toFixed(2);
      viewer.setPointSize(pointSize);
      publishActiveReviewLayerState();
      const output = document.getElementById("point-size-value");
      if (output) output.textContent = pointSize.toFixed(2);
    },
  );
  document.getElementById("save-screenshot")?.addEventListener("click", async () => {
    if (screenshotRequestId !== undefined) return;
    const identity = reviewActionIdentity();
    try {
      const blob = await viewer.capturePng();
      if (!reviewActionStillCurrent(identity)) return;
      if (blob.size === 0 || blob.size > maximumScreenshotBytes) {
        showStatus(`Screenshot exceeds ${(maximumScreenshotBytes / 1024 / 1024).toFixed(0)} MiB limit`, "error");
        return;
      }
      const bytes = await blob.arrayBuffer();
      if (!reviewActionStillCurrent(identity)) return;
      if (bytes.byteLength === 0 || bytes.byteLength > maximumScreenshotBytes) {
        showStatus("Screenshot bytes are invalid", "error");
        return;
      }
      screenshotRequestId = ++nextScreenshotRequest;
      vscode.postMessage({
        type: "saveScreenshot",
        requestId: screenshotRequestId,
        suggestedName: screenshotName(currentCloudName),
        bytes,
        ...(identity ?? {}),
      });
      showStatus(localized("screenshotSaving", "Saving screenshot…"), "loading");
    } catch (error) {
      showStatus(error instanceof Error ? error.message : String(error), "error");
    }
  });
  const displayToggle = document.getElementById("display-toggle");
  const overlayMenu = document.getElementById("overlay-menu");
  if (displayToggle instanceof HTMLButtonElement && overlayMenu) {
    const closeOverlayMenu = (): void => {
      overlayMenu.hidden = true;
      displayToggle.setAttribute("aria-expanded", "false");
    };
    displayToggle.addEventListener("click", () => {
      const open = overlayMenu.hidden;
      overlayMenu.hidden = !open;
      displayToggle.setAttribute("aria-expanded", String(open));
    });
    document.addEventListener("pointerdown", (event) => {
      if (!overlayMenu.hidden && event.target instanceof Node &&
          !overlayMenu.contains(event.target) &&
          !displayToggle.contains(event.target)) {
        closeOverlayMenu();
      }
    });
    document.addEventListener("keydown", (event) => {
      if (event.key === "Escape" && !overlayMenu.hidden) {
        closeOverlayMenu();
        displayToggle.focus();
      }
    });
  }
  const detailsToggle = document.getElementById("details-toggle");
  const information = document.getElementById("information");
  if (detailsToggle instanceof HTMLButtonElement && information) {
    detailsToggle.addEventListener("click", () => {
      const open = information.hidden;
      information.hidden = !open;
      detailsToggle.setAttribute("aria-expanded", String(open));
    });
  }
  const inspectionToggle = document.getElementById("inspection-toggle");
  const inspectionPanel = document.getElementById("inspection-panel");
  if (inspectionToggle instanceof HTMLButtonElement && inspectionPanel) {
    const closeInspection = (): void => {
      inspectionPanel.hidden = true;
      inspectionToggle.setAttribute("aria-expanded", "false");
    };
    inspectionToggle.addEventListener("click", () => {
      const open = inspectionPanel.hidden;
      inspectionPanel.hidden = !open;
      inspectionToggle.setAttribute("aria-expanded", String(open));
    });
    document.addEventListener("keydown", (event) => {
      if (event.key === "Escape" && !inspectionPanel.hidden) {
        closeInspection();
        inspectionToggle.focus();
      }
    });
  }
  const measureToggle = document.getElementById("measure-toggle");
  if (measureToggle instanceof HTMLButtonElement) {
    measureToggle.addEventListener("click", () => {
      const enabled = measureToggle.getAttribute("aria-pressed") !== "true";
      viewer.setMeasurementEnabled(enabled);
      measureToggle.setAttribute("aria-pressed", String(enabled));
      measureToggle.textContent = enabled
        ? localized("measurementStop", "Stop measuring")
        : localized("measurementStart", "Measure");
      if (enabled) renderMeasurement();
    });
  }
  document.getElementById("clear-measurement")?.addEventListener("click", () => {
    measurements = [];
    importedReviewMeasurementsDirty = true;
    viewer.setMeasurement(measurements);
    renderMeasurement();
    publishReviewShareState();
  });
  document.getElementById("apply-roi")?.addEventListener("click", () => {
    const roi = roiFromInputs();
    if (!roi || !setReviewRoi(roi)) {
      const output = document.getElementById("roi-result");
      if (output) output.textContent = localized(
        "roiInvalid", "ROI needs finite min ≤ max on every axis.",
      );
      return;
    }
    renderRoi();
    publishReviewShareState();
  });
  document.getElementById("reset-roi")?.addEventListener("click", () => {
    if (setReviewRoi(undefined)) {
      renderRoi();
      publishReviewShareState();
    }
  });
  document.getElementById("export-roi")?.addEventListener("click", async () => {
    if (exportRequestId !== undefined) return;
    const identity = reviewActionIdentity();
    const result = viewer.prepareVisiblePlyExport();
    const output = document.getElementById("roi-result");
    if (result.error || !result.blob || result.count === 0) {
      if (output) output.textContent = result.error ?? localized(
        "roiEmpty", "No finite point in ROI.",
      );
      return;
    }
    if (result.blob.size > maximumTransportBytes) {
      if (output) output.textContent = `PLY export exceeds ${(maximumTransportBytes /
        1024 / 1024).toFixed(0)} MiB transport limit; narrow the ROI.`;
      return;
    }
    try {
      const bytes = await result.blob.arrayBuffer();
      if (!reviewActionStillCurrent(identity)) return;
      if (bytes.byteLength === 0 || bytes.byteLength > maximumTransportBytes) {
        if (output) output.textContent = "PLY export exceeds transport limit; narrow the ROI.";
        return;
      }
      exportRequestId = ++nextExportRequest;
      vscode.postMessage({
        type: "exportPly",
        requestId: exportRequestId,
        suggestedName: roiExportName(currentCloudName),
        pointCount: result.count,
        bytes,
        ...(identity ?? {}),
      });
      if (output) output.textContent = localized("roiExporting", "Saving PLY…");
      renderRoi();
    } catch (error) {
      if (output) output.textContent = error instanceof Error
        ? error.message : String(error);
    }
  });
  const bookmarkList = document.getElementById("bookmark-list") as HTMLSelectElement | null;
  bookmarkList?.addEventListener("change", renderBookmarks);
  document.getElementById("bookmark-save")?.addEventListener("click", () => {
    const defaultName = formatLocalized(
      "bookmarkDefault", [String(bookmarks.length + 1)], "View {0}",
    );
    const name = window.prompt(localized("bookmarkName", "Bookmark name"), defaultName);
    if (name === null) return;
    const normalized = name.trim().replace(/[\u0000-\u001f\u007f]/gu, "").slice(0, 80);
    if (!normalized) return;
    bookmarks = [...bookmarks.slice(-99), {
      name: normalized,
      camera: viewer.getCameraBookmark(),
    }];
    persistInspection();
    renderBookmarks();
    publishReviewShareState();
  });
  document.getElementById("bookmark-restore")?.addEventListener("click", () => {
    const index = Number(bookmarkList?.value);
    const bookmark = Number.isInteger(index) ? bookmarks[index] : undefined;
    if (!bookmark || !viewer.restoreCameraBookmark(bookmark.camera)) return;
    renderBookmarks();
  });
  document.getElementById("bookmark-remove")?.addEventListener("click", () => {
    const index = Number(bookmarkList?.value);
    if (!Number.isInteger(index) || !bookmarks[index]) return;
    bookmarks = bookmarks.filter((_, candidate) => candidate !== index);
    persistInspection();
    renderBookmarks();
    publishReviewShareState();
  });
  document.getElementById("export-review-share")?.addEventListener("click", () => {
    if (shareRequestId !== undefined) return;
    try {
      const identity = reviewActionIdentity();
      shareRequestId = ++nextShareRequest;
      vscode.postMessage({
        type: "exportReviewShare",
        requestId: shareRequestId,
        suggestedName: reviewShareName(currentCloudName),
        document: captureReviewShare(),
        ...(identity ?? {}),
      });
      showStatus(localized("reviewShareSaving", "Saving Review Share…"), "loading");
    } catch (error) {
      shareRequestId = undefined;
      showStatus(error instanceof Error ? error.message : String(error), "error");
    }
  });
  document.getElementById("import-review-share")?.addEventListener("click", () => {
    if (shareRequestId !== undefined) return;
    const identity = reviewActionIdentity();
    shareRequestId = ++nextShareRequest;
    vscode.postMessage({
      type: "importReviewShare",
      requestId: shareRequestId,
      ...(identity ?? {}),
    });
    showStatus(localized("reviewShareImporting", "Choosing Review Share…"), "loading");
  });
  renderBookmarks();
  renderMeasurement();
  renderLayers();
  installDraggableOverlays([
    document.getElementById("toolbar"),
    information,
    overlayMenu,
    document.getElementById("controls-help"),
    document.getElementById("player"),
    inspectionPanel,
  ]);
  requiredInput<HTMLInputElement>("background").addEventListener(
    "input",
    (event) => viewer.setBackground(
      (event.currentTarget as HTMLInputElement).value,
    ),
  );
  document.querySelector<HTMLInputElement>("#fixed-color")?.addEventListener(
    "input",
    (event) => {
      viewer.setFixedColor((event.currentTarget as HTMLInputElement).value);
      publishActiveReviewLayerState();
    },
  );
  document.querySelector<HTMLInputElement>("#noise-color")?.addEventListener(
    "input",
    (event) => {
      viewer.setNoiseColor((event.currentTarget as HTMLInputElement).value);
      publishActiveReviewLayerState();
    },
  );
  document.querySelector<HTMLInputElement>("#highlight-noise")?.addEventListener(
    "change",
    (event) => {
      viewer.setNoiseHighlight((event.currentTarget as HTMLInputElement).checked);
      publishActiveReviewLayerState();
    },
  );
  requiredInput<HTMLInputElement>("show-axes").addEventListener(
    "change",
    (event) => viewer.setAxesVisible(
      (event.currentTarget as HTMLInputElement).checked,
    ),
  );
  requiredInput<HTMLInputElement>("show-grid").addEventListener(
    "change",
    (event) => viewer.setGridVisible(
      (event.currentTarget as HTMLInputElement).checked,
    ),
  );
  document.querySelectorAll<HTMLButtonElement>("[data-view]").forEach(
    (button) => button.addEventListener(
      "click",
      () => viewer.setView(button.dataset.view as StandardView),
    ),
  );
  const activeLayerKey = (): string | undefined => selectedUnresolvedSourceKey
    ? undefined : viewer.getActiveLayerKey();
  (document.getElementById("layer-list") as HTMLSelectElement | null)?.addEventListener(
    "change",
    (event) => {
      const runtimeId = (event.currentTarget as HTMLSelectElement).value;
      if (viewer.setActiveLayer(runtimeId)) {
        selectedUnresolvedSourceKey = undefined;
        renderLayers();
        renderRoi();
        return;
      }
      const unresolved = [...importedReviewLayers.values()].find((layer) =>
        layer.runtime_id === runtimeId);
      selectedUnresolvedSourceKey = unresolved?.source_key;
      renderLayers();
    },
  );
  document.getElementById("layer-visible")?.addEventListener("change", (event) => {
    const runtimeId = activeLayerKey();
    if (!runtimeId) return;
    viewer.setLayerVisible(runtimeId, (event.currentTarget as HTMLInputElement).checked);
    publishActiveReviewLayerState(runtimeId);
    renderLayers();
    renderRoi();
  });
  document.getElementById("layer-opacity")?.addEventListener("input", (event) => {
    const runtimeId = activeLayerKey();
    if (!runtimeId) return;
    viewer.setLayerOpacity(runtimeId, Number((event.currentTarget as HTMLInputElement).value));
    publishActiveReviewLayerState(runtimeId);
    renderLayers();
  });
  document.getElementById("layer-size")?.addEventListener("input", (event) => {
    const runtimeId = activeLayerKey();
    if (!runtimeId) return;
    viewer.setLayerPointSize(runtimeId, Number((event.currentTarget as HTMLInputElement).value));
    publishActiveReviewLayerState(runtimeId);
    renderLayers();
  });
  document.getElementById("layer-color")?.addEventListener("input", (event) => {
    const runtimeId = activeLayerKey();
    if (!runtimeId) return;
    viewer.setLayerFixedColor(runtimeId, (event.currentTarget as HTMLInputElement).value);
    publishActiveReviewLayerState(runtimeId);
    renderLayers();
  });
  document.getElementById("apply-layer-transform")?.addEventListener("click", () => {
    const runtimeId = activeLayerKey();
    if (!runtimeId) return;
    const vector = (kind: "pos" | "rot" | "scale"): [number, number, number] =>
      ["x", "y", "z"].map((axis) => Number(
        (document.getElementById(`layer-${kind}-${axis}`) as HTMLInputElement | null)?.value,
      )) as [number, number, number];
    const transform: LayerTransform = {
      position: vector("pos"), rotation: vector("rot"), scale: vector("scale"),
    };
    if (viewer.setLayerTransform(runtimeId, transform)) {
      publishActiveReviewLayerState(runtimeId);
      renderLayers();
      renderRoi();
    } else {
      const output = document.getElementById("layer-transform-state");
      if (output) output.textContent = localized(
        "invalidLayerTransform", "Transform is invalid for this layer.",
      );
    }
  });
  document.getElementById("remove-layer")?.addEventListener("click", () => {
    const unresolvedSourceKey = selectedUnresolvedSourceKey;
    if (unresolvedSourceKey) {
      if (!importedReviewLayers.delete(unresolvedSourceKey)) return;
      tombstoneReviewSource(unresolvedSourceKey);
      selectedUnresolvedSourceKey = undefined;
      vscode.postMessage({
        type: "removeLayer",
        sourceKey: unresolvedSourceKey,
        ...(reviewActionIdentity() ?? {}),
      });
      publishReviewShareState();
      renderLayers();
      renderRoi();
      renderMeasurement();
      return;
    }
    const runtimeId = activeLayerKey();
    if (!runtimeId) return;
    const sourceKey = viewer.getLayers().find((layer) =>
      layer.runtimeId === runtimeId)?.sourceKey;
    if (viewer.removeLayer(runtimeId)) {
      // The source key is opaque; extension host removes only matching
      // host-catalog entries and never receives a path from the webview.
      if (sourceKey) {
        // Keep the current semantic set authoritative too. Otherwise a late
        // imported replay could still find this stale entry and recreate it.
        importedReviewLayers.delete(sourceKey);
        tombstoneReviewSource(sourceKey);
      }
      if (sourceKey) vscode.postMessage(reviewSessionActive ? {
        type: "removeLayer",
        sourceKey,
        sessionGeneration: latestReviewSessionGeneration,
        replayEpoch: latestReviewReplayEpoch,
      } : { type: "removeLayer", sourceKey });
      publishReviewShareState();
      renderLayers();
      renderRoi();
      renderMeasurement();
    }
  });
  document.getElementById("fit-active")?.addEventListener("click", () => {
    viewer.setView("fit");
  });
  document.getElementById("fit-visible")?.addEventListener("click", () => {
    viewer.fitVisible();
  });
  document.getElementById("add-layers")?.addEventListener("click", () => {
    const requestId = ++nextLayerRequest;
    vscode.postMessage(reviewSessionActive ? {
      type: "addLayers",
      requestId,
      sessionGeneration: latestReviewSessionGeneration,
      replayEpoch: latestReviewReplayEpoch,
    } : { type: "addLayers", requestId });
  });
  document.getElementById("locate-review-source")?.addEventListener("click", () => {
    const sourceKey = selectedUnresolvedSourceKey;
    if (!sourceKey || !importedReviewLayers.has(sourceKey)) return;
    vscode.postMessage({
      type: "locateReviewSource",
      requestId: ++nextLayerRequest,
      sourceKey,
      sessionGeneration: latestReviewSessionGeneration,
      replayEpoch: latestReviewReplayEpoch,
    });
    showStatus(localized("reviewShareLocating", "Choosing Review Share source…"), "loading");
  });
  requiredInput<HTMLButtonElement>("reload").addEventListener("click", () => {
    // This post is deliberately first. The host consumes the exact current
    // semantic snapshot before it starts a new same-session replay epoch.
    publishReviewShareState();
    cancelLayerDecodes(localized("layerReloadCancelled", "Layer load cancelled by reload."));
    worker?.terminate();
    worker = undefined;
    workerBusy = false;
    activeWorkerRequest = undefined;
    pendingLoads.length = 0;
    clearDecodeTimeouts();
    ++activeRequest;
    preserveLayersOnNextPrimaryLoad = primarySourceKey !== undefined &&
      viewer.hasLayer(primarySourceKey);
    primaryReady = false;
    showStatus(localized("reloading", "Reloading…"), "loading");
    if (frameCount > 0) {
      ++sequenceGeneration;
      frameCache.clear();
      requestedFrames.clear();
      requestFrame(currentFrame);
    } else {
      vscode.postMessage(reviewSessionActive ? {
        type: "reload",
        sessionGeneration: latestReviewSessionGeneration,
        replayEpoch: latestReviewReplayEpoch,
      } : { type: "reload" });
    }
  });
  requiredInput<HTMLInputElement>("frame").addEventListener(
    "input",
    (event) => selectFrame(
      Number((event.currentTarget as HTMLInputElement).value),
    ),
  );
  const stopPlayback = (): void => {
    const button = requiredInput<HTMLButtonElement>("play");
    if (playback !== undefined) {
      window.clearInterval(playback);
      playback = undefined;
    }
    button.textContent = "▶";
  };
  const startPlayback = (direction: 1 | -1): void => {
    stopPlayback();
    playbackDirection = direction;
    const rate = Number(requiredInput<HTMLSelectElement>("rate").value);
    playback = window.setInterval(
      () => {
        const next = currentFrame + playbackDirection;
        if (next >= 0 && next < frameCount) {
          selectFrame(next);
          return;
        }
        if ((document.getElementById("loop-playback") as HTMLInputElement | null)?.checked) {
          selectFrame(playbackDirection > 0 ? 0 : frameCount - 1);
          return;
        }
        stopPlayback();
      },
      1000 / rate,
    );
    requiredInput<HTMLButtonElement>("play").textContent = "⏸";
  };
  requiredInput<HTMLButtonElement>("play").addEventListener("click", () => {
    if (playback !== undefined) stopPlayback();
    else startPlayback(1);
  });
  document.getElementById("reverse-play")?.addEventListener("click", () => {
    if (playback !== undefined && playbackDirection === -1) stopPlayback();
    else startPlayback(-1);
  });
  document.getElementById("previous-frame")?.addEventListener("click", () => {
    stopPlayback();
    selectFrame(Math.max(0, currentFrame - 1));
  });
  document.getElementById("next-frame")?.addEventListener("click", () => {
    stopPlayback();
    selectFrame(Math.min(frameCount - 1, currentFrame + 1));
  });
  document.getElementById("reset-playback")?.addEventListener("click", () => {
    stopPlayback();
    selectFrame(0);
  });
  requiredInput<HTMLSelectElement>("rate").addEventListener("change", () => {
    if (playback !== undefined) startPlayback(playbackDirection);
  });

  vscode.postMessage({ type: "ready" });
}

async function fetchBounded(
  uri: string,
  maximumBytes: number,
  description: string,
): Promise<ArrayBuffer> {
  const controller = new AbortController();
  const timeout = window.setTimeout(() => controller.abort(), 15_000);
  try {
    const response = await fetch(uri, { signal: controller.signal });
    if (!response.ok)
      throw new Error(`cannot load ${description} (${response.status})`);
    const contentLength = Number(response.headers.get("content-length"));
    if (Number.isSafeInteger(contentLength) && contentLength > maximumBytes)
      throw new Error(`${description} exceeds memory limit`);
    const bytes = await response.arrayBuffer();
    if (bytes.byteLength === 0 || bytes.byteLength > maximumBytes)
      throw new Error(`${description} exceeds memory limit`);
    return bytes;
  } catch (error) {
    if (controller.signal.aborted)
      throw new Error(`cannot load ${description} (timed out)`);
    throw error;
  } finally {
    window.clearTimeout(timeout);
  }
}

interface StoredBookmark {
  name: string;
  camera: CameraBookmark;
}

interface InspectionState {
  version: 1;
  bookmarks: StoredBookmark[];
}

function readInspectionState(value: unknown): InspectionState {
  if (!value || typeof value !== "object") return { version: 1, bookmarks: [] };
  const state = value as Record<string, unknown>;
  if (state.version !== 1 || !Array.isArray(state.bookmarks))
    return { version: 1, bookmarks: [] };
  const bookmarks: StoredBookmark[] = [];
  for (const value of state.bookmarks.slice(-100)) {
    if (!value || typeof value !== "object") continue;
    const bookmark = value as Record<string, unknown>;
    const camera = storedCameraBookmark(bookmark.camera);
    if (typeof bookmark.name !== "string" || bookmark.name.length === 0 ||
        bookmark.name.length > 80 || !camera) continue;
    bookmarks.push({ name: bookmark.name, camera });
  }
  return { version: 1, bookmarks };
}

function mergeBookmarks(
  existing: readonly StoredBookmark[],
  incoming: readonly StoredBookmark[],
): StoredBookmark[] {
  const merged = new Map<string, StoredBookmark>();
  for (const bookmark of existing) {
    if (isCameraBookmark(bookmark.camera)) merged.set(bookmark.name, bookmark);
  }
  for (const bookmark of incoming) {
    if (!isCameraBookmark(bookmark.camera)) continue;
    // Delete first so an incoming replacement also occupies a deterministic
    // latest position if capacity evicts older entries.
    merged.delete(bookmark.name);
    merged.set(bookmark.name, bookmark);
  }
  return [...merged.values()].slice(-100);
}

function isCameraBookmark(value: unknown): value is CameraBookmark {
  if (!value || typeof value !== "object") return false;
  const bookmark = value as Record<string, unknown>;
  return validVector(bookmark.position) && validVector(bookmark.target) &&
    validVector(bookmark.rotationCenter) && validVector(bookmark.up) &&
    hasPortableCameraFov(bookmark.fov);
}

/**
 * Local webview state predates Review Share v2's separate rotation center.
 * Preserve old bookmarks by treating their target as the historic pivot, then
 * persist the complete state on the next save.
 */
function storedCameraBookmark(value: unknown): CameraBookmark | undefined {
  if (!value || typeof value !== "object") return undefined;
  const bookmark = value as Record<string, unknown>;
  if (!validVector(bookmark.position) || !validVector(bookmark.target) ||
      !validVector(bookmark.up) || typeof bookmark.fov !== "number" ||
      !hasPortableCameraFov(bookmark.fov)) {
    return undefined;
  }
  const rotationCenter = validVector(bookmark.rotationCenter)
    ? bookmark.rotationCenter
    : bookmark.target;
  const normalized: CameraBookmark = {
    position: [...bookmark.position] as [number, number, number],
    target: [...bookmark.target] as [number, number, number],
    rotationCenter: [...rotationCenter] as [number, number, number],
    up: [...bookmark.up] as [number, number, number],
    fov: bookmark.fov,
  };
  return isCameraBookmark(normalized) ? normalized : undefined;
}

function cameraBookmarkFromReview(
  camera: ReviewShareDocument["bookmarks"][number]["camera"],
): CameraBookmark | undefined {
  const target = camera.target;
  const up = [
    camera.camera_to_world[0][1],
    camera.camera_to_world[1][1],
    camera.camera_to_world[2][1],
  ] as [number, number, number];
  const backward = [
    camera.camera_to_world[0][2],
    camera.camera_to_world[1][2],
    camera.camera_to_world[2][2],
  ] as [number, number, number];
  const position = target.map((value, axis) =>
    value + backward[axis] * camera.distance,
  ) as [number, number, number];
  const bookmark: CameraBookmark = {
    position,
    target: [...target] as [number, number, number],
    rotationCenter: [...camera.rotation_center] as [number, number, number],
    up,
    fov: camera.fov_y_degrees,
  };
  return isCameraBookmark(bookmark) ? bookmark : undefined;
}

function validVector(value: unknown): value is [number, number, number] {
  return Array.isArray(value) && value.length === 3 && value.every(Number.isFinite);
}

function roiFields(): {
  minimum: Array<HTMLInputElement | null>;
  maximum: Array<HTMLInputElement | null>;
} {
  return {
    minimum: ["x", "y", "z"].map((axis) =>
      document.getElementById(`roi-min-${axis}`) as HTMLInputElement | null),
    maximum: ["x", "y", "z"].map((axis) =>
      document.getElementById(`roi-max-${axis}`) as HTMLInputElement | null),
  };
}

function roiFromInputs(): RoiBox | undefined {
  const fields = roiFields();
  if (fields.minimum.some((field) => !field) || fields.maximum.some((field) => !field))
    return undefined;
  const min = fields.minimum.map((field) => Number(field!.value));
  const max = fields.maximum.map((field) => Number(field!.value));
  if (!min.every(Number.isFinite) || !max.every(Number.isFinite) ||
      min.some((value, axis) => value > max[axis])) return undefined;
  return {
    min: min as RoiBox["min"],
    max: max as RoiBox["max"],
  };
}

function roiExportName(name: string): string {
  const stem = name.replace(/\.[^.]+$/u, "")
    .replace(/[\\/\u0000-\u001f\u007f-\u009f]/gu, "_")
    .trim() || "point-cloud";
  return `${stem}-roi.ply`;
}

function screenshotName(name: string): string {
  const stem = name.replace(/\.[^.]+$/u, "")
    .replace(/[\\/\u0000-\u001f\u007f-\u009f]/gu, "_")
    .trim() || "point-cloud";
  return `${stem}-review.png`;
}

function reviewShareName(name: string): string {
  const stem = name.replace(/\.[^.]+$/u, "")
    .replace(/[\\/\u0000-\u001f\u007f-\u009f]/gu, "_")
    .trim() || "point-cloud";
  return `${stem}-review.json`;
}

function decodeBase64Bounded(
  encoded: string,
  maximumBytes: number,
  description: string,
): ArrayBuffer {
  if (encoded.length === 0 || encoded.length > Math.ceil(maximumBytes / 3) * 4)
    throw new Error(`${description} exceeds memory limit`);
  const decoded = atob(encoded);
  if (decoded.length === 0 || decoded.length > maximumBytes)
    throw new Error(`${description} exceeds memory limit`);
  const bytes = new Uint8Array(decoded.length);
  for (let index = 0; index < decoded.length; ++index)
    bytes[index] = decoded.charCodeAt(index);
  return bytes.buffer;
}

function validInteger(value: unknown, maximum = 0xffffffff): value is number {
  return typeof value === "number" && Number.isSafeInteger(value) &&
    value >= 0 && value <= maximum;
}

/** Review session and replay identities are an atomic pair on wire. */
function validReviewIdentity(
  sessionGeneration: unknown,
  replayEpoch: unknown,
): boolean {
  if (sessionGeneration === undefined && replayEpoch === undefined) return true;
  return validInteger(sessionGeneration) && sessionGeneration > 0 &&
    validInteger(replayEpoch) && replayEpoch > 0;
}

function validText(value: unknown, maximumBytes: number): value is string {
  return typeof value === "string" && value.length > 0 &&
    new TextEncoder().encode(value).byteLength <= maximumBytes &&
    !/[\u0000-\u001f\u007f-\u009f]/u.test(value);
}

function validName(value: unknown): value is string {
  return validText(value, maximumNameBytes) &&
    !/[\\/]/u.test(value);
}

function validSourceKey(value: unknown): value is string {
  return hasValidSourceKeyByteLength(value) &&
    /^sha256:[a-f0-9]{64}$/u.test(value);
}

function validReviewLayerState(value: unknown): boolean {
  return validateReviewShareStateLayer(value);
}

function validReviewShareState(value: unknown): value is ReviewShareState {
  if (!value || typeof value !== "object") return false;
  const state = value as Record<string, unknown>;
  if (!Array.isArray(state.layers) || !Array.isArray(state.measurements)) return false;
  const document = {
    schema_version: state.schema_version,
    intensity_scale_mode: state.intensity_scale_mode,
    layers: state.layers.map((layer) => {
      if (!layer || typeof layer !== "object") return layer;
      const candidate = layer as Record<string, unknown>;
      return {
        source_key: candidate.source_key,
        source_path: null,
        local_to_world: candidate.local_to_world,
        style: candidate.style,
        visible: candidate.visible,
      };
    }),
    roi: state.roi,
    measurements: state.measurements,
    bookmarks: state.bookmarks,
  };
  return validateReviewShare(document) && state.layers.every((layer) => {
    if (!layer || typeof layer !== "object") return false;
    const candidate = layer as Record<string, unknown>;
    // `validateReviewShare` deliberately accepts native logical `path:` and
    // `opaque:` identities at the host/file boundary. They must not cross
    // into the webview, where a path payload could disclose a Remote source.
    // A host-sanitized state therefore admits only transport SHA-256 keys and
    // has no source_path field at all.
    return validSourceKey(candidate.source_key) &&
      !("source_path" in candidate) && validName(candidate.name) &&
      typeof candidate.runtime_id === "string" &&
      /^review-[0-9]+-[0-9]+$/u.test(candidate.runtime_id);
  }) && state.measurements.every((measurement) => {
    if (!measurement || typeof measurement !== "object") return false;
    const candidate = measurement as Record<string, unknown>;
    return validSourceKey(candidate.first_source_key) &&
      (candidate.second_source_key === null ||
        validSourceKey(candidate.second_source_key));
  });
}

function validFrameFields(value: Record<string, unknown>): boolean {
  const frame = value.frameIndex;
  const generation = value.generation;
  return (frame === undefined && generation === undefined) ||
    (validInteger(frame) && validInteger(generation));
}

function validExtensionMessage(value: unknown): value is ExtensionToWebviewMessage {
  if (!value || typeof value !== "object") return false;
  const message = value as Record<string, unknown>;
  if (typeof message.type !== "string") return false;
  if (message.type === "load") {
    const labelBytes = message.labelBytes;
    return validInteger(message.requestId) && validName(message.name) &&
      (message.sourceKey === undefined || validSourceKey(message.sourceKey)) &&
      message.bytes instanceof ArrayBuffer && message.bytes.byteLength > 0 &&
      message.bytes.byteLength <= maximumCloudBytes &&
      (labelBytes === undefined ||
       (labelBytes instanceof ArrayBuffer && labelBytes.byteLength > 0 &&
        labelBytes.byteLength <= maximumLabelBytes)) &&
      message.bytes.byteLength <= maximumTransportBytes -
        (labelBytes instanceof ArrayBuffer ? labelBytes.byteLength : 0) &&
      validFrameFields(message);
  }
  if (message.type === "addLayer") {
    return validInteger(message.requestId) && validSourceKey(message.sourceKey) &&
      validName(message.name) && message.bytes instanceof ArrayBuffer &&
      message.bytes.byteLength > 0 && message.bytes.byteLength <= maximumCloudBytes &&
      validReviewIdentity(message.sessionGeneration, message.replayEpoch) &&
      (message.sourceRevision === undefined || validInteger(message.sourceRevision)) &&
      (message.reviewLayer === undefined ||
        (message.sessionGeneration !== undefined && message.replayEpoch !== undefined &&
          validReviewLayerState(message.reviewLayer)));
  }
  if (message.type === "hostError") {
    return validInteger(message.requestId) &&
      typeof message.message === "string" && message.message.length <= 16_384 &&
      (message.primary === undefined || typeof message.primary === "boolean") &&
      validFrameFields(message);
  }
  if (message.type === "exportedPly") {
    return validInteger(message.requestId) &&
      validInteger(message.pointCount, 20_000_000) && validName(message.name);
  }
  if (message.type === "screenshotSaved") {
    return validInteger(message.requestId) && validName(message.name);
  }
  if (message.type === "reviewShareSaved") {
    return validInteger(message.requestId) && validName(message.name);
  }
  if (message.type === "reviewShareLoaded") {
    return validInteger(message.requestId) &&
      validInteger(message.sessionGeneration) && message.sessionGeneration > 0 &&
      validInteger(message.replayEpoch) && message.replayEpoch > 0 &&
      validReviewShareState(message.document);
  }
  if (message.type !== "sequenceCatalog" ||
      !validInteger(message.frameCount, 100_000) || message.frameCount < 1 ||
      !validName(message.name) || !Array.isArray(message.trajectories) ||
      !Array.isArray(message.framePoses) ||
      message.trajectories.length > message.frameCount ||
      message.framePoses.length > message.frameCount) return false;
  let trajectoryPoints = 0;
  for (const trajectory of message.trajectories) {
    if (!Array.isArray(trajectory) || trajectory.length > 1_000_000)
      return false;
    trajectoryPoints += trajectory.length;
    if (trajectoryPoints > 200_000) return false;
    if (trajectory.some((point) => !Array.isArray(point) || point.length !== 3 ||
        point.some((coordinate) => !Number.isFinite(coordinate)))) return false;
  }
  return message.framePoses.every((matrix) =>
    Array.isArray(matrix) && matrix.length === 16 &&
    matrix.every((value) => Number.isFinite(value)));
}

function validWorkerResponse(value: unknown): value is WorkerResponse {
  if (!value || typeof value !== "object") return false;
  const message = value as Record<string, unknown>;
  if (message.type === "decodeStarted") {
    return validInteger(message.requestId) && validFrameFields(message);
  }
  if (message.type === "decodeError") {
    return validInteger(message.requestId) && validFrameFields(message) &&
      typeof message.message === "string" && message.message.length <= 16_384 &&
      ["unsupported-format", "invalid-input", "point-limit", "memory-limit",
        "internal-error"].includes(String(message.code));
  }
  if (message.type !== "decoded" || !validInteger(message.requestId) ||
      !validInteger(message.pointCount, 10_000_000) ||
      !validFrameFields(message)) return false;
  const count = message.pointCount;
  const positions = message.positions;
  const colors = message.colors;
  const intensities = message.intensities;
  const noises = message.noises;
  const pointOrder = message.pointOrder;
  const chunkRanges = message.chunkRanges;
  const lodIndices = message.lodIndices;
  const hasColor = message.hasColor;
  const hasNoise = message.hasNoise;
  const decodeMilliseconds = message.decodeMilliseconds;
  const indexMilliseconds = message.indexMilliseconds;
  if (!(positions instanceof Float32Array) || positions.length !== count * 3 ||
      !(colors instanceof Uint8Array) ||
      colors.length !== (hasColor ? count * 3 : 0) ||
      !(intensities instanceof Float32Array) || intensities.length !== count ||
      !(noises instanceof Uint8Array) ||
      noises.length !== (hasNoise ? count : 0) ||
      !(pointOrder instanceof Uint32Array) || pointOrder.length !== count ||
      !(chunkRanges instanceof Uint32Array) || chunkRanges.length % 2 !== 0 ||
      chunkRanges.length > maximumChunks * 2 ||
      !(lodIndices instanceof Uint32Array) ||
      lodIndices.length > count || typeof hasColor !== "boolean" ||
      typeof hasNoise !== "boolean" || typeof message.hasIntensity !== "boolean" ||
      typeof message.noiseCount !== "number" || message.noiseCount < 0 ||
      message.noiseCount > count || typeof message.defaultColorMode !== "string") {
    return false;
  }
  if (!["rgb", "intensity", "height"].includes(message.defaultColorMode) ||
      !Number.isFinite(message.noiseCount) ||
      typeof decodeMilliseconds !== "number" ||
      !Number.isFinite(decodeMilliseconds) || decodeMilliseconds < 0 ||
      typeof indexMilliseconds !== "number" || !Number.isFinite(indexMilliseconds) ||
      indexMilliseconds < 0 ||
      frameBytes(message as unknown as DecodedCloudMessage) > maximumDecodedBytes ||
      !validBounds(message.bounds)) return false;
  for (const index of pointOrder) if (index >= count) return false;
  for (const index of lodIndices) if (index >= count) return false;
  for (let index = 0; index < chunkRanges.length; index += 2) {
    const start = chunkRanges[index];
    const length = chunkRanges[index + 1];
    if (start > count || length > count - start) return false;
  }
  return true;
}

function validBounds(value: unknown): boolean {
  if (value === null) return true;
  if (!value || typeof value !== "object") return false;
  const bounds = value as Record<string, unknown>;
  if (!Array.isArray(bounds.min) || !Array.isArray(bounds.max) ||
      bounds.min.length !== 3 || bounds.max.length !== 3) return false;
  return bounds.min.every(Number.isFinite) && bounds.max.every(Number.isFinite) &&
    bounds.min.every((minimum, axis) =>
      minimum <= (bounds.max as number[])[axis]);
}

function frameBytes(message: DecodedCloudMessage): number {
  return message.positions.byteLength + message.colors.byteLength +
    message.intensities.byteLength + message.noises.byteLength +
    message.pointOrder.byteLength +
    message.chunkRanges.byteLength + message.lodIndices.byteLength;
}

function transportBytes(message: LoadCloudMessage | WorkerRequest): number {
  return message.bytes.byteLength + (message.labelBytes?.byteLength ?? 0) +
    ("wasmBinary" in message ? message.wasmBinary?.byteLength ?? 0 : 0);
}

function requiredElement(id: string): HTMLElement {
  const element = document.getElementById(id);
  if (!element) throw new Error(`missing #${id}`);
  return element;
}

function requiredInput<T extends HTMLElement>(id: string): T {
  return requiredElement(id) as T;
}

function installDraggableOverlays(
  candidates: Array<HTMLElement | null>,
): void {
  let nextZIndex = 10;
  for (const element of candidates) {
    if (!element) continue;
    const handle = element.querySelector<HTMLElement>("[data-drag-handle]");
    if (!handle) continue;
    let pointerId: number | undefined;
    let originX = 0;
    let originY = 0;
    let startX = 0;
    let startY = 0;

    const clamp = (): void => {
      if (element.hidden || getComputedStyle(element).display === "none") return;
      const rect = element.getBoundingClientRect();
      const left = Math.max(0, Math.min(rect.left, window.innerWidth - rect.width));
      const top = Math.max(0, Math.min(rect.top, window.innerHeight - rect.height));
      element.style.left = `${left}px`;
      element.style.top = `${top}px`;
      element.style.right = "auto";
      element.style.bottom = "auto";
      element.style.transform = "none";
    };

    handle.addEventListener("pointerdown", (event) => {
      if (event.button !== 0) return;
      const rect = element.getBoundingClientRect();
      originX = rect.left;
      originY = rect.top;
      startX = event.clientX;
      startY = event.clientY;
      pointerId = event.pointerId;
      clamp();
      element.style.zIndex = String(++nextZIndex);
      handle.setPointerCapture(pointerId);
      event.preventDefault();
    });
    handle.addEventListener("pointermove", (event) => {
      if (pointerId !== event.pointerId) return;
      element.style.left = `${originX + event.clientX - startX}px`;
      element.style.top = `${originY + event.clientY - startY}px`;
      clamp();
    });
    const finishDrag = (event: PointerEvent): void => {
      if (pointerId !== event.pointerId) return;
      if (handle.hasPointerCapture(pointerId)) handle.releasePointerCapture(pointerId);
      pointerId = undefined;
    };
    handle.addEventListener("pointerup", finishDrag);
    handle.addEventListener("pointercancel", finishDrag);
    window.addEventListener("resize", clamp);
  }
}

function showStatus(
  message: string,
  kind: "loading" | "ready" | "error",
): void {
  const status = requiredElement("status");
  status.textContent = message;
  status.dataset.kind = kind;
}

function showCloudInfo(
  message: Pick<DecodedCloudMessage,
    "bounds" | "hasNoise" | "noiseCount" | "pointCount">,
  gridSpacing: number,
): void {
  const info = requiredElement("cloud-info");
  info.hidden = false;
  const noiseInfo = document.getElementById("noise-info");
  if (noiseInfo) {
    noiseInfo.textContent = message.hasNoise
      ? formatLocalized("noiseAvailable", [
          message.noiseCount.toLocaleString(),
          message.pointCount.toLocaleString(),
        ], "Noise: {0} / {1}")
      : localized("noiseUnavailable", "Noise: unavailable");
  }
  if (!message.bounds) {
    requiredElement("aabb-min").textContent = localized(
      "minimumUnavailable", "Min: unavailable",
    );
    requiredElement("aabb-max").textContent = localized(
      "maximumUnavailable", "Max: unavailable",
    );
    requiredElement("aabb-size").textContent = localized(
      "sizeUnavailable", "Size: unavailable",
    );
    requiredElement("grid-spacing").textContent =
      formatLocalized("gridValue", [formatCoordinate(gridSpacing)],
        "Grid: {0} units / division");
    info.dataset.bounds = "unavailable";
    return;
  }
  const size = message.bounds.max.map(
    (value, axis) => value - message.bounds!.min[axis],
  );
  requiredElement("aabb-min").textContent =
    formatLocalized("minimumValue", [formatVector(message.bounds.min)], "Min: {0}");
  requiredElement("aabb-max").textContent =
    formatLocalized("maximumValue", [formatVector(message.bounds.max)], "Max: {0}");
  requiredElement("aabb-size").textContent =
    formatLocalized("sizeValue", [formatVector(size)], "Size: {0}");
  requiredElement("grid-spacing").textContent =
    formatLocalized("gridValue", [formatCoordinate(gridSpacing)],
      "Grid: {0} units / division");
  info.dataset.bounds = "available";
}

function formatVector(values: number[]): string {
  return `(${values.map(formatCoordinate).join(", ")})`;
}

function formatCoordinate(value: number): string {
  if (!Number.isFinite(value)) return "unavailable";
  if (Object.is(value, -0) || value === 0) return "0";
  return Number(value.toPrecision(6)).toString();
}

function localized(key: string, fallback: string): string {
  return localizedText[key] ?? fallback;
}

function formatLocalized(
  key: string,
  values: readonly string[],
  fallback: string,
): string {
  return values.reduce(
    (message, value, index) => message.replaceAll(`{${index}}`, value),
    localized(key, fallback),
  );
}
