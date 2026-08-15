import type {
  DecodedCloudMessage,
  ExtensionToWebviewMessage,
  LoadCloudMessage,
  WorkerRequest,
  WorkerResponse,
} from "../src/protocol";
import {
  maximumCloudBytes,
  maximumLabelBytes,
  maximumNameBytes,
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
  const layerRequests = new Map<number, {
    sourceKey: string;
    name: string;
    append: boolean;
  }>();
  const cacheBudget = 192 * 1024 * 1024;
  const restoredInspection = readInspectionState(vscode.getState?.());
  let bookmarks = restoredInspection.bookmarks;
  let measurements: PointPick[] = [];
  let currentCloudName = "point-cloud";

  const persistInspection = (): void => {
    vscode.setState?.({ version: 1, bookmarks });
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
    if (message) {
      value.textContent = message;
      return;
    }
    if (measurements.length === 0) {
      value.textContent = localized("measurementEmpty", "Click two points to measure.");
      return;
    }
    if (measurements.length === 1) {
      value.textContent = formatLocalized(
        "measurementFirst", [formatVector(measurements[0].point)],
        "First: {0}. Pick second point.",
      );
      return;
    }
    const [first, second] = measurements;
    const distance = Math.hypot(
      first.point[0] - second.point[0],
      first.point[1] - second.point[1],
      first.point[2] - second.point[2],
    );
    value.textContent = formatLocalized(
      "measurementDistance", [
        formatCoordinate(distance),
        formatVector(first.point),
        formatVector(second.point),
      ],
      "Distance: {0} · {1} → {2}",
    );
  };

  const renderRoi = (): void => {
    const output = document.getElementById("roi-result");
    const exportButton = document.getElementById("export-roi") as HTMLButtonElement | null;
    const count = viewer.getVisiblePointCount();
    if (output) output.textContent = viewer.getRoi()
      ? formatLocalized("roiCount", [count.toLocaleString()], "ROI: {0} points")
      : formatLocalized("roiInactive", [count.toLocaleString()], "Full cloud: {0} points");
    if (exportButton) exportButton.disabled = count === 0;
  };

  const renderLayers = (): void => {
    const list = document.getElementById("layer-list") as HTMLSelectElement | null;
    const remove = document.getElementById("remove-layer") as HTMLButtonElement | null;
    const activeKey = viewer.getActiveLayerKey();
    const layers = viewer.getLayers();
    if (list) {
      list.replaceChildren();
      for (const layer of layers) {
        const option = document.createElement("option");
        option.value = layer.sourceKey;
        option.textContent = `${layer.visible ? "●" : "○"} ${layer.name} · ${
          layer.pointCount.toLocaleString()
        }`;
        list.append(option);
      }
      list.value = activeKey ?? "";
      list.disabled = layers.length === 0;
    }
    if (remove) remove.disabled = !activeKey;
    const active = layers.find((layer) => layer.sourceKey === activeKey);
    syncLayerControls(active);
  };

  const syncLayerControls = (layer: LayerSummary | undefined): void => {
    const elements = [
      document.getElementById("layer-visible"),
      document.getElementById("layer-opacity"),
      document.getElementById("layer-size"),
      document.getElementById("layer-color"),
      document.getElementById("apply-layer-transform"),
      ...["pos", "rot", "scale"].flatMap((kind) => ["x", "y", "z"].map(
        (axis) => document.getElementById(`layer-${kind}-${axis}`),
      )),
    ];
    for (const element of elements) {
      if (element instanceof HTMLInputElement || element instanceof HTMLButtonElement)
        element.disabled = !layer;
    }
    if (!layer) return;
    const visible = document.getElementById("layer-visible") as HTMLInputElement | null;
    const opacity = document.getElementById("layer-opacity") as HTMLInputElement | null;
    const size = document.getElementById("layer-size") as HTMLInputElement | null;
    const color = document.getElementById("layer-color") as HTMLInputElement | null;
    if (visible) visible.checked = layer.visible;
    if (opacity) opacity.value = String(layer.opacity);
    if (size) size.value = String(layer.pointSize);
    if (color) color.value = layer.fixedColor;
    const setVector = (kind: "pos" | "rot" | "scale", values: readonly number[]) => {
      for (const [index, axis] of ["x", "y", "z"].entries()) {
        const field = document.getElementById(`layer-${kind}-${axis}`) as HTMLInputElement | null;
        if (field) field.value = String(values[index]);
      }
    };
    setVector("pos", layer.transform.position);
    setVector("rot", layer.transform.rotation);
    setVector("scale", layer.transform.scale);
  };

  const resetInspectionForCloud = (message: DecodedCloudMessage): void => {
    measurements = [];
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

  viewer.setMeasurementPickHandler((pick) => {
    if (!pick) {
      renderMeasurement(localized("measurementMiss", "No sampled point near cursor."));
      return;
    }
    if (measurements.length >= 2) measurements = [];
    measurements.push(pick);
    viewer.setMeasurement(measurements);
    renderMeasurement();
  });

  const showDecoded = (message: DecodedCloudMessage): void => {
    try {
      currentCloudName = requestNames.get(message.requestId) ?? currentCloudName;
      requestNames.delete(message.requestId);
      if (message.frameIndex !== undefined) {
        viewer.showTrajectories(trajectories, framePoses[message.frameIndex]);
      }
      const layerRequest = layerRequests.get(message.requestId);
      layerRequests.delete(message.requestId);
      const defaultMode = layerRequest?.append
        ? viewer.addLayer(message, layerRequest.sourceKey, layerRequest.name)
        : viewer.show(
          message,
          layerRequest?.sourceKey ?? `primary:${message.requestId}`,
          layerRequest?.name ?? currentCloudName,
        );
      if (!layerRequest?.append) resetInspectionForCloud(message);
      renderLayers();
      showCloudInfo(message, viewer.getGridSpacing());
      const mode = requiredInput<HTMLSelectElement>("color-mode");
      mode.value = defaultMode;
      for (const option of mode.options) {
        option.disabled =
          (option.value === "rgb" && !message.hasColor) ||
          (option.value === "intensity" && !message.hasIntensity);
      }
      const noiseToggle = document.getElementById(
        "highlight-noise",
      ) as HTMLInputElement | null;
      if (noiseToggle) noiseToggle.disabled = !message.hasNoise;
      showStatus(formatLocalized("pointsStatus", [
        message.pointCount.toLocaleString(),
        message.decodeMilliseconds.toFixed(0),
        message.indexMilliseconds.toFixed(0),
      ], "{0} points · {1} ms decode · {2} ms index"), "ready");
      vscode.postMessage({
        type: "rendered",
        requestId: message.requestId,
        pointCount: message.pointCount,
      });
      if (message.frameIndex === currentFrame &&
          frameBytes(message) <= cacheBudget / 3) {
        requestFrame(currentFrame - 1);
        requestFrame(currentFrame + 1);
      }
    } catch (error) {
      const detail = error instanceof Error ? error.message : String(error);
      showStatus(detail, "error");
      vscode.postMessage({
        type: "renderError",
        requestId: message.requestId,
        message: detail,
      });
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
        layerRequests.delete(message.requestId);
        showStatus(message.message, "error");
        vscode.postMessage({
          type: "renderError",
          requestId: message.requestId,
          message: message.message,
        });
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
      vscode.postMessage({
        type: "renderError",
        requestId: failedRequest?.requestId ?? activeRequest,
        message,
      });
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
      vscode.postMessage({
        type: "renderError",
        requestId: request.requestId,
        message: detail,
      });
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
      vscode.postMessage({ type: "renderError", requestId: message.requestId,
        message: detail });
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
        showStatus(message.message, "error");
        return;
      }
      if (message.type === "addLayer") {
        const load: LoadCloudMessage = {
          type: "load",
          requestId: message.requestId,
          name: message.name,
          bytes: message.bytes,
          sourceKey: message.sourceKey,
        };
        layerRequests.set(message.requestId, {
          sourceKey: message.sourceKey,
          name: message.name,
          append: true,
        });
        showStatus(formatLocalized(
          "loadingCloud", [message.name], "Loading {0}…",
        ), "loading");
        dispatchLoad(load);
        return;
      }
      if (message.type !== "load") return;
      if (message.frameIndex !== undefined &&
          message.generation !== sequenceGeneration) return;
      if (message.frameIndex === undefined) {
        activeRequest = message.requestId;
        layerRequests.set(message.requestId, {
          sourceKey: message.sourceKey ?? `primary:${message.requestId}`,
          name: message.name,
          append: false,
        });
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
    (event) => viewer.setColorMode(
      (event.currentTarget as HTMLSelectElement).value as ColorMode,
    ),
  );
  const colorMap = document.getElementById("color-map") as HTMLSelectElement | null;
  const equalizeIntensity = document.getElementById("equalize-intensity") as HTMLInputElement | null;
  if (colorMap) {
    colorMap.addEventListener("change", () => {
      viewer.setColorMap(colorMap.value as ColorMap);
    });
  }
  if (equalizeIntensity) {
    equalizeIntensity.addEventListener("change", () => {
      viewer.setIntensityEqualization(equalizeIntensity.checked);
    });
  }
  requiredInput<HTMLInputElement>("point-size").addEventListener(
    "input",
    (event) => {
      const input = event.currentTarget as HTMLInputElement;
      const pointSize = Number.isFinite(Number(input.value))
        ? Math.max(0, Math.min(Number(input.value), 5))
        : 1.5;
      input.value = pointSize.toFixed(2);
      viewer.setPointSize(pointSize);
      const output = document.getElementById("point-size-value");
      if (output) output.textContent = pointSize.toFixed(2);
    },
  );
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
    viewer.setMeasurement(measurements);
    renderMeasurement();
  });
  document.getElementById("apply-roi")?.addEventListener("click", () => {
    const roi = roiFromInputs();
    if (!roi || !viewer.setRoi(roi)) {
      const output = document.getElementById("roi-result");
      if (output) output.textContent = localized(
        "roiInvalid", "ROI needs finite min ≤ max on every axis.",
      );
      return;
    }
    renderRoi();
  });
  document.getElementById("reset-roi")?.addEventListener("click", () => {
    if (viewer.setRoi(undefined)) renderRoi();
  });
  document.getElementById("export-roi")?.addEventListener("click", () => {
    const count = viewer.downloadVisiblePly(currentCloudName.replace(/\.[^.]+$/u, "") + "-roi.ply");
    const output = document.getElementById("roi-result");
    if (output) output.textContent = count > 0
      ? formatLocalized("roiExported", [count.toLocaleString()], "Downloaded {0} points as PLY.")
      : localized("roiEmpty", "No finite point in ROI.");
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
    (event) => viewer.setFixedColor(
      (event.currentTarget as HTMLInputElement).value,
    ),
  );
  document.querySelector<HTMLInputElement>("#noise-color")?.addEventListener(
    "input",
    (event) => viewer.setNoiseColor(
      (event.currentTarget as HTMLInputElement).value,
    ),
  );
  document.querySelector<HTMLInputElement>("#highlight-noise")?.addEventListener(
    "change",
    (event) => viewer.setNoiseHighlight(
      (event.currentTarget as HTMLInputElement).checked,
    ),
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
  const activeLayerKey = (): string | undefined => viewer.getActiveLayerKey();
  (document.getElementById("layer-list") as HTMLSelectElement | null)?.addEventListener(
    "change",
    (event) => {
      const sourceKey = (event.currentTarget as HTMLSelectElement).value;
      if (viewer.setActiveLayer(sourceKey)) {
        renderLayers();
        renderRoi();
      }
    },
  );
  document.getElementById("layer-visible")?.addEventListener("change", (event) => {
    const sourceKey = activeLayerKey();
    if (!sourceKey) return;
    viewer.setLayerVisible(sourceKey, (event.currentTarget as HTMLInputElement).checked);
    renderLayers();
    renderRoi();
  });
  document.getElementById("layer-opacity")?.addEventListener("input", (event) => {
    const sourceKey = activeLayerKey();
    if (!sourceKey) return;
    viewer.setLayerOpacity(sourceKey, Number((event.currentTarget as HTMLInputElement).value));
    renderLayers();
  });
  document.getElementById("layer-size")?.addEventListener("input", (event) => {
    const sourceKey = activeLayerKey();
    if (!sourceKey) return;
    viewer.setLayerPointSize(sourceKey, Number((event.currentTarget as HTMLInputElement).value));
    renderLayers();
  });
  document.getElementById("layer-color")?.addEventListener("input", (event) => {
    const sourceKey = activeLayerKey();
    if (!sourceKey) return;
    viewer.setLayerFixedColor(sourceKey, (event.currentTarget as HTMLInputElement).value);
    renderLayers();
  });
  document.getElementById("apply-layer-transform")?.addEventListener("click", () => {
    const sourceKey = activeLayerKey();
    if (!sourceKey) return;
    const vector = (kind: "pos" | "rot" | "scale"): [number, number, number] =>
      ["x", "y", "z"].map((axis) => Number(
        (document.getElementById(`layer-${kind}-${axis}`) as HTMLInputElement | null)?.value,
      )) as [number, number, number];
    const transform: LayerTransform = {
      position: vector("pos"), rotation: vector("rot"), scale: vector("scale"),
    };
    if (viewer.setLayerTransform(sourceKey, transform)) {
      renderLayers();
      renderRoi();
    }
  });
  document.getElementById("remove-layer")?.addEventListener("click", () => {
    const sourceKey = activeLayerKey();
    if (!sourceKey) return;
    if (viewer.removeLayer(sourceKey)) {
      renderLayers();
      renderRoi();
    }
  });
  document.getElementById("fit-active")?.addEventListener("click", () => {
    viewer.setView("fit");
  });
  document.getElementById("fit-visible")?.addEventListener("click", () => {
    viewer.fitVisible();
  });
  document.getElementById("add-layers")?.addEventListener("click", () => {
    vscode.postMessage({ type: "addLayers", requestId: ++nextLayerRequest });
  });
  requiredInput<HTMLButtonElement>("reload").addEventListener("click", () => {
    worker?.terminate();
    worker = undefined;
    workerBusy = false;
    activeWorkerRequest = undefined;
    pendingLoads.length = 0;
    clearDecodeTimeouts();
    ++activeRequest;
    showStatus(localized("reloading", "Reloading…"), "loading");
    if (frameCount > 0) {
      ++sequenceGeneration;
      frameCache.clear();
      requestedFrames.clear();
      requestFrame(currentFrame);
    } else {
      vscode.postMessage({ type: "reload" });
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
    if (typeof bookmark.name !== "string" || bookmark.name.length === 0 ||
        bookmark.name.length > 80 || !isCameraBookmark(bookmark.camera)) continue;
    bookmarks.push({ name: bookmark.name, camera: bookmark.camera });
  }
  return { version: 1, bookmarks };
}

function isCameraBookmark(value: unknown): value is CameraBookmark {
  if (!value || typeof value !== "object") return false;
  const bookmark = value as Record<string, unknown>;
  return validVector(bookmark.position) && validVector(bookmark.target) &&
    validVector(bookmark.up) && typeof bookmark.fov === "number" &&
    Number.isFinite(bookmark.fov) && bookmark.fov > 0 && bookmark.fov < 180;
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
  return typeof value === "string" && /^sha256:[a-f0-9]{64}$/u.test(value);
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
      message.bytes.byteLength > 0 && message.bytes.byteLength <= maximumCloudBytes;
  }
  if (message.type === "hostError") {
    return validInteger(message.requestId) &&
      typeof message.message === "string" && message.message.length <= 16_384 &&
      validFrameFields(message);
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
  message: DecodedCloudMessage,
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
