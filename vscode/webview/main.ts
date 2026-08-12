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
import type { ColorMode, StandardView } from "./viewer";

declare function acquireVsCodeApi(): {
  postMessage(message: unknown): void;
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
  let pendingLoad: LoadCloudMessage | undefined;
  let activeWorkerRequest: WorkerRequest | undefined;
  const decodeTimeouts = new Map<number, number>();
  const configuredTimeout = Number(document.body.dataset.decodeTimeoutMs);
  const configuredDecodeTimeout =
    Number.isFinite(configuredTimeout) && configuredTimeout > 0
      ? configuredTimeout
      : undefined;
  let activeRequest = 0;
  let nextRequest = 0;
  let frameCount = 0;
  let currentFrame = 0;
  let sequenceGeneration = 1;
  let playback: number | undefined;
  let trajectories: Array<Array<[number, number, number]>> = [];
  let framePoses: number[][] = [];
  const frameCache = new Map<number, DecodedCloudMessage>();
  const requestedFrames = new Set<number>();
  const cacheBudget = 192 * 1024 * 1024;

  const showDecoded = (message: DecodedCloudMessage): void => {
    try {
      if (message.frameIndex !== undefined) {
        viewer.showTrajectories(trajectories, framePoses[message.frameIndex]);
      }
      const defaultMode = viewer.show(message);
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
        if (frameCount === 0 && message.requestId !== activeRequest) return;
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
      if (message.requestId === activeRequest) showDecoded(message);
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
      pendingLoad = undefined;
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
        if (request.requestId !== activeRequest) {
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
      pendingLoad = undefined;
      if (request.frameIndex === undefined) {
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
    if (workerBusy) {
      if (pendingLoad?.frameIndex !== undefined) {
        requestedFrames.delete(pendingLoad.frameIndex);
      }
      pendingLoad = message;
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
    const next = pendingLoad;
    pendingLoad = undefined;
    if (!next) return;
    if (next.frameIndex !== undefined && next.generation !== sequenceGeneration) {
      requestedFrames.delete(next.frameIndex);
      return;
    }
    dispatchLoad(next);
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
      if (message.type !== "load") return;
      if (message.frameIndex !== undefined &&
          message.generation !== sequenceGeneration) return;
      if (message.frameIndex === undefined) activeRequest = message.requestId;
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
  requiredInput<HTMLButtonElement>("reload").addEventListener("click", () => {
    worker?.terminate();
    worker = undefined;
    workerBusy = false;
    activeWorkerRequest = undefined;
    pendingLoad = undefined;
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
  requiredInput<HTMLButtonElement>("play").addEventListener("click", () => {
    const button = requiredInput<HTMLButtonElement>("play");
    if (playback !== undefined) {
      window.clearInterval(playback);
      playback = undefined;
      button.textContent = "▶";
      return;
    }
    const rate = Number(requiredInput<HTMLSelectElement>("rate").value);
    playback = window.setInterval(
      () => selectFrame((currentFrame + 1) % frameCount),
      1000 / rate,
    );
    button.textContent = "⏸";
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
      message.bytes instanceof ArrayBuffer && message.bytes.byteLength > 0 &&
      message.bytes.byteLength <= maximumCloudBytes &&
      (labelBytes === undefined ||
       (labelBytes instanceof ArrayBuffer && labelBytes.byteLength > 0 &&
        labelBytes.byteLength <= maximumLabelBytes)) &&
      message.bytes.byteLength <= maximumTransportBytes -
        (labelBytes instanceof ArrayBuffer ? labelBytes.byteLength : 0) &&
      validFrameFields(message);
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
