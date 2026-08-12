import type {
  DecodedCloudMessage,
  ExtensionToWebviewMessage,
  WorkerRequest,
  WorkerResponse,
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

const maximumInputBytes = 512 * 1024 * 1024;
const maximumLabelBytes = 256 * 1024 * 1024;
const maximumWorkingSetBytes = 768 * 1024 * 1024;
const maximumWasmBytes = 64 * 1024 * 1024;
const maximumNameBytes = 1024;

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
  const decodeTimeouts = new Map<number, number>();
  const configuredTimeout = Number(document.body.dataset.decodeTimeoutMs);
  const decodeTimeoutMilliseconds =
    Number.isFinite(configuredTimeout) && configuredTimeout > 0
      ? configuredTimeout
      : 10_000;
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
  const cacheBudget = 384 * 1024 * 1024;

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
      showStatus(
        `${message.pointCount.toLocaleString()} points · ` +
          `${message.decodeMilliseconds.toFixed(0)} ms decode · ` +
          `${message.indexMilliseconds.toFixed(0)} ms index`,
        "ready",
      );
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
      if (message.type === "decodeError") {
        if (message.frameIndex !== undefined &&
            message.generation !== sequenceGeneration) return;
        if (message.frameIndex !== undefined) {
          requestedFrames.delete(message.frameIndex);
          if (message.frameIndex !== currentFrame) return;
        }
        if (frameCount === 0 && message.requestId !== activeRequest) return;
        showStatus(message.message, "error");
        vscode.postMessage({
          type: "renderError",
          requestId: message.requestId,
          message: message.message,
        });
        return;
      }
      if (message.frameIndex !== undefined) {
        if (message.generation !== sequenceGeneration) return;
        requestedFrames.delete(message.frameIndex);
        frameCache.set(message.frameIndex, message);
        evictFrameCache();
        if (message.frameIndex === currentFrame) showDecoded(message);
        return;
      }
      if (message.requestId === activeRequest) showDecoded(message);
    };
    next.onerror = (event) => {
      if (worker !== next) return;
      clearDecodeTimeouts();
      next.terminate();
      worker = undefined;
      workerNeedsWasm = true;
      requestedFrames.clear();
      ++sequenceGeneration;
      const message = event.message || "decoder worker failed";
      showStatus(message, "error");
      vscode.postMessage({
        type: "renderError",
        requestId: activeRequest,
        message,
      });
    };
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
      if (request.frameIndex === undefined) {
        activeRequest = Math.max(activeRequest, request.requestId) + 1;
      } else {
        ++sequenceGeneration;
        requestedFrames.clear();
      }
      const detail = `decoder worker timed out after ${
        decodeTimeoutMilliseconds.toLocaleString()
      } ms`;
      showStatus(detail, "error");
      vscode.postMessage({
        type: "renderError",
        requestId: request.requestId,
        message: detail,
      });
    }, decodeTimeoutMilliseconds);
    decodeTimeouts.set(request.requestId, timeout);
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
      restartWorker();
    }
    requestFrame(currentFrame);
  };

  window.addEventListener(
    "message",
    (event: MessageEvent<unknown>) => {
      if (!validMessageEvent(event) || !validExtensionMessage(event.data)) return;
      const message = event.data;
      if (message.type === "sequenceCatalog") {
        frameCount = message.frameCount;
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
        showStatus(`Loading ${message.name}…`, "loading");
      }
      const decoder = frameCount > 0
        ? (worker ?? restartWorker())
        : restartWorker();
      const request: WorkerRequest = workerNeedsWasm
        ? { ...message, wasmBinary: wasmBinary.slice(0) }
        : { ...message };
      const transfers: Transferable[] = [request.bytes];
      if (request.wasmBinary) transfers.push(request.wasmBinary);
      if (request.labelBytes) transfers.push(request.labelBytes);
      decoder.postMessage(request, transfers);
      workerNeedsWasm = false;
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
    (event) => viewer.setPointSize(
      Number((event.currentTarget as HTMLInputElement).value),
    ),
  );
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
    ++activeRequest;
    showStatus("Reloading…", "loading");
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

function validMessageEvent(event: MessageEvent<unknown>): boolean {
  if (event.source !== null && event.source !== window) return false;
  return event.origin === "" || event.origin === "null" ||
    event.origin === window.location.origin ||
    event.origin.startsWith("vscode-webview://");
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
      message.bytes.byteLength <= maximumInputBytes &&
      (labelBytes === undefined ||
       (labelBytes instanceof ArrayBuffer && labelBytes.byteLength > 0 &&
        labelBytes.byteLength <= maximumLabelBytes)) &&
      message.bytes.byteLength <= maximumWorkingSetBytes -
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
    if (trajectoryPoints > 2_000_000) return false;
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
      !validInteger(message.pointCount, 20_000_000) ||
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
  if (!(positions instanceof Float32Array) || positions.length !== count * 3 ||
      !(colors instanceof Uint8Array) ||
      colors.length !== (hasColor ? count * 3 : 0) ||
      !(intensities instanceof Float32Array) || intensities.length !== count ||
      !(noises instanceof Uint8Array) ||
      noises.length !== (hasNoise ? count : 0) ||
      !(pointOrder instanceof Uint32Array) || pointOrder.length !== count ||
      !(chunkRanges instanceof Uint32Array) || chunkRanges.length % 2 !== 0 ||
      !(lodIndices instanceof Uint32Array) ||
      lodIndices.length > count || typeof hasColor !== "boolean" ||
      typeof hasNoise !== "boolean" || typeof message.hasIntensity !== "boolean" ||
      typeof message.noiseCount !== "number" || message.noiseCount < 0 ||
      message.noiseCount > count || typeof message.defaultColorMode !== "string") {
    return false;
  }
  return ["rgb", "intensity", "height"].includes(message.defaultColorMode) &&
    Number.isFinite(message.noiseCount);
}

function frameBytes(message: DecodedCloudMessage): number {
  return message.positions.byteLength + message.colors.byteLength +
    message.intensities.byteLength + message.noises.byteLength +
    message.pointOrder.byteLength +
    message.chunkRanges.byteLength + message.lodIndices.byteLength;
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
      ? `Noise: ${message.noiseCount.toLocaleString()} / ` +
        message.pointCount.toLocaleString()
      : "Noise: unavailable";
  }
  if (!message.bounds) {
    requiredElement("aabb-min").textContent = "Min: unavailable";
    requiredElement("aabb-max").textContent = "Max: unavailable";
    requiredElement("aabb-size").textContent = "Size: unavailable";
    requiredElement("grid-spacing").textContent =
      `Grid: ${formatCoordinate(gridSpacing)} units / division`;
    info.dataset.bounds = "unavailable";
    return;
  }
  const size = message.bounds.max.map(
    (value, axis) => value - message.bounds!.min[axis],
  );
  requiredElement("aabb-min").textContent =
    `Min: ${formatVector(message.bounds.min)}`;
  requiredElement("aabb-max").textContent =
    `Max: ${formatVector(message.bounds.max)}`;
  requiredElement("aabb-size").textContent =
    `Size: ${formatVector(size)}`;
  requiredElement("grid-spacing").textContent =
    `Grid: ${formatCoordinate(gridSpacing)} units / division`;
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
