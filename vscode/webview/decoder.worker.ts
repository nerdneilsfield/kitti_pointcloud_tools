/// <reference lib="webworker" />

import createDecoder from "../generated/kpt_decoder.js";
import type {
  DecodedCloudMessage,
  DecodeErrorMessage,
  WorkerRequest,
} from "../src/protocol";
import {
  maximumCloudBytes,
  maximumLabelBytes,
  maximumNameBytes,
  maximumTransportBytes,
} from "../src/protocol";

let decoderPromise:
  | Promise<Awaited<ReturnType<typeof createDecoder>>>
  | undefined;
let decoding = false;
let pendingRequest: WorkerRequest | undefined;
let activeRequestBytes = 0;

const maximumWasmBytes = 64 * 1024 * 1024;
const maximumPoints = 10_000_000;
const maximumDecodedBytes = 256 * 1024 * 1024;
const maximumBufferedRequestBytes = 256 * 1024 * 1024;
const maximumLeaves = 2048;

self.onmessage = (event: MessageEvent<unknown>) => {
  if (!validRequest(event.data)) {
    postError(0, "invalid decoder request", "invalid-input");
    return;
  }
  const request = event.data;
  if (!decoding) {
    void drain(request);
    return;
  }
  const requestBytes = transportBytes(request);
  if (activeRequestBytes + requestBytes > maximumBufferedRequestBytes) {
    postError(request.requestId, "decoder request queue exceeds memory limit",
      "memory-limit", request.frameIndex, request.generation);
    return;
  }
  // Retain only latest desired frame. This bounds detached-buffer retention and
  // prevents playback from building an unbounded decode backlog.
  if (pendingRequest) {
    postError(pendingRequest.requestId, "decoder request was superseded",
      "invalid-input", pendingRequest.frameIndex, pendingRequest.generation);
  }
  pendingRequest = request;
};

async function drain(first: WorkerRequest): Promise<void> {
  decoding = true;
  let current: WorkerRequest | undefined = first;
  try {
    while (current) {
      activeRequestBytes = transportBytes(current);
      await handleRequest(current);
      current = pendingRequest;
      pendingRequest = undefined;
    }
  } finally {
    activeRequestBytes = 0;
    decoding = false;
  }
}

function transportBytes(request: WorkerRequest): number {
  return request.bytes.byteLength + (request.labelBytes?.byteLength ?? 0) +
    (request.wasmBinary?.byteLength ?? 0);
}

function postError(
  requestId: number,
  message: string,
  code: DecodeErrorMessage["code"],
  frameIndex?: number,
  generation?: number,
): void {
  self.postMessage({
    type: "decodeError", requestId, code, message, frameIndex, generation,
  } satisfies DecodeErrorMessage);
}

function validInteger(value: unknown, maximum = 0xffffffff): value is number {
  return typeof value === "number" && Number.isSafeInteger(value) &&
    value >= 0 && value <= maximum;
}

function validText(value: unknown, maximumBytes: number): value is string {
  if (typeof value !== "string" || value.length === 0)
    return false;
  if (new TextEncoder().encode(value).byteLength > maximumBytes)
    return false;
  return !/[\u0000-\u001f\u007f-\u009f]/u.test(value);
}

function validRequest(value: unknown): value is WorkerRequest {
  if (!value || typeof value !== "object") return false;
  const request = value as Partial<WorkerRequest>;
  if (!validInteger(request.requestId) ||
      !validText(request.name, maximumNameBytes) ||
      !(request.bytes instanceof ArrayBuffer) ||
      request.bytes.byteLength === 0 ||
      request.bytes.byteLength > maximumCloudBytes) return false;
  if (request.labelBytes !== undefined &&
      (!(request.labelBytes instanceof ArrayBuffer) ||
       request.labelBytes.byteLength === 0 ||
       request.labelBytes.byteLength > maximumLabelBytes)) return false;
  if (request.bytes.byteLength > maximumTransportBytes -
      (request.labelBytes?.byteLength ?? 0)) return false;
  if (request.wasmBinary !== undefined &&
      (!(request.wasmBinary instanceof ArrayBuffer) ||
       request.wasmBinary.byteLength === 0 ||
       request.wasmBinary.byteLength > maximumWasmBytes)) return false;
  if (request.frameIndex !== undefined && !validInteger(request.frameIndex)) {
    return false;
  }
  if (request.generation !== undefined && !validInteger(request.generation)) {
    return false;
  }
  return true;
}

async function handleRequest(request: WorkerRequest): Promise<void> {
  self.postMessage({
    type: "decodeStarted",
    requestId: request.requestId,
    frameIndex: request.frameIndex,
    generation: request.generation,
  });
  try {
    const decoder = await getDecoder(request.wasmBinary);
    postDecoded(decode(request, decoder));
  } catch (error) {
    const message: DecodeErrorMessage = {
      type: "decodeError",
      requestId: request.requestId,
      code: classifyError(error),
      message: error instanceof Error ? error.message : String(error),
      frameIndex: request.frameIndex,
      generation: request.generation,
    };
    self.postMessage(message);
  }
}

function getDecoder(
  wasmBinary?: ArrayBuffer,
): Promise<Awaited<ReturnType<typeof createDecoder>>> {
  if (!decoderPromise) {
    if (!wasmBinary) {
      return Promise.reject(new Error("decoder WASM binary is missing"));
    }
    if (wasmBinary.byteLength === 0 || wasmBinary.byteLength > maximumWasmBytes) {
      return Promise.reject(new Error("decoder WASM exceeds memory limit"));
    }
    const initializing = createDecoder({
      wasmBinary: new Uint8Array(wasmBinary),
    }).then((module) => {
      const abi = module.ccall(
        "kpt_decoder_abi_version",
        "number",
        [],
        [],
      );
      if (abi !== 5) {
        throw new Error(`unsupported decoder ABI ${abi}`);
      }
      return module;
    });
    const guarded = initializing.catch((error: unknown) => {
      if (decoderPromise === guarded) decoderPromise = undefined;
      throw error;
    });
    decoderPromise = guarded;
  }
  return decoderPromise;
}

function decode(
  request: WorkerRequest,
  module: Awaited<ReturnType<typeof createDecoder>>,
): DecodedCloudMessage {
  const started = performance.now();
  const nameBytes = new TextEncoder().encode(request.name).byteLength;
  let inputPointer = 0;
  let labelPointer = 0;
  let handle = 0;
  try {
    inputPointer = module.ccall(
      "kpt_alloc", "number", ["number"], [request.bytes.byteLength],
    );
    if (!inputPointer) throw new Error("decoder input allocation failed");
    module.HEAPU8.set(new Uint8Array(request.bytes), inputPointer);
    labelPointer = request.labelBytes
      ? module.ccall(
          "kpt_alloc", "number", ["number"], [request.labelBytes.byteLength],
        )
      : 0;
    if (request.labelBytes && !labelPointer) {
      throw new Error("decoder label allocation failed");
    }
    if (request.labelBytes) {
      module.HEAPU8.set(new Uint8Array(request.labelBytes), labelPointer);
    }
    handle = module.ccall(
      request.labelBytes ? "kpt_decode_labeled_memory" : "kpt_decode_memory",
      "number",
      request.labelBytes
        ? ["number", "number", "string", "number", "number", "number"]
        : ["number", "number", "string", "number"],
      request.labelBytes
        ? [
            inputPointer, request.bytes.byteLength, request.name, nameBytes,
            labelPointer, request.labelBytes.byteLength,
          ]
        : [inputPointer, request.bytes.byteLength, request.name, nameBytes],
    );
    const errorPointer = module.ccall(
      "kpt_decode_result_error",
      "number",
      ["number"],
      [handle],
    );
    const error = module.UTF8ToString(errorPointer);
    if (error) {
      throw new Error(error);
    }
    const pointCount = callGetter(module, "kpt_decode_result_point_count", handle);
    const positionsPointer = callGetter(
      module,
      "kpt_decode_result_positions",
      handle,
    );
    const colorsPointer = callGetter(module, "kpt_decode_result_colors", handle);
    const intensitiesPointer = callGetter(
      module,
      "kpt_decode_result_intensities",
      handle,
    );
    const noisesPointer = callGetter(module, "kpt_decode_result_noises", handle);
    const boundsPointer = callGetter(module, "kpt_decode_result_bounds", handle);
    const boundsValid =
      callGetter(module, "kpt_decode_result_bounds_valid", handle) === 1;
    const hasColor =
      callGetter(module, "kpt_decode_result_has_color", handle) === 1;
    const hasIntensity =
      callGetter(module, "kpt_decode_result_has_intensity", handle) === 1;
    const hasNoise =
      callGetter(module, "kpt_decode_result_has_noise", handle) === 1;
    if (!validInteger(pointCount, maximumPoints))
      throw new Error("decoded point count exceeds limit");
    const coordinateCount = pointCount * 3;
    const positionBytes = coordinateCount * Float32Array.BYTES_PER_ELEMENT;
    const colorBytes = hasColor ? coordinateCount : 0;
    const noiseBytes = hasNoise ? pointCount : 0;
    const estimatedOutputBytes = positionBytes + colorBytes +
      pointCount * Float32Array.BYTES_PER_ELEMENT + noiseBytes +
      pointCount * Uint32Array.BYTES_PER_ELEMENT +
      maximumLeaves * 2 * Uint32Array.BYTES_PER_ELEMENT +
      (Math.ceil(pointCount / 4) + maximumLeaves) *
        Uint32Array.BYTES_PER_ELEMENT;
    if (estimatedOutputBytes > maximumDecodedBytes)
      throw new Error("decoded output exceeds memory limit");
    checkedHeapRange(module, positionsPointer, positionBytes, "positions");
    checkedHeapRange(module, colorsPointer, colorBytes, "colors");
    checkedHeapRange(
      module, intensitiesPointer,
      pointCount * Float32Array.BYTES_PER_ELEMENT, "intensities",
    );
    checkedHeapRange(module, noisesPointer, noiseBytes, "noise");
    checkedHeapRange(module, boundsPointer, 6 * Float32Array.BYTES_PER_ELEMENT,
      "bounds");
    const positions = module.HEAPF32.slice(
      positionsPointer / Float32Array.BYTES_PER_ELEMENT,
      positionsPointer / Float32Array.BYTES_PER_ELEMENT + coordinateCount,
    );
    const colors = module.HEAPU8.slice(
      colorsPointer,
      colorsPointer + (hasColor ? pointCount * 3 : 0),
    );
    const intensities = module.HEAPF32.slice(
      intensitiesPointer / Float32Array.BYTES_PER_ELEMENT,
      intensitiesPointer / Float32Array.BYTES_PER_ELEMENT + pointCount,
    );
    const noises = module.HEAPU8.slice(
      noisesPointer,
      noisesPointer + (hasNoise ? pointCount : 0),
    );
    const rawBounds = module.HEAPF32.subarray(
      boundsPointer / Float32Array.BYTES_PER_ELEMENT,
      boundsPointer / Float32Array.BYTES_PER_ELEMENT + 6,
    );
    const decodedAt = performance.now();
    const spatialIndex = buildSpatialIndex(positions);
    const actualOutputBytes = positions.byteLength + colors.byteLength +
      intensities.byteLength + noises.byteLength +
      spatialIndex.pointOrder.byteLength + spatialIndex.chunkRanges.byteLength +
      spatialIndex.lodIndices.byteLength;
    if (actualOutputBytes > maximumDecodedBytes)
      throw new Error("decoded output exceeds memory limit");
    return {
      type: "decoded",
      requestId: request.requestId,
      pointCount,
      positions,
      colors,
      intensities,
      noises,
      bounds: boundsValid
        ? {
            min: [rawBounds[0], rawBounds[1], rawBounds[2]],
            max: [rawBounds[3], rawBounds[4], rawBounds[5]],
          }
        : null,
      defaultColorMode: hasColor
        ? "rgb"
        : hasIntensity ? "intensity" : "height",
      hasColor,
      hasIntensity,
      hasNoise,
      noiseCount: hasNoise
        ? noises.reduce((count, value) => count + Number(value !== 0), 0)
        : 0,
      pointOrder: spatialIndex.pointOrder,
      chunkRanges: spatialIndex.chunkRanges,
      lodIndices: spatialIndex.lodIndices,
      decodeMilliseconds: decodedAt - started,
      indexMilliseconds: performance.now() - decodedAt,
      frameIndex: request.frameIndex,
      generation: request.generation,
    };
  } finally {
    if (handle) {
      module.ccall("kpt_decode_result_free", null, ["number"], [handle]);
    }
    if (inputPointer) {
      module.ccall("kpt_free", null, ["number"], [inputPointer]);
    }
    if (labelPointer) module.ccall("kpt_free", null, ["number"], [labelPointer]);
  }
}

function callGetter(
  module: Awaited<ReturnType<typeof createDecoder>>,
  name: string,
  handle: number,
): number {
  return module.ccall(name, "number", ["number"], [handle]);
}

function checkedHeapRange(
  module: Awaited<ReturnType<typeof createDecoder>>,
  pointer: number,
  byteLength: number,
  description: string,
): void {
  if (!Number.isSafeInteger(pointer) || pointer < 0 ||
      pointer > module.HEAPU8.byteLength ||
      byteLength > module.HEAPU8.byteLength - pointer) {
    throw new Error(`decoder ${description} pointer is outside wasm heap`);
  }
}

function postDecoded(message: DecodedCloudMessage): void {
  self.postMessage(message, {
    transfer: [
      message.positions.buffer,
      message.colors.buffer,
      message.intensities.buffer,
      message.noises.buffer,
      message.pointOrder.buffer,
      message.chunkRanges.buffer,
      message.lodIndices.buffer,
    ],
  });
}

interface SpatialIndex {
  pointOrder: Uint32Array;
  chunkRanges: Uint32Array;
  lodIndices: Uint32Array;
}

function buildSpatialIndex(
  positions: Float32Array,
  maximumLeafPoints = 100_000,
  maximumDepth = 8,
): SpatialIndex {
  const root = new Uint32Array(positions.length / 3);
  for (let index = 0; index < root.length; ++index) root[index] = index;
  const leaves: Uint32Array[] = [];
  const split = (indices: Uint32Array, depth: number): void => {
    if (indices.length <= maximumLeafPoints) {
      if (leaves.length >= maximumLeaves)
        throw new Error("spatial index exceeds memory limit");
      leaves.push(indices);
      return;
    }
    if (depth >= maximumDepth) {
      const middle = Math.ceil(indices.length / 2);
      split(indices.subarray(0, middle), depth);
      split(indices.subarray(middle), depth);
      return;
    }
    let minX = Infinity; let minY = Infinity; let minZ = Infinity;
    let maxX = -Infinity; let maxY = -Infinity; let maxZ = -Infinity;
    for (const index of indices) {
      const offset = index * 3;
      const x = positions[offset];
      const y = positions[offset + 1];
      const z = positions[offset + 2];
      if (!Number.isFinite(x + y + z)) continue;
      minX = Math.min(minX, x); minY = Math.min(minY, y);
      minZ = Math.min(minZ, z); maxX = Math.max(maxX, x);
      maxY = Math.max(maxY, y); maxZ = Math.max(maxZ, z);
    }
    const centerX = (minX + maxX) * 0.5;
    const centerY = (minY + maxY) * 0.5;
    const centerZ = (minZ + maxZ) * 0.5;
    const counts = new Uint32Array(8);
    const octantOf = (index: number): number => {
      const offset = index * 3;
      if (!Number.isFinite(
        positions[offset] + positions[offset + 1] + positions[offset + 2],
      )) return 0;
      return (positions[offset] >= centerX ? 1 : 0) |
        (positions[offset + 1] >= centerY ? 2 : 0) |
        (positions[offset + 2] >= centerZ ? 4 : 0);
    };
    for (const index of indices) ++counts[octantOf(index)];
    const populated = [...counts].filter((count) => count > 0).length;
    if (populated <= 1) {
      const middle = Math.ceil(indices.length / 2);
      split(indices.subarray(0, middle), depth + 1);
      split(indices.subarray(middle), depth + 1);
      return;
    }
    const children = Array.from(
      { length: 8 },
      (_, octant) => new Uint32Array(counts[octant]),
    );
    counts.fill(0);
    for (const index of indices) {
      const octant = octantOf(index);
      children[octant][counts[octant]++] = index;
    }
    for (const child of children) {
      if (child.length) split(child, depth + 1);
    }
  };
  split(root, 0);
  const pointOrder = new Uint32Array(root.length);
  const chunkRanges = new Uint32Array(leaves.length * 2);
  let offset = 0;
  leaves.forEach((leaf, index) => {
    pointOrder.set(leaf, offset);
    chunkRanges[index * 2] = offset;
    chunkRanges[index * 2 + 1] = leaf.length;
    offset += leaf.length;
  });
  const lodCount = leaves.reduce((total, leaf) => {
    const stride = Math.max(4, Math.ceil(leaf.length / 8192));
    return total + Math.ceil(leaf.length / stride);
  }, 0);
  const lodIndices = new Uint32Array(lodCount);
  offset = 0;
  for (const leaf of leaves) {
    const stride = Math.max(4, Math.ceil(leaf.length / 8192));
    for (let index = 0; index < leaf.length; index += stride) {
      lodIndices[offset++] = leaf[index];
    }
  }
  return { pointOrder, chunkRanges, lodIndices };
}

function classifyError(error: unknown): DecodeErrorMessage["code"] {
  const message = error instanceof Error ? error.message : String(error);
  const normalized = message.toLowerCase();
  if (normalized.includes("unsupported") ||
      normalized.includes("unknown format")) return "unsupported-format";
  if (normalized.includes("point count exceeds")) return "point-limit";
  if (normalized.includes("memory") || normalized.includes("allocation")) {
    return "memory-limit";
  }
  if (
    normalized.includes("invalid") ||
    normalized.includes("truncated") ||
    normalized.includes("parse")
  ) {
    return "invalid-input";
  }
  return "internal-error";
}
