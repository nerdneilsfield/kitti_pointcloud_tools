/// <reference lib="webworker" />

import createDecoder from "../generated/kpt_decoder.js";
import type {
  DecodedCloudMessage,
  DecodeErrorMessage,
  WorkerRequest,
} from "../src/protocol";

let decoderPromise:
  | Promise<Awaited<ReturnType<typeof createDecoder>>>
  | undefined;
let decodeQueue = Promise.resolve();

self.onmessage = (event: MessageEvent<WorkerRequest>) => {
  const request = event.data;
  decodeQueue = decodeQueue.then(
    () => handleRequest(request),
    () => handleRequest(request),
  );
};

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
    decoderPromise = createDecoder({
      wasmBinary: new Uint8Array(wasmBinary),
    }).then((module) => {
      const abi = module.ccall(
        "kpt_decoder_abi_version",
        "number",
        [],
        [],
      );
      if (abi !== 2) {
        throw new Error(`unsupported decoder ABI ${abi}`);
      }
      return module;
    });
  }
  return decoderPromise;
}

function decode(
  request: WorkerRequest,
  module: Awaited<ReturnType<typeof createDecoder>>,
): DecodedCloudMessage {
  const started = performance.now();
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
      ? ["number", "number", "string", "number", "number"]
      : ["number", "number", "string"],
    request.labelBytes
      ? [
          inputPointer, request.bytes.byteLength, request.name,
          labelPointer, request.labelBytes.byteLength,
        ]
      : [inputPointer, request.bytes.byteLength, request.name],
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
    const boundsPointer = callGetter(module, "kpt_decode_result_bounds", handle);
    const boundsValid =
      callGetter(module, "kpt_decode_result_bounds_valid", handle) === 1;
    const hasColor =
      callGetter(module, "kpt_decode_result_has_color", handle) === 1;
    const hasIntensity =
      callGetter(module, "kpt_decode_result_has_intensity", handle) === 1;
    const positions = module.HEAPF32.slice(
      positionsPointer / 4,
      positionsPointer / 4 + pointCount * 3,
    );
    const colors = module.HEAPU8.slice(
      colorsPointer,
      colorsPointer + (hasColor ? pointCount * 3 : 0),
    );
    const intensities = module.HEAPF32.slice(
      intensitiesPointer / 4,
      intensitiesPointer / 4 + pointCount,
    );
    const rawBounds = module.HEAPF32.subarray(
      boundsPointer / 4,
      boundsPointer / 4 + 6,
    );
    const decodedAt = performance.now();
    const spatialIndex = buildSpatialIndex(positions);
    return {
      type: "decoded",
      requestId: request.requestId,
      pointCount,
      positions,
      colors,
      intensities,
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

function postDecoded(message: DecodedCloudMessage): void {
  self.postMessage(message, {
    transfer: [
      message.positions.buffer,
      message.colors.buffer,
      message.intensities.buffer,
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
      leaves.push(indices);
      return;
    }
    if (depth >= maximumDepth) {
      const middle = Math.ceil(indices.length / 2);
      split(indices.slice(0, middle), depth);
      split(indices.slice(middle), depth);
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
      split(indices.slice(0, middle), depth + 1);
      split(indices.slice(middle), depth + 1);
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
  if (message.includes("unsupported")) return "unsupported-format";
  if (message.includes("point count exceeds")) return "point-limit";
  if (message.includes("memory") || message.includes("allocation")) {
    return "memory-limit";
  }
  if (
    message.includes("invalid") ||
    message.includes("truncated") ||
    message.includes("parse")
  ) {
    return "invalid-input";
  }
  return "internal-error";
}
