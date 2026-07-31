import { createRequire } from "node:module";
import { parentPort, workerData } from "node:worker_threads";

const require = createRequire(import.meta.url);
const createDecoder = require(workerData.modulePath);
const module = await createDecoder();
const abiVersion = module.ccall("kpt_decoder_abi_version", "number", [], []);
if (abiVersion !== 3) {
  throw new Error(`unsupported decoder ABI ${abiVersion}`);
}
const extension = workerData.name.split(".").pop();
const virtualPath = `/input.${extension}`;
module.FS.writeFile(virtualPath, new Uint8Array(workerData.bytes));

const handle = module.ccall(
  "kpt_decode_file",
  "number",
  ["string"],
  [virtualPath],
);

try {
  const errorPointer = module.ccall(
    "kpt_decode_result_error",
    "number",
    ["number"],
    [handle],
  );
  const error = module.UTF8ToString(errorPointer);
  if (error) {
    parentPort.postMessage({ error });
  } else {
    const pointCount = module.ccall(
      "kpt_decode_result_point_count",
      "number",
      ["number"],
      [handle],
    );
    const positionsPointer = module.ccall(
      "kpt_decode_result_positions",
      "number",
      ["number"],
      [handle],
    );
    const colorsPointer = module.ccall(
      "kpt_decode_result_colors",
      "number",
      ["number"],
      [handle],
    );
    const hasColor = module.ccall(
      "kpt_decode_result_has_color",
      "number",
      ["number"],
      [handle],
    ) === 1;
    const intensitiesPointer = module.ccall(
      "kpt_decode_result_intensities",
      "number",
      ["number"],
      [handle],
    );
    const noisesPointer = module.ccall(
      "kpt_decode_result_noises",
      "number",
      ["number"],
      [handle],
    );
    const hasNoise = module.ccall(
      "kpt_decode_result_has_noise",
      "number",
      ["number"],
      [handle],
    ) === 1;
    const boundsPointer = module.ccall(
      "kpt_decode_result_bounds",
      "number",
      ["number"],
      [handle],
    );
    const boundsValid = module.ccall(
      "kpt_decode_result_bounds_valid",
      "number",
      ["number"],
      [handle],
    );

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
    const noises = module.HEAPU8.slice(
      noisesPointer,
      noisesPointer + (hasNoise ? pointCount : 0),
    );
    const bounds = Array.from(
      module.HEAPF32.subarray(boundsPointer / 4, boundsPointer / 4 + 6),
    );

    parentPort.postMessage(
      {
        pointCount,
        positions,
        colors,
        hasColor,
        intensities,
        noises,
        hasNoise,
        bounds,
        boundsValid: boundsValid === 1,
      },
      [positions.buffer, colors.buffer, intensities.buffer, noises.buffer],
    );
  }
} finally {
  module.ccall("kpt_decode_result_free", null, ["number"], [handle]);
  module.FS.unlink(virtualPath);
}
