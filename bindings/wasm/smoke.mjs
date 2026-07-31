import { readFile } from "node:fs/promises";
import { resolve } from "node:path";
import { Worker } from "node:worker_threads";

const [modulePath, inputPath] = process.argv.slice(2);
if (!modulePath || !inputPath) {
  throw new Error("usage: node smoke.mjs <kpt_decoder.js> <cloud>");
}

const input = await readFile(inputPath);
const bytes = input.buffer.slice(
  input.byteOffset,
  input.byteOffset + input.byteLength,
);
const worker = new Worker(
  new URL("./smoke_worker.mjs", import.meta.url),
  {
    workerData: {
      modulePath: resolve(modulePath),
      name: inputPath,
      bytes,
    },
    transferList: [bytes],
  },
);
let result;
try {
  result = await new Promise((resolveResult, reject) => {
    let received = false;
    const timeout = setTimeout(() => {
      reject(new Error("decoder worker timed out"));
    }, 30_000);
    worker.once("message", (message) => {
      received = true;
      clearTimeout(timeout);
      resolveResult(message);
    });
    worker.once("error", (error) => {
      clearTimeout(timeout);
      reject(error);
    });
    worker.once("exit", (code) => {
      if (!received) {
        clearTimeout(timeout);
        reject(new Error(`decoder worker exited before result (${code})`));
      }
    });
  });
} finally {
  await worker.terminate();
}

if (result.error) {
  throw new Error(result.error);
}
if (
  result.pointCount === 0 ||
  !result.boundsValid ||
  result.bounds.some((value) => !Number.isFinite(value))
) {
  throw new Error("decoder returned an empty or non-finite cloud");
}
if (
  result.positions.byteLength !== result.pointCount * 3 * 4 ||
  result.colors.byteLength !== (result.hasColor ? result.pointCount * 3 : 0) ||
  result.intensities.byteLength !== result.pointCount * 4 ||
  result.noises.byteLength !== (result.hasNoise ? result.pointCount : 0)
) {
  throw new Error("decoder returned inconsistent SoA buffer sizes");
}

process.stdout.write(
  `${JSON.stringify({
    pointCount: result.pointCount,
    bounds: result.bounds,
  })}\n`,
);
