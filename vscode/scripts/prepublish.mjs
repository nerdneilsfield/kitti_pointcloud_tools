import { spawnSync } from "node:child_process";
import { readFile, stat } from "node:fs/promises";
import { dirname, join, resolve } from "node:path";
import { fileURLToPath } from "node:url";

const vscodeRoot = resolve(dirname(fileURLToPath(import.meta.url)), "..");
const repositoryRoot = resolve(vscodeRoot, "..");
const npm = process.platform === "win32" ? "npm.cmd" : "npm";

// Packaging may run from different checkout mount points (for example local
// Docker and GitHub-hosted runners). Discard any non-relocatable CMake cache.
run("cmake", ["--preset", "wasm-decoder-release", "--fresh"], repositoryRoot);
run("cmake", ["--build", "--preset", "wasm-decoder-release"], repositoryRoot);
run(npm, ["run", "check"], vscodeRoot);
run(process.execPath, ["scripts/verify-decoder-pair.mjs"], vscodeRoot);

await requireMarkers(join(vscodeRoot, "dist", "extension.js"), [
  "show-axes",
  "show-grid",
  "highlight-noise",
  "noise-info",
]);
await requireMarkers(join(vscodeRoot, "dist", "browser.js"), [
  "kpt.openPointCloud",
  "kpt.convertPointCloud",
]);
await requireMarkers(join(vscodeRoot, "dist", "webview.js"), [
  "decoderWorkerBase64",
  "decoderWasmBase64",
  "setAxesVisible",
  "setGridVisible",
  "setNoiseHighlight",
]);
await forbidMarkers(join(vscodeRoot, "dist", "extension.js"), [
  "decoderResourcesRequest",
  "decoderResourcesError",
]);
await forbidMarkers(join(vscodeRoot, "dist", "browser.js"), [
  "decoderResourcesRequest",
  "decoderResourcesError",
]);
await forbidMarkers(join(vscodeRoot, "dist", "webview.js"), [
  "decoderResourcesRequest",
  "decoderResourcesError",
]);
await requireMarkers(join(vscodeRoot, "dist", "decoder.worker.js"), [
  "kpt_decode_result_has_noise",
  "kpt_decode_result_noises",
  "unsupported decoder ABI",
]);
await requireMarkers(join(vscodeRoot, "dist", "converter.node.worker.js"), [
  "kpt_convert_memory",
  "unsupported decoder ABI",
]);
await requireMarkers(join(vscodeRoot, "dist", "converter.worker.js"), [
  "kpt_convert_memory",
  "unsupported decoder ABI",
]);
await requireMarkers(join(vscodeRoot, "generated", "kpt_decoder.js"), [
  "kpt_decode_result_has_noise",
  "kpt_decode_result_noises",
  "kpt_convert_memory",
]);

const wasm = await stat(join(vscodeRoot, "dist", "kpt_decoder.wasm"));
if (wasm.size === 0) {
  throw new Error("dist/kpt_decoder.wasm is empty");
}

console.log("Verified current axes, grid, and noise runtime assets.");

function run(command, arguments_, cwd) {
  const result = spawnSync(command, arguments_, {
    cwd,
    stdio: "inherit",
  });
  if (result.error) throw result.error;
  if (result.status !== 0) {
    process.exit(result.status ?? 1);
  }
}

async function requireMarkers(path, markers) {
  const contents = await readFile(path, "utf8");
  const missing = markers.filter((marker) => !contents.includes(marker));
  if (missing.length > 0) {
    throw new Error(`${path} lacks required runtime markers: ${missing.join(", ")}`);
  }
}

async function forbidMarkers(path, markers) {
  const contents = await readFile(path, "utf8");
  const present = markers.filter((marker) => contents.includes(marker));
  if (present.length > 0) {
    throw new Error(`${path} contains forbidden runtime markers: ${present.join(", ")}`);
  }
}
