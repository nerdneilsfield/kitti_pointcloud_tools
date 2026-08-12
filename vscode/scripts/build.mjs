import { copyFile, mkdir, readFile } from "node:fs/promises";
import { dirname, join, resolve } from "node:path";
import { fileURLToPath } from "node:url";
import * as esbuild from "esbuild";

const vscodeRoot = resolve(dirname(fileURLToPath(import.meta.url)), "..");
const repositoryRoot = resolve(vscodeRoot, "..");
const decoderRoot = join(
  repositoryRoot,
  "build",
  "wasm-decoder-release",
  "decoder",
);
const generatedRoot = join(vscodeRoot, "generated");
const outputRoot = join(vscodeRoot, "dist");

await mkdir(generatedRoot, { recursive: true });
await mkdir(outputRoot, { recursive: true });
await copyFile(
  join(decoderRoot, "kpt_decoder.js"),
  join(generatedRoot, "kpt_decoder.js"),
);
await copyFile(
  join(decoderRoot, "kpt_decoder.wasm"),
  join(outputRoot, "kpt_decoder.wasm"),
);

await esbuild.build({
  entryPoints: [join(vscodeRoot, "webview", "decoder.worker.ts")],
  outfile: join(outputRoot, "decoder.worker.js"),
  bundle: true,
  platform: "browser",
  format: "iife",
  target: "es2022",
  external: ["node:fs", "node:crypto"],
  sourcemap: true,
});

const [decoderWorker, decoderWasm] = await Promise.all([
  readFile(join(outputRoot, "decoder.worker.js")),
  readFile(join(outputRoot, "kpt_decoder.wasm")),
]);
const decoderResourcesPlugin = {
  name: "decoder-resources",
  setup(build) {
    build.onResolve({ filter: /^kpt-decoder-resources$/ }, () => ({
      path: "kpt-decoder-resources",
      namespace: "kpt-decoder-resources",
    }));
    build.onLoad({ filter: /.*/, namespace: "kpt-decoder-resources" }, () => ({
      contents: [
        `export const decoderWorkerBase64 = ${JSON.stringify(decoderWorker.toString("base64"))};`,
        `export const decoderWasmBase64 = ${JSON.stringify(decoderWasm.toString("base64"))};`,
      ].join("\n"),
      loader: "js",
    }));
  },
};

await Promise.all([
  esbuild.build({
    entryPoints: [join(vscodeRoot, "src", "extension.ts")],
    outfile: join(outputRoot, "extension.js"),
    bundle: true,
    platform: "node",
    format: "cjs",
    target: "node20",
    external: ["vscode"],
    sourcemap: true,
  }),
  esbuild.build({
    entryPoints: [join(vscodeRoot, "src", "extension.ts")],
    outfile: join(outputRoot, "browser.js"),
    bundle: true,
    platform: "browser",
    format: "cjs",
    target: "es2022",
    external: ["vscode", "node:fs", "node:crypto", "node:worker_threads"],
    sourcemap: true,
  }),
  esbuild.build({
    entryPoints: [join(vscodeRoot, "webview", "main.ts")],
    outfile: join(outputRoot, "webview.js"),
    bundle: true,
    platform: "browser",
    format: "iife",
    target: "es2022",
    external: ["node:fs", "node:crypto"],
    plugins: [decoderResourcesPlugin],
    sourcemap: true,
  }),
  esbuild.build({
    entryPoints: [join(vscodeRoot, "src", "converter.node.worker.ts")],
    outfile: join(outputRoot, "converter.node.worker.js"),
    bundle: true,
    platform: "node",
    format: "cjs",
    target: "node20",
    sourcemap: true,
  }),
  esbuild.build({
    entryPoints: [join(vscodeRoot, "webview", "converter.worker.ts")],
    outfile: join(outputRoot, "converter.worker.js"),
    bundle: true,
    platform: "browser",
    format: "iife",
    target: "es2022",
    external: ["node:fs", "node:crypto"],
    sourcemap: true,
  }),
  esbuild.build({
    entryPoints: [join(vscodeRoot, "tests", "integration.ts")],
    outfile: join(outputRoot, "integration.js"),
    bundle: true,
    platform: "node",
    format: "cjs",
    target: "node20",
    external: ["vscode"],
    sourcemap: true,
  }),
]);
