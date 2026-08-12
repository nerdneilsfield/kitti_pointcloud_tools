import { readFile } from "node:fs/promises";
import { createRequire } from "node:module";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";

const vscodeRoot = dirname(dirname(fileURLToPath(import.meta.url)));
const createDecoder = createRequire(import.meta.url)(
  join(vscodeRoot, "generated", "kpt_decoder.js"),
);
const wasmBinary = await readFile(join(vscodeRoot, "dist", "kpt_decoder.wasm"));
const decoder = await createDecoder({ wasmBinary });
const abi = decoder.ccall("kpt_decoder_abi_version", "number", [], []);
if (abi !== 5) throw new Error(`unsupported decoder ABI ${abi}`);
console.log("Verified paired decoder glue/WASM ABI 5.");
