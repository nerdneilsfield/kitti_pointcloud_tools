/// <reference lib="webworker" />

import createDecoder from "../generated/kpt_decoder.js";
import { maximumCloudBytes } from "../src/protocol";

const maximumInputBytes = maximumCloudBytes;
const maximumWasmBytes = 64 * 1024 * 1024;
const maximumNameBytes = 1024;

interface ConversionData {
  wasmBinary: ArrayBuffer;
  sourceBytes: ArrayBuffer;
  sourceName: string;
  targetName: string;
}

function textBytes(value: string): number {
  return new TextEncoder().encode(value).byteLength;
}

function validName(value: unknown): value is string {
  return typeof value === "string" && value.length > 0 &&
    textBytes(value) <= maximumNameBytes &&
    !/[\u0000-\u001f\u007f-\u009f]/u.test(value);
}

function validate(data: ConversionData): void {
  if (!(data.wasmBinary instanceof ArrayBuffer) ||
      data.wasmBinary.byteLength === 0 ||
      data.wasmBinary.byteLength > maximumWasmBytes ||
      !(data.sourceBytes instanceof ArrayBuffer) ||
      data.sourceBytes.byteLength === 0 ||
      data.sourceBytes.byteLength > maximumInputBytes ||
      !validName(data.sourceName) || !validName(data.targetName)) {
    throw new Error("conversion request exceeds safety limits");
  }
}

async function convert(data: ConversionData): Promise<ArrayBuffer> {
  validate(data);
  const module = await createDecoder({
    wasmBinary: new Uint8Array(data.wasmBinary),
  });
  if (module.ccall("kpt_decoder_abi_version", "number", [], []) !== 5)
    throw new Error("unsupported decoder ABI");
  let inputPointer = 0;
  let handle = 0;
  try {
    inputPointer = module.ccall(
      "kpt_alloc", "number", ["number"], [data.sourceBytes.byteLength],
    );
    if (!inputPointer) throw new Error("converter input allocation failed");
    module.HEAPU8.set(new Uint8Array(data.sourceBytes), inputPointer);
    const sourceNameBytes = textBytes(data.sourceName);
    const targetNameBytes = textBytes(data.targetName);
    handle = module.ccall(
      "kpt_convert_memory",
      "number",
      ["number", "number", "string", "number", "string", "number"],
      [
        inputPointer, data.sourceBytes.byteLength, data.sourceName,
        sourceNameBytes, data.targetName, targetNameBytes,
      ],
    );
    const error = module.UTF8ToString(module.ccall(
      "kpt_convert_result_error", "number", ["number"], [handle],
    ));
    if (error) throw new Error(error);
    const size = module.ccall(
      "kpt_convert_result_size", "number", ["number"], [handle],
    );
    if (!Number.isSafeInteger(size) || size < 0 || size > maximumInputBytes)
      throw new Error("conversion output exceeds memory limit");
    const pointer = module.ccall(
      "kpt_convert_result_data", "number", ["number"], [handle],
    );
    if (!Number.isSafeInteger(pointer) || pointer < 0 ||
        pointer > module.HEAPU8.byteLength ||
        size > module.HEAPU8.byteLength - pointer) {
      throw new Error("converter output is outside wasm heap");
    }
    return module.HEAPU8.slice(pointer, pointer + size).buffer;
  } finally {
    if (handle)
      module.ccall("kpt_convert_result_free", null, ["number"], [handle]);
    if (inputPointer)
      module.ccall("kpt_free", null, ["number"], [inputPointer]);
  }
}

self.onmessage = (event: MessageEvent<ConversionData>) => {
  void convert(event.data).then(
    (bytes) => self.postMessage({ ok: true, bytes }, [bytes]),
    (error) => self.postMessage({
      ok: false,
      error: error instanceof Error ? error.message : String(error),
    }),
  );
};
