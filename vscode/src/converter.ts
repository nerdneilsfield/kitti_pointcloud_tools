import * as vscode from "vscode";
import createDecoder from "../generated/kpt_decoder.js";

type DecoderModule = Awaited<ReturnType<typeof createDecoder>>;

let decoderPromise: Promise<DecoderModule> | undefined;

export async function convertPointCloud(
  extensionUri: vscode.Uri,
  sourceName: string,
  targetName: string,
  sourceBytes: ArrayBuffer,
): Promise<Uint8Array> {
  const module = await getDecoder(extensionUri);
  let inputPointer = 0;
  let handle = 0;
  try {
    inputPointer = module.ccall(
      "kpt_alloc",
      "number",
      ["number"],
      [sourceBytes.byteLength],
    );
    if (!inputPointer) throw new Error("converter input allocation failed");
    module.HEAPU8.set(new Uint8Array(sourceBytes), inputPointer);
    handle = module.ccall(
      "kpt_convert_memory",
      "number",
      ["number", "number", "string", "string"],
      [inputPointer, sourceBytes.byteLength, sourceName, targetName],
    );
    const error = module.UTF8ToString(module.ccall(
      "kpt_convert_result_error",
      "number",
      ["number"],
      [handle],
    ));
    if (error) throw new Error(error);
    const size = module.ccall(
      "kpt_convert_result_size", "number", ["number"], [handle],
    );
    const data = module.ccall(
      "kpt_convert_result_data", "number", ["number"], [handle],
    );
    return module.HEAPU8.slice(data, data + size);
  } finally {
    if (handle) {
      module.ccall("kpt_convert_result_free", null, ["number"], [handle]);
    }
    if (inputPointer) {
      module.ccall("kpt_free", null, ["number"], [inputPointer]);
    }
  }
}

function getDecoder(extensionUri: vscode.Uri): Promise<DecoderModule> {
  if (!decoderPromise) {
    decoderPromise = Promise.resolve(vscode.workspace.fs.readFile(
      vscode.Uri.joinPath(extensionUri, "dist", "kpt_decoder.wasm"),
    )).then(async (wasmBinary) => {
      const module = await createDecoder({ wasmBinary });
      const abi = module.ccall(
        "kpt_decoder_abi_version", "number", [], [],
      );
      if (abi !== 4) throw new Error(`unsupported decoder ABI ${abi}`);
      return module;
    });
  }
  return decoderPromise;
}
