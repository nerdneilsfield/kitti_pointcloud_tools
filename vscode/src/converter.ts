import * as vscode from "vscode";

const maximumInputBytes = 512 * 1024 * 1024;
const maximumWasmBytes = 64 * 1024 * 1024;
const maximumNameBytes = 1024;

let wasmBinaryPromise: Promise<ArrayBuffer> | undefined;

export async function convertPointCloud(
  extensionUri: vscode.Uri,
  sourceName: string,
  targetName: string,
  sourceBytes: ArrayBuffer,
): Promise<Uint8Array> {
  validateName(sourceName);
  validateName(targetName);
  if (sourceBytes.byteLength === 0 || sourceBytes.byteLength > maximumInputBytes)
    throw new Error("conversion input exceeds memory limit");
  const wasmBinary = await getWasmBinary(extensionUri);
  const request = {
    wasmBinary: wasmBinary.slice(0),
    sourceBytes: sourceBytes.slice(0),
    sourceName,
    targetName,
  };
  const result = isNodeRuntime()
    ? await convertInNodeWorker(extensionUri, request)
    : await convertInBrowserWorker(extensionUri, request);
  if (!(result instanceof ArrayBuffer) || result.byteLength > maximumInputBytes)
    throw new Error("conversion output exceeds memory limit");
  return new Uint8Array(result);
}

interface ConversionRequest {
  wasmBinary: ArrayBuffer;
  sourceBytes: ArrayBuffer;
  sourceName: string;
  targetName: string;
}

function isNodeRuntime(): boolean {
  return typeof process !== "undefined" &&
    typeof process.versions?.node === "string";
}

function validateName(value: string): void {
  if (value.length === 0 || new TextEncoder().encode(value).byteLength > maximumNameBytes ||
      /[\u0000-\u001f\u007f-\u009f]/u.test(value)) {
    throw new Error("conversion filename is invalid");
  }
}

async function getWasmBinary(extensionUri: vscode.Uri): Promise<ArrayBuffer> {
  if (!wasmBinaryPromise) {
    wasmBinaryPromise = Promise.resolve(vscode.workspace.fs.readFile(
      vscode.Uri.joinPath(extensionUri, "dist", "kpt_decoder.wasm"),
    )).then((bytes) => {
      if (bytes.byteLength === 0 || bytes.byteLength > maximumWasmBytes)
        throw new Error("decoder WASM exceeds memory limit");
      return bytes.buffer.slice(
        bytes.byteOffset, bytes.byteOffset + bytes.byteLength,
      ) as ArrayBuffer;
    });
  }
  return wasmBinaryPromise;
}

interface WorkerResult {
  ok: boolean;
  bytes?: ArrayBuffer;
  error?: string;
}

function validateWorkerResult(value: unknown): ArrayBuffer {
  if (!value || typeof value !== "object")
    throw new Error("converter worker returned invalid response");
  const result = value as WorkerResult;
  if (!result.ok)
    throw new Error(typeof result.error === "string"
      ? result.error : "converter worker failed");
  if (!(result.bytes instanceof ArrayBuffer) ||
      result.bytes.byteLength > maximumInputBytes)
    throw new Error("converter worker returned invalid output");
  return result.bytes;
}

async function convertInNodeWorker(
  extensionUri: vscode.Uri,
  request: ConversionRequest,
): Promise<ArrayBuffer> {
  const { Worker } = await import("node:worker_threads");
  const workerPath = vscode.Uri.joinPath(
    extensionUri, "dist", "converter.node.worker.js",
  ).fsPath;
  if (!workerPath)
    throw new Error("converter worker path is unavailable");
  return new Promise((resolve, reject) => {
    let settled = false;
    const worker = new Worker(workerPath, {
      workerData: request,
      transferList: [request.wasmBinary, request.sourceBytes],
    });
    const finish = (callback: () => void): void => {
      if (settled) return;
      settled = true;
      callback();
      void worker.terminate();
    };
    worker.once("message", (message: unknown) => {
      finish(() => {
        try {
          resolve(validateWorkerResult(message));
        } catch (error) {
          reject(error);
        }
      });
    });
    worker.once("error", (error: Error) => finish(() => reject(error)));
    worker.once("exit", (code: number) => {
      if (code !== 0) finish(() => reject(
        new Error(`converter worker exited with code ${code}`),
      ));
    });
  });
}

async function convertInBrowserWorker(
  extensionUri: vscode.Uri,
  request: ConversionRequest,
): Promise<ArrayBuffer> {
  const WorkerConstructor = globalThis.Worker;
  if (!WorkerConstructor)
    throw new Error("converter worker is unavailable");
  const worker = new WorkerConstructor(vscode.Uri.joinPath(
    extensionUri, "dist", "converter.worker.js",
  ).toString());
  return new Promise((resolve, reject) => {
    let settled = false;
    const finish = (callback: () => void): void => {
      if (settled) return;
      settled = true;
      callback();
      worker.terminate();
    };
    worker.onmessage = (event: MessageEvent<unknown>) => finish(() => {
      try {
        resolve(validateWorkerResult(event.data));
      } catch (error) {
        reject(error);
      }
    });
    worker.onerror = (event) => finish(() => reject(
      new Error(event.message || "converter worker failed"),
    ));
    worker.postMessage(request, [request.wasmBinary, request.sourceBytes]);
  });
}
