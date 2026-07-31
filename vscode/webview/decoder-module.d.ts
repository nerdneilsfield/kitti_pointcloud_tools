declare module "*kpt_decoder.js" {
  interface DecoderModule {
    FS: {
      writeFile(path: string, bytes: Uint8Array): void;
      unlink(path: string): void;
    };
    HEAPF32: Float32Array;
    HEAPU8: Uint8Array;
    ccall(
      name: string,
      returnType: string | null,
      argumentTypes: string[],
      arguments_: unknown[],
    ): number;
    UTF8ToString(pointer: number): string;
  }

  const createDecoder: (options?: {
    locateFile?: (path: string) => string;
    wasmBinary?: Uint8Array;
  }) => Promise<DecoderModule>;
  export default createDecoder;
}
