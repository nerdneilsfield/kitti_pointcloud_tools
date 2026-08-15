export interface CloudBounds {
  min: [number, number, number];
  max: [number, number, number];
}

// Keep transport limits shared by both sides of the extension-host/webview
// boundary. A message accepted by the host must never be silently rejected by
// the webview because the two bundles disagree about its size.
export const maximumCloudBytes = 128 * 1024 * 1024;
export const maximumLabelBytes = 64 * 1024 * 1024;
export const maximumTransportBytes = 192 * 1024 * 1024;
export const maximumNameBytes = 1024;

export interface LoadCloudMessage {
  type: "load";
  requestId: number;
  name: string;
  bytes: ArrayBuffer;
  /**
   * Opaque stable identity of the host-side source URI. It is optional while
   * single-cloud webviews remain compatible with older extension hosts.
   * Source paths and URI strings must never be sent to the webview.
   */
  sourceKey?: string;
  labelBytes?: ArrayBuffer;
  frameIndex?: number;
  generation?: number;
}

/** A webview request for the extension host to select remote point-cloud URIs. */
export interface AddLayersRequestMessage {
  type: "addLayers";
  requestId: number;
}

/**
 * One additive layer payload. The extension host reads the URI through
 * vscode.workspace.fs, then sends only its opaque source key and display name.
 */
export interface AddLayerMessage {
  type: "addLayer";
  requestId: number;
  sourceKey: string;
  name: string;
  bytes: ArrayBuffer;
}

export interface SequenceCatalogMessage {
  type: "sequenceCatalog";
  frameCount: number;
  name: string;
  trajectories: Array<Array<[number, number, number]>>;
  framePoses: number[][];
}

export interface RequestFrameMessage {
  type: "requestFrame";
  requestId: number;
  frameIndex: number;
  generation: number;
}

export interface ReadyMessage {
  type: "ready";
}

export interface ReloadMessage {
  type: "reload";
}

export interface RenderedMessage {
  type: "rendered";
  requestId: number;
  pointCount: number;
}

export interface RenderErrorMessage {
  type: "renderError";
  requestId: number;
  message: string;
}

export interface HostErrorMessage {
  type: "hostError";
  requestId: number;
  message: string;
  /** True only for a document-primary read failure before `load` is posted. */
  primary?: boolean;
  frameIndex?: number;
  generation?: number;
}

export interface DecodeCloudMessage extends LoadCloudMessage {
  wasmBinary?: ArrayBuffer;
}

export interface DecodeStartedMessage {
  type: "decodeStarted";
  requestId: number;
  frameIndex?: number;
  generation?: number;
}

export interface DecodedCloudMessage {
  type: "decoded";
  requestId: number;
  pointCount: number;
  positions: Float32Array;
  colors: Uint8Array;
  intensities: Float32Array;
  noises: Uint8Array;
  bounds: CloudBounds | null;
  defaultColorMode: "rgb" | "intensity" | "height";
  hasColor: boolean;
  hasIntensity: boolean;
  hasNoise: boolean;
  noiseCount: number;
  pointOrder: Uint32Array;
  chunkRanges: Uint32Array;
  lodIndices: Uint32Array;
  indexMilliseconds: number;
  decodeMilliseconds: number;
  frameIndex?: number;
  generation?: number;
}

export interface DecodeErrorMessage {
  type: "decodeError";
  requestId: number;
  code:
    | "unsupported-format"
    | "invalid-input"
    | "point-limit"
    | "memory-limit"
    | "internal-error";
  message: string;
  frameIndex?: number;
  generation?: number;
}

export type ExtensionToWebviewMessage =
  | LoadCloudMessage
  | AddLayerMessage
  | HostErrorMessage
  | SequenceCatalogMessage;
export type WebviewToExtensionMessage =
  | ReadyMessage
  | ReloadMessage
  | AddLayersRequestMessage
  | RequestFrameMessage
  | RenderedMessage
  | RenderErrorMessage;
export type WorkerRequest = DecodeCloudMessage;
export type WorkerResponse =
  | DecodeStartedMessage
  | DecodedCloudMessage
  | DecodeErrorMessage;
