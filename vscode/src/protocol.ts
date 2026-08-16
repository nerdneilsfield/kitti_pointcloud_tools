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
/**
 * Portable logical identities are metadata, not unbounded user payloads.
 * This is a UTF-8 byte limit, including the `path:`, `opaque:`, or `sha256:`
 * prefix; JavaScript UTF-16 code-unit counts are not a wire-size limit.
 */
export const maximumSourceKeyBytes = 16 * 1024;
/** Review-share JSON is metadata only; keep parser and host memory bounded. */
export const maximumReviewShareBytes = 4 * 1024 * 1024;
/** PNG is transferred once to the Remote-aware extension host for saving. */
export const maximumScreenshotBytes = 64 * 1024 * 1024;

/** Shared byte-bound predicate; grammar is enforced at each trust boundary. */
export function hasValidSourceKeyByteLength(value: unknown): value is string {
  if (typeof value !== "string") return false;
  const bytes = new TextEncoder().encode(value).byteLength;
  return bytes > 0 && bytes <= maximumSourceKeyBytes;
}

/**
 * CameraSnapshot stores FOV as native `float`. Validate the rounded value so
 * JSON accepted by the webview cannot round to 0 or 180 before native restore.
 */
export function hasPortableCameraFov(value: unknown): value is number {
  if (typeof value !== "number" || !Number.isFinite(value)) return false;
  const floatFov = Math.fround(value);
  // Compare float-to-float: native parses JSON into `float` then checks its
  // inclusive 0.01F lower bound.  Comparing this rounded value to the JS
  // double literal 0.01 would incorrectly reject a textual 0.01.
  return Number.isFinite(floatFov) && floatFov >= Math.fround(0.01) &&
    floatFov < 180;
}

export interface ReviewShareStyle {
  /**
   * Portable native ColorBy value. This intentionally does not match the
   * WebGL shader's compact mode values:
   * Intensity=0, RGB=1, Z=2, Label=3, None=4.
   */
  color_by: number;
  color_map: number;
  point_size: number;
  opacity: number;
  scalar_min: number;
  scalar_max: number;
  fixed_color: [number, number, number];
  noise_color: [number, number, number];
  highlight_noise: boolean;
  intensity_equalize: boolean;
}

/**
 * Portable semantic layer state. `source_path` is relative to the share JSON
 * only. It is never a host URI and is stripped before sending state back to a
 * webview.
 */
export interface ReviewShareLayer {
  source_key: string;
  source_path: string | null;
  local_to_world: number[][];
  style: ReviewShareStyle;
  visible: boolean;
}

export interface ReviewShareMeasurement {
  first_source_key: string;
  first_world: [number, number, number];
  second_source_key: string | null;
  second_world: [number, number, number] | null;
}

export interface ReviewShareBookmark {
  name: string;
  camera: {
    target: [number, number, number];
    rotation_center: [number, number, number];
    camera_to_world: number[][];
    distance: number;
    fov_y_degrees: number;
  };
}

/** Review Share v2 matches native InspectionShareFile semantic fields. */
export interface ReviewShareDocument {
  schema_version: 2;
  layers: ReviewShareLayer[];
  roi: { minimum: [number, number, number]; maximum: [number, number, number] } | null;
  measurements: ReviewShareMeasurement[];
  bookmarks: ReviewShareBookmark[];
}

/** A host-sanitized share state has no source paths or Remote URI metadata. */
export interface ReviewShareState extends Omit<ReviewShareDocument, "layers"> {
  layers: Array<Omit<ReviewShareLayer, "source_path"> & { name: string; runtime_id: string }>;
}

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

/** Remove a replayable review layer from the extension-host catalog. */
export interface RemoveLayerRequestMessage {
  type: "removeLayer";
  sourceKey: string;
}

/**
 * Reattach one unresolved Review Share layer through the extension-host
 * filesystem. The opaque transport key selects review state; it is never a
 * host path or URI supplied by the webview.
 */
export interface LocateReviewSourceRequestMessage {
  type: "locateReviewSource";
  requestId: number;
  sourceKey: string;
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
  /**
   * Host review-session generation when this payload belongs to a Review
   * Share. It gates a late Remote read after another share replaced the view.
   */
  sessionGeneration?: number;
  /** Imported share state, already stripped of host URI/source_path. */
  reviewLayer?: ReviewShareState["layers"][number];
}

/**
 * A bounded, world-space PLY payload produced by the webview. The extension
 * host chooses the destination and writes it through `workspace.fs`, so a
 * Remote SSH/container review never falls back to a client-local download.
 */
export interface ExportPlyRequestMessage {
  type: "exportPly";
  requestId: number;
  suggestedName: string;
  pointCount: number;
  bytes: ArrayBuffer;
}

/** Host acknowledgement after a Remote-aware PLY save completes. */
export interface ExportedPlyMessage {
  type: "exportedPly";
  requestId: number;
  name: string;
  pointCount: number;
}

/** A WebGL PNG produced in the webview, saved through the extension host. */
export interface SaveScreenshotRequestMessage {
  type: "saveScreenshot";
  requestId: number;
  suggestedName: string;
  bytes: ArrayBuffer;
}

export interface ScreenshotSavedMessage {
  type: "screenshotSaved";
  requestId: number;
  name: string;
}

/** Webview supplies semantic state; host resolves paths and writes JSON. */
export interface ExportReviewShareRequestMessage {
  type: "exportReviewShare";
  requestId: number;
  suggestedName: string;
  document: ReviewShareDocument;
}

/** Host chooses a Remote-aware share URI and resolves only relative paths. */
export interface ImportReviewShareRequestMessage {
  type: "importReviewShare";
  requestId: number;
}

export interface ReviewShareSavedMessage {
  type: "reviewShareSaved";
  requestId: number;
  name: string;
}

/** Host-owned import result; source paths are deliberately removed. */
export interface ReviewShareLoadedMessage {
  type: "reviewShareLoaded";
  requestId: number;
  /**
   * Host-owned monotonic session identity. A reconstructed panel may receive
   * an older replay after a newer import; the webview uses this to reject it.
   */
  sessionGeneration: number;
  document: ReviewShareState;
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
  /**
   * A decoded review layer reports its effective semantic state with the
   * render acknowledgement. The host keeps this alongside the Remote URI so
   * a Reload cannot replay a manually-added cloud with fabricated defaults.
   */
  reviewLayer?: ReviewShareState["layers"][number];
  /** Required whenever `reviewLayer` is present. */
  sessionGeneration?: number;
}

export interface RenderErrorMessage {
  type: "renderError";
  requestId: number;
  message: string;
}

/**
 * A review-layer edit made inside the webview. This contains only portable
 * semantic state; source URI/path ownership stays with the extension host.
 */
export interface ReviewLayerStateMessage {
  type: "reviewLayerState";
  sessionGeneration: number;
  layer: ReviewShareState["layers"][number];
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
  | ExportedPlyMessage
  | ScreenshotSavedMessage
  | ReviewShareSavedMessage
  | ReviewShareLoadedMessage
  | SequenceCatalogMessage;
export type WebviewToExtensionMessage =
  | ReadyMessage
  | ReloadMessage
  | AddLayersRequestMessage
  | RemoveLayerRequestMessage
  | LocateReviewSourceRequestMessage
  | ExportPlyRequestMessage
  | SaveScreenshotRequestMessage
  | ExportReviewShareRequestMessage
  | ImportReviewShareRequestMessage
  | RequestFrameMessage
  | RenderedMessage
  | RenderErrorMessage
  | ReviewLayerStateMessage;
export type WorkerRequest = DecodeCloudMessage;
export type WorkerResponse =
  | DecodeStartedMessage
  | DecodedCloudMessage
  | DecodeErrorMessage;
