import type {
  ReviewShareDocument,
  ReviewShareLayer,
  ReviewShareState,
} from "./protocol";
import { maximumNameBytes, maximumReviewShareBytes } from "./protocol";

const sha256SourceKeyPattern = /^sha256:[a-f0-9]{64}$/u;
const maximumMeasurements = 10_000;
const maximumBookmarks = 100;

/** Strictly validate portable Review Share v1 before any filesystem action. */
export function validateReviewShare(
  value: unknown,
): value is ReviewShareDocument {
  if (!value || typeof value !== "object") return false;
  const document = value as Record<string, unknown>;
  if (document.schema_version !== 1 || !Array.isArray(document.layers) ||
      !Array.isArray(document.measurements) || !Array.isArray(document.bookmarks) ||
      document.measurements.length > maximumMeasurements ||
      document.bookmarks.length > maximumBookmarks ||
      !validRoi(document.roi)) return false;
  const layerKeys = new Set<string>();
  for (const layer of document.layers) {
    if (!validLayer(layer)) return false;
    const sourceKey = (layer as { source_key: string }).source_key;
    if (layerKeys.has(sourceKey)) return false;
    layerKeys.add(sourceKey);
  }
  for (const measurement of document.measurements) {
    if (!validMeasurement(measurement)) return false;
  }
  const bookmarkNames = new Set<string>();
  for (const bookmark of document.bookmarks) {
    if (!validBookmark(bookmark)) return false;
    const name = (bookmark as { name: string }).name;
    if (bookmarkNames.has(name)) return false;
    bookmarkNames.add(name);
  }
  return true;
}

/** Parse a bounded JSON file. Callers keep existing review state on failure. */
export function parseReviewShare(bytes: Uint8Array): ReviewShareDocument {
  if (bytes.byteLength === 0 || bytes.byteLength > maximumReviewShareBytes) {
    throw new Error("Review Share exceeds 4 MiB limit");
  }
  let value: unknown;
  try {
    value = JSON.parse(new TextDecoder("utf-8", { fatal: true }).decode(bytes));
  } catch {
    throw new Error("Review Share is not valid UTF-8 JSON");
  }
  if (!validateReviewShare(value)) throw new Error("Review Share schema is invalid");
  return value;
}

export function encodeReviewShare(document: ReviewShareDocument): Uint8Array {
  if (!validateReviewShare(document)) throw new Error("Review Share schema is invalid");
  const bytes = new TextEncoder().encode(`${JSON.stringify(document)}\n`);
  if (bytes.byteLength > maximumReviewShareBytes) {
    throw new Error("Review Share exceeds 4 MiB limit");
  }
  return bytes;
}

/** Remove source paths before a document crosses into a webview. */
export function reviewShareState(
  document: ReviewShareDocument,
  runtimeIdFor: (layer: ReviewShareLayer, index: number) => string,
  displayNameFor: (layer: ReviewShareLayer, index: number) => string,
): ReviewShareState {
  return {
    schema_version: 1,
    layers: document.layers.map((layer, index) => ({
      source_key: layer.source_key,
      local_to_world: layer.local_to_world.map((row) => [...row]),
      style: {
        ...layer.style,
        fixed_color: [...layer.style.fixed_color] as [number, number, number],
        noise_color: [...layer.style.noise_color] as [number, number, number],
      },
      visible: layer.visible,
      runtime_id: runtimeIdFor(layer, index),
      name: displayNameFor(layer, index),
    })),
    roi: document.roi && {
      minimum: [...document.roi.minimum] as [number, number, number],
      maximum: [...document.roi.maximum] as [number, number, number],
    },
    measurements: document.measurements.map((measurement) => ({
      first_source_key: measurement.first_source_key,
      first_world: [...measurement.first_world] as [number, number, number],
      second_source_key: measurement.second_source_key,
      second_world: measurement.second_world &&
        [...measurement.second_world] as [number, number, number] | null,
    })),
    bookmarks: document.bookmarks.map((bookmark) => ({
      name: bookmark.name,
      camera: {
        target: [...bookmark.camera.target] as [number, number, number],
        rotation_center: [...bookmark.camera.rotation_center] as [number, number, number],
        camera_to_world: bookmark.camera.camera_to_world.map((row) => [...row]),
        distance: bookmark.camera.distance,
        fov_y_degrees: bookmark.camera.fov_y_degrees,
      },
    })),
  };
}

export function validRelativeSharePath(value: unknown): value is string {
  if (typeof value !== "string" || value.length === 0 ||
      new TextEncoder().encode(value).byteLength > 16 * 1024 ||
      /[\\\u0000-\u001f\u007f-\u009f]/u.test(value) || value.startsWith("/")) {
    return false;
  }
  const normalized = normalizeRelativeSharePath(value);
  return normalized !== undefined && normalized === value;
}

function normalizeRelativeSharePath(value: string): string | undefined {
  const output: string[] = [];
  for (const part of value.split("/")) {
    if (part.length === 0 || part === ".") return undefined;
    // Review source paths may never escape the share JSON's directory.
    if (part === "..") return undefined;
    output.push(part);
  }
  return output.length > 0 ? output.join("/") : undefined;
}

function validLayer(value: unknown): value is ReviewShareLayer {
  if (!value || typeof value !== "object") return false;
  const layer = value as Record<string, unknown>;
  return validSourceKey(layer.source_key) &&
    (layer.source_path === null || validRelativeSharePath(layer.source_path)) &&
    validMatrix(layer.local_to_world, 4, [0, 0, 0, 1]) &&
    validStyle(layer.style) && typeof layer.visible === "boolean";
}

function validStyle(value: unknown): boolean {
  if (!value || typeof value !== "object") return false;
  const style = value as Record<string, unknown>;
  return validInteger(style.color_by, 0, 3) && validInteger(style.color_map, 0, 9) &&
    finiteIn(style.point_size, 0, 5) && finiteIn(style.opacity, 0, 1) &&
    Number.isFinite(style.scalar_min) && Number.isFinite(style.scalar_max) &&
    validVector(style.fixed_color) && validVector(style.noise_color) &&
    (style.fixed_color as number[]).every((item) => finiteIn(item, 0, 1)) &&
    (style.noise_color as number[]).every((item) => finiteIn(item, 0, 1)) &&
    typeof style.highlight_noise === "boolean" &&
    typeof style.intensity_equalize === "boolean";
}

function validRoi(value: unknown): boolean {
  if (value === null) return true;
  if (!value || typeof value !== "object") return false;
  const roi = value as Record<string, unknown>;
  return validVector(roi.minimum) && validVector(roi.maximum) &&
    (roi.minimum as number[]).every((item, index) => item <= (roi.maximum as number[])[index]);
}

function validMeasurement(value: unknown): boolean {
  if (!value || typeof value !== "object") return false;
  const measurement = value as Record<string, unknown>;
  const hasSecondKey = measurement.second_source_key !== null;
  const hasSecondWorld = measurement.second_world !== null;
  return validSourceKey(measurement.first_source_key) &&
    validVector(measurement.first_world) && hasSecondKey === hasSecondWorld &&
    (!hasSecondKey || (validSourceKey(measurement.second_source_key) &&
      validVector(measurement.second_world)));
}

function validBookmark(value: unknown): boolean {
  if (!value || typeof value !== "object") return false;
  const bookmark = value as Record<string, unknown>;
  if (!validText(bookmark.name, 80) || !bookmark.camera ||
      typeof bookmark.camera !== "object") return false;
  const camera = bookmark.camera as Record<string, unknown>;
  return validVector(camera.target) && validVector(camera.rotation_center) &&
    validMatrix(camera.camera_to_world, 3) && finiteIn(camera.distance, 1e-9) &&
    finiteIn(camera.fov_y_degrees, 1e-6, 179.999999);
}

function validSourceKey(value: unknown): value is string {
  // Native v1 captures `path:` keys. They are accepted only while this host
  // parses JSON, then rewritten to an opaque URI hash before webview state is
  // emitted. Thus a Remote filesystem path never crosses the boundary.
  if (typeof value !== "string") return false;
  if (sha256SourceKeyPattern.test(value)) return true;
  if (value.startsWith("opaque:")) {
    return validPrintableUnicodeScalars(value.slice("opaque:".length));
  }
  return validNativePathKey(value);
}

function validNativePathKey(value: string): boolean {
  if (!value.startsWith("path:") || value.length > 16_384 ||
      value.includes("\\") ||
      !validPrintableUnicodeScalars(value.slice("path:".length))) return false;
  const path = value.slice("path:".length);
  if (!/^(?:\/[\s\S]*|[A-Za-z]:\/[\s\S]*)$/u.test(path) ||
      path.includes("//")) return false;
  return !path.split("/").some((part) => part === "." || part === "..");
}

/**
 * JSON may contain an escaped lone UTF-16 surrogate even though its bytes
 * decode successfully. Treat source-key payloads as portable UTF-8 text:
 * every code point must be a scalar and C0/C1 controls are forbidden.
 * Other scalars, including non-ASCII names and '/', remain opaque data.
 */
function validPrintableUnicodeScalars(value: string): boolean {
  if (value.length === 0) return false;
  for (let index = 0; index < value.length; ++index) {
    const codePoint = value.codePointAt(index);
    if (codePoint === undefined ||
        (codePoint >= 0 && codePoint <= 0x1f) ||
        (codePoint >= 0x7f && codePoint <= 0x9f) ||
        (codePoint >= 0xd800 && codePoint <= 0xdfff)) return false;
    if (codePoint > 0xffff) ++index;
  }
  return true;
}

function validText(value: unknown, maximum = maximumNameBytes): value is string {
  return typeof value === "string" && value.length > 0 &&
    new TextEncoder().encode(value).byteLength <= maximum &&
    !/[\u0000-\u001f\u007f-\u009f]/u.test(value);
}

function validVector(value: unknown): value is [number, number, number] {
  return Array.isArray(value) && value.length === 3 && value.every(Number.isFinite);
}

function validMatrix(
  value: unknown,
  size: number,
  expectedLastRow?: readonly number[],
): boolean {
  if (!Array.isArray(value) || value.length !== size ||
      value.some((row) => !Array.isArray(row) || row.length !== size ||
        row.some((entry) => !Number.isFinite(entry)))) return false;
  return !expectedLastRow || expectedLastRow.every((entry, index) =>
    (value[size - 1] as number[])[index] === entry);
}

function finiteIn(value: unknown, minimum: number, maximum = Number.POSITIVE_INFINITY): value is number {
  return typeof value === "number" && Number.isFinite(value) &&
    value >= minimum && value <= maximum;
}

function validInteger(value: unknown, minimum: number, maximum: number): value is number {
  return typeof value === "number" && Number.isSafeInteger(value) &&
    value >= minimum && value <= maximum;
}
