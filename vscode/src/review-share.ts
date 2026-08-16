import type {
  ReviewShareDocument,
  ReviewShareLayer,
  ReviewShareState,
} from "./protocol";
import {
  hasPortableCameraFov,
  hasValidSourceKeyByteLength,
  maximumNameBytes,
  maximumReviewShareBytes,
} from "./protocol";

const sha256SourceKeyPattern = /^sha256:[a-f0-9]{64}$/u;
const maximumMeasurements = 10_000;
const maximumBookmarks = 100;
// Keep this wire validation aligned with ViewportModel::setCameraSnapshot.
const cameraOrthonormalTolerance = 1e-4;
// Bounds the double-precision CameraSnapshot fields before they overflow the
// native viewport's float renderer normalization/orbit probes.
const maximumRenderableCameraMagnitude = 1e30;

/** Strictly validate portable Review Share v2 before any filesystem action. */
export function validateReviewShare(
  value: unknown,
): value is ReviewShareDocument {
  if (!value || typeof value !== "object") return false;
  const document = value as Record<string, unknown>;
  if (document.schema_version !== 2 || !Array.isArray(document.layers) ||
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
    schema_version: 2,
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

/**
 * Validate one host-sanitized layer state returned by a webview. Unlike an
 * on-disk Review Share, it must not carry a source path, and a manually-added
 * layer may use its opaque source key as the stable runtime ID. Imported
 * layers retain their host-issued `review-<session>-<index>` IDs.
 */
export function validateReviewShareStateLayer(
  value: unknown,
): value is ReviewShareState["layers"][number] {
  if (!value || typeof value !== "object") return false;
  const layer = value as Record<string, unknown>;
  const document = {
    schema_version: 2,
    layers: [{
      source_key: layer.source_key,
      source_path: null,
      local_to_world: layer.local_to_world,
      style: layer.style,
      visible: layer.visible,
    }],
    roi: null,
    measurements: [],
    bookmarks: [],
  };
  if (!validateReviewShare(document) ||
      !sha256SourceKeyPattern.test(String(layer.source_key)) ||
      "source_path" in layer || !validText(layer.name, maximumNameBytes) ||
      /[\\/]/u.test(layer.name) ||
      typeof layer.runtime_id !== "string") return false;
  return /^review-[0-9]+-[0-9]+$/u.test(layer.runtime_id) ||
    layer.runtime_id === layer.source_key;
}

/**
 * Validate a full host-sanitized Review Share snapshot returned by a
 * webview.  It is intentionally stricter than an on-disk document: source
 * paths and native logical keys are never allowed to cross this boundary.
 */
export function validateReviewShareState(
  value: unknown,
): value is ReviewShareState {
  if (!value || typeof value !== "object") return false;
  const state = value as Record<string, unknown>;
  if (!Array.isArray(state.layers) || !Array.isArray(state.measurements) ||
      !Array.isArray(state.bookmarks)) return false;
  const portable = {
    schema_version: state.schema_version,
    layers: state.layers.map((layer) => {
      if (!layer || typeof layer !== "object") return layer;
      const candidate = layer as Record<string, unknown>;
      return {
        source_key: candidate.source_key,
        source_path: null,
        local_to_world: candidate.local_to_world,
        style: candidate.style,
        visible: candidate.visible,
      };
    }),
    roi: state.roi,
    measurements: state.measurements,
    bookmarks: state.bookmarks,
  };
  if (!validateReviewShare(portable) ||
      !state.layers.every(validateReviewShareStateLayer)) return false;
  const runtimeIds = new Set<string>();
  for (const layer of state.layers as ReviewShareState["layers"]) {
    if (!sha256SourceKeyPattern.test(layer.source_key) ||
        "source_path" in (layer as object) || runtimeIds.has(layer.runtime_id)) {
      return false;
    }
    runtimeIds.add(layer.runtime_id);
  }
  return state.measurements.every((measurement) => {
    if (!measurement || typeof measurement !== "object") return false;
    const candidate = measurement as Record<string, unknown>;
    return sha256SourceKeyPattern.test(String(candidate.first_source_key)) &&
      (candidate.second_source_key === null ||
        sha256SourceKeyPattern.test(String(candidate.second_source_key)));
  });
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
  return validInteger(style.color_by, 0, 4) && validInteger(style.color_map, 0, 9) &&
    finiteExclusiveLower(style.point_size, 0, 5) && finiteIn(style.opacity, 0, 1) &&
    Number.isFinite(style.scalar_min) && Number.isFinite(style.scalar_max) &&
    (style.scalar_min as number) <= (style.scalar_max as number) &&
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
  return validRenderableCameraVector(camera.target) &&
    validRenderableCameraVector(camera.rotation_center) &&
    validCameraToWorld(camera.camera_to_world) &&
    finiteIn(camera.distance, 1e-9, maximumRenderableCameraMagnitude) &&
    hasPortableCameraFov(camera.fov_y_degrees);
}

/**
 * A bookmark camera basis is a proper world rotation, never a general affine.
 * Native rejects a skew, zero basis, and reflected basis before restoring its
 * CameraSnapshot; reject them at this protocol boundary too.
 */
function validCameraToWorld(value: unknown): boolean {
  if (!validMatrix(value, 3)) return false;
  // Native parses this field as Matrix3f. Work in equivalent f32 values and
  // apply Eigen's relative Frobenius `isApprox(identity, 1e-4)` contract,
  // rather than incorrectly treating every matrix element independently.
  const matrix = (value as number[][]).map((row) => row.map(Math.fround));
  if (matrix.some((row) => row.some((entry) => !Number.isFinite(entry)))) return false;
  let differenceNormSquared = 0;
  let productNormSquared = 0;
  for (let left = 0; left < 3; ++left) {
    for (let right = 0; right < 3; ++right) {
      let dot = 0;
      for (let row = 0; row < 3; ++row) {
        dot = Math.fround(dot + Math.fround(
          matrix[row][left] * matrix[row][right],
        ));
      }
      const difference = Math.fround(dot - (left === right ? 1 : 0));
      differenceNormSquared = Math.fround(
        differenceNormSquared + Math.fround(difference * difference),
      );
      productNormSquared = Math.fround(
        productNormSquared + Math.fround(dot * dot),
      );
    }
  }
  const toleranceSquared = Math.fround(
    cameraOrthonormalTolerance * cameraOrthonormalTolerance,
  );
  if (!Number.isFinite(differenceNormSquared) || !Number.isFinite(productNormSquared) ||
      differenceNormSquared > toleranceSquared * Math.min(productNormSquared, 3)) {
    return false;
  }
  const determinant =
    matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1]) -
    matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0]) +
    matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]);
  return Number.isFinite(determinant) && determinant > 0;
}

function validRenderableCameraVector(value: unknown): value is [number, number, number] {
  return validVector(value) && (value as number[]).every((entry) =>
    Math.abs(entry) <= maximumRenderableCameraMagnitude);
}

function validSourceKey(value: unknown): value is string {
  // Native v1 captures `path:` keys. They are accepted only while this host
  // parses JSON, then rewritten to an opaque URI hash before webview state is
  // emitted. Thus a Remote filesystem path never crosses the boundary.
  if (!hasValidSourceKeyByteLength(value)) return false;
  if (sha256SourceKeyPattern.test(value)) return true;
  if (value.startsWith("opaque:")) {
    return validPrintableUnicodeScalars(value.slice("opaque:".length));
  }
  return validNativePathKey(value);
}

function validNativePathKey(value: string): boolean {
  if (!value.startsWith("path:") ||
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
  let hasScalar = false;
  for (const scalar of value) {
    const codePoint = scalar.codePointAt(0);
    if (codePoint === undefined ||
        (codePoint >= 0 && codePoint <= 0x1f) ||
        (codePoint >= 0x7f && codePoint <= 0x9f) ||
        (codePoint >= 0xd800 && codePoint <= 0xdfff)) return false;
    hasScalar = true;
  }
  return hasScalar;
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

function finiteExclusiveLower(
  value: unknown,
  minimum: number,
  maximum = Number.POSITIVE_INFINITY,
): value is number {
  return typeof value === "number" && Number.isFinite(value) &&
    value > minimum && value <= maximum;
}

function validInteger(value: unknown, minimum: number, maximum: number): value is number {
  return typeof value === "number" && Number.isSafeInteger(value) &&
    value >= minimum && value <= maximum;
}
