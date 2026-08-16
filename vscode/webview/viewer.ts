import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import type {
  CloudBounds,
  DecodedCloudMessage,
  ReviewShareBookmark,
  ReviewShareLayer,
} from "../src/protocol";

export type ColorMode = "rgb" | "intensity" | "height" | "fixed";
export type ColorMap =
  "turbo" | "viridis" | "plasma" | "inferno" | "magma" |
  "grayscale" | "hot" | "jet" | "spring" | "autumn";
export type StandardView = "fit" | "top" | "front" | "left" | "right" | "iso";

/** A closed, world-space axis-aligned crop box. */
export interface RoiBox {
  min: [number, number, number];
  max: [number, number, number];
}

/** A world-space point picked from a renderer layer. */
export interface PointPick {
  index: number;
  point: [number, number, number];
  sourceKey: string;
  layerName: string;
}

/** CPU picking policy exposed for an explicit Inspector degradation notice. */
export interface PickingScope {
  degraded: boolean;
  enabled: boolean;
  activeLayerName?: string;
  visibleLayerCount: number;
  candidateLimitPerLayer: number;
}

export interface LayerTransform {
  position: [number, number, number];
  /** Euler degrees, applied XYZ in the layer's local coordinate system. */
  rotation: [number, number, number];
  scale: [number, number, number];
}

export interface LayerSummary {
  sourceKey: string;
  name: string;
  visible: boolean;
  opacity: number;
  pointSize: number;
  fixedColor: string;
  noiseColor: string;
  colorMode: ColorMode;
  colorMap: ColorMap;
  intensityEqualize: boolean;
  highlightNoise: boolean;
  transform: LayerTransform;
  pointCount: number;
  finitePointCount: number;
  hasColor: boolean;
  hasIntensity: boolean;
  hasNoise: boolean;
  noiseCount: number;
  bounds: CloudBounds | null;
  renderQuality: "full" | "lod";
}

interface LayerStyle {
  colorMode: ColorMode;
  colorMap: ColorMap;
  intensityEqualize: boolean;
  pointSize: number;
  fixedColor: THREE.Color;
  noiseColor: THREE.Color;
  highlightNoise: boolean;
  opacity: number;
  visible: boolean;
}

interface PointLayer {
  sourceKey: string;
  name: string;
  group: THREE.Group;
  cloud: THREE.Points<THREE.BufferGeometry, THREE.ShaderMaterial>;
  lodCloud?: THREE.Points<THREE.BufferGeometry, THREE.ShaderMaterial>;
  /** Stable dynamic index avoids leaking replaced full-ROI GL index buffers. */
  fullIndex?: THREE.BufferAttribute;
  message: DecodedCloudMessage;
  equalizedIntensities: Float32Array;
  roiIndices?: Uint32Array;
  /** Backing-buffer bytes retained by the ROI subarray, not only its length. */
  roiBytes: number;
  roiCentroid?: THREE.Vector3;
  roiFinitePointCount?: number;
  center: THREE.Vector3;
  boundsCenter: THREE.Vector3;
  finitePointCount: number;
  /** LOD-only layers keep full CPU data for ROI/export but not full GPU data. */
  renderQuality: "full" | "lod";
  renderIndices?: Uint32Array;
  gpuBytes: number;
  cpuBytes: number;
  radius: number;
  style: LayerStyle;
}

interface PointStatistics {
  centroid: THREE.Vector3;
  finitePointCount: number;
}

interface RoiFilterResult extends PointStatistics {
  indices: Uint32Array;
  storageBytes: number;
}

/** Serializable camera state used by webview-local bookmarks. */
export interface CameraBookmark {
  position: [number, number, number];
  target: [number, number, number];
  up: [number, number, number];
  fov: number;
}

const colorModeValue: Record<ColorMode, number> = {
  rgb: 0,
  intensity: 1,
  height: 2,
  fixed: 3,
};
const maximumFloat32 = 3.4028234663852886e38;
const maximumFramingSamples = 100_000;
// Four layers stay below a 100k projected-point budget. This is below the
// documented 100k per-layer ceiling and keeps the click path responsive.
const maximumPickingCandidates = 25_000;
const minimumGpuLodPoints = 4_096;
const mebibyte = 1024 * 1024;
// ASCII PLY is built in a webview Blob before one bounded host message. Keep
// worst-case string and Blob memory bounded rather than freezing a Remote
// review with an unbounded crop.
const maximumWebviewExportPoints = 1_000_000;
const framingPercentile = 0.95;
const framingFill = 0.85;
const minimumAutoFov = 35;
const maximumAutoFov = 75;
const colorMapValue: Record<ColorMap, number> = {
  turbo: 0, viridis: 1, plasma: 2, inferno: 3, magma: 4,
  grayscale: 5, hot: 6, jet: 7, spring: 8, autumn: 9,
};

export class PointCloudViewer {
  private readonly scene = new THREE.Scene();
  private readonly camera = new THREE.PerspectiveCamera(50, 1, 0.01, 10000);
  private readonly renderer: THREE.WebGLRenderer;
  private readonly controls: OrbitControls;
  private readonly observer: ResizeObserver;
  private readonly themeObserver: MutationObserver;
  private readonly axes = new THREE.Group();
  private readonly axisArrows = [
    new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0)),
    new THREE.ArrowHelper(new THREE.Vector3(0, 1, 0)),
    new THREE.ArrowHelper(new THREE.Vector3(0, 0, 1)),
  ];
  private readonly grids: THREE.GridHelper[] = [];
  private readonly measurementGroup = new THREE.Group();
  private readonly layers = new Map<string, PointLayer>();
  private gpuBytesInUse = 0;
  private cpuBytesInUse = 0;
  private roiBytesInUse = 0;
  private readonly gpuByteBudget = rendererGpuBudget();
  private readonly cpuByteBudget = rendererCpuBudget();
  /**
   * Buffer allocations surface on a later WebGL render. Keep every layer
   * whose geometry changed since the previous successful frame so recovery
   * demotes an upload candidate, never an unrelated active layer.
   */
  private readonly pendingGpuLayerKeys = new Set<string>();
  private activeLayerKey?: string;
  // Active-layer aliases keep the existing single-cloud controls compatible.
  private cloud?: THREE.Points<THREE.BufferGeometry, THREE.ShaderMaterial>;
  private lodCloud?: THREE.Points<THREE.BufferGeometry, THREE.ShaderMaterial>;
  private roiHelper?: THREE.Box3Helper;
  private readonly trajectories: THREE.Line[] = [];
  private readonly trajectoryGroup = new THREE.Group();
  private trajectoryPaths?: Array<Array<[number, number, number]>>;
  private message?: DecodedCloudMessage;
  private equalizedIntensities?: Float32Array;
  private roi?: RoiBox;
  private roiIndices?: Uint32Array;
  private roiGeneration = 0;
  private roiFilterTimer?: number;
  private roiFiltering = false;
  private lastRoiPreviewAt = 0;
  private roiChangeHandler?: () => void;
  private rendererWarningHandler?: (message: string) => void;
  private framingPositions?: Float32Array;
  private center = new THREE.Vector3();
  private referenceMinimum = new THREE.Vector3(-5, -5, -5);
  private referenceMaximum = new THREE.Vector3(5, 5, 5);
  private radius = 1;
  private frame = 0;
  private colorMode: ColorMode = "height";
  private colorMap: ColorMap = "turbo";
  private intensityEqualize = true;
  private pointSize = 1.5;
  private fixedColor = new THREE.Color("#ffffff");
  private noiseColor = new THREE.Color("#ff0000");
  private highlightNoise = true;
  private gridSpacing = 1;
  private axesVisible = false;
  private gridVisible = false;
  private customBackground = false;
  private rolling = false;
  private rollPointer = -1;
  private previousRollX = 0;
  private measurementEnabled = false;
  private measurementPointer?: {
    id: number;
    x: number;
    y: number;
    moved: boolean;
  };
  private measurementPickHandler?: (pick: PointPick | undefined) => void;

  constructor(private readonly container: HTMLElement) {
    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
    this.container.append(this.renderer.domElement);
    this.scene.background = new THREE.Color(readThemeBackground());
    this.axes.add(...this.axisArrows);
    this.updateReferenceColors();
    this.axes.visible = false;
    this.scene.add(this.axes);
    this.scene.add(this.measurementGroup);
    this.trajectoryGroup.matrixAutoUpdate = false;
    this.scene.add(this.trajectoryGroup);
    this.camera.up.set(0, 0, 1);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.08;
    this.controls.mouseButtons.LEFT = THREE.MOUSE.ROTATE;
    this.controls.mouseButtons.MIDDLE = THREE.MOUSE.DOLLY;
    this.controls.mouseButtons.RIGHT = THREE.MOUSE.PAN;
    this.render = this.render.bind(this);
    this.controls.addEventListener("change", this.invalidate);
    this.renderer.domElement.addEventListener(
      "pointerdown",
      this.beginRoll,
      true,
    );
    this.renderer.domElement.addEventListener(
      "pointermove",
      this.continueRoll,
      true,
    );
    this.renderer.domElement.addEventListener("pointerup", this.endRoll, true);
    this.renderer.domElement.addEventListener(
      "pointercancel",
      this.endRoll,
      true,
    );
    this.renderer.domElement.addEventListener(
      "pointerdown",
      this.beginMeasurement,
      true,
    );
    this.renderer.domElement.addEventListener(
      "pointermove",
      this.continueMeasurement,
      true,
    );
    this.renderer.domElement.addEventListener(
      "pointerup",
      this.endMeasurement,
      true,
    );
    this.renderer.domElement.addEventListener(
      "pointercancel",
      this.cancelMeasurement,
      true,
    );
    this.observer = new ResizeObserver(() => this.resize());
    this.observer.observe(this.container);
    this.themeObserver = new MutationObserver(() => {
      this.syncThemeBackground();
    });
    this.themeObserver.observe(document.documentElement, {
      attributes: true,
      attributeFilter: ["class", "style"],
    });
    this.resize();
  }

  show(
    message: DecodedCloudMessage,
    sourceKey = "primary",
    name = "point-cloud",
    append = false,
    fitAfterAppend = true,
  ): ColorMode {
    const firstCloud = !append || this.layers.size === 0;
    const replacedLayer = append ? this.layers.get(sourceKey) : undefined;
    const previousActiveKey = this.activeLayerKey;
    const previousLayerOrder = replacedLayer ? [...this.layers.keys()] : undefined;
    const replacementStyle = replacedLayer
      ? copyLayerStyle(replacedLayer.style)
      : undefined;
    const replacementTransform = replacedLayer
      ? layerTransform(replacedLayer.group)
      : undefined;
    const cpuBytes = estimatedCpuBytes(message);
    const cpuAvailable = !append
      ? this.cpuByteBudget
      : this.cpuByteBudget - this.cpuBytesInUse + (replacedLayer?.cpuBytes ?? 0);
    const projectedCpuBytes = append
      ? this.cpuBytesInUse - (replacedLayer?.cpuBytes ?? 0) + cpuBytes
      : cpuBytes;
    if (cpuBytes > cpuAvailable) {
      throw new Error("CPU review budget is exhausted; remove a layer before adding another");
    }
    if (append && this.roi && projectedCpuBytes +
        this.projectedRoiReserveBytes(sourceKey, message, true) >
        this.cpuByteBudget) {
      throw new Error("ROI review budget is exhausted; remove a layer before adding another");
    }
    const equalizedIntensities = equalizeIntensities(message.intensities);
    this.equalizedIntensities = equalizedIntensities;
    // Build/validate the new geometry before replacing a duplicate source or
    // clearing the primary scene. A rejected GPU allocation leaves review
    // state intact instead of deleting a usable old layer.
    const gpuAvailable = !append
      ? this.gpuByteBudget
      : this.gpuByteBudget - this.gpuBytesInUse + (replacedLayer?.gpuBytes ?? 0);
    const fullGpuBytes = estimatedFullGpuBytes(message);
    const lodSource = message.lodIndices.length > 0
      ? message.lodIndices
      : message.pointOrder;
    let geometry: THREE.BufferGeometry;
    let renderQuality: "full" | "lod";
    let lodOnlyIndices: Uint32Array | undefined;
    let primaryLodIndices: Uint32Array | undefined;
    let gpuBytes: number;
    if (fullGpuBytes <= gpuAvailable) {
      geometry = fullGeometry(message, equalizedIntensities);
      renderQuality = "full";
      gpuBytes = fullGpuBytes;
      const lodGpuBytes = estimatedPackedGpuBytes(message, lodSource.length);
      if (message.pointCount > maximumFramingSamples &&
          gpuBytes + lodGpuBytes <= gpuAvailable) {
        primaryLodIndices = lodSource;
        gpuBytes += lodGpuBytes;
      }
    } else {
      const maximumLodPoints = Math.floor(
        gpuAvailable / packedGpuBytesPerPoint(message),
      );
      if (maximumLodPoints < minimumGpuLodPoints) {
        throw new Error("GPU review budget is exhausted; remove a layer before adding another");
      }
      const lodIndices = uniformlySampleIndices(
        lodSource,
        Math.max(minimumGpuLodPoints, maximumLodPoints),
      );
      geometry = gatherGeometry(message, lodIndices, equalizedIntensities);
      renderQuality = "lod";
      lodOnlyIndices = lodIndices;
      gpuBytes = estimatedPackedGpuBytes(message, lodIndices.length);
    }

    const intensityRange = finiteRange(message.intensities);
    const heightRange = message.bounds
      ? [message.bounds.min[2], message.bounds.max[2]]
      : [0, 1];
    if (firstCloud ||
        (this.colorMode === "rgb" && !message.hasColor) ||
        (this.colorMode === "intensity" && !message.hasIntensity)) {
      this.colorMode = message.defaultColorMode;
    }
    const material = new THREE.ShaderMaterial({
      uniforms: {
        colorMode: { value: colorModeValue[this.colorMode] },
        colorMap: { value: colorMapValue[this.colorMap] },
        intensityEqualize: { value: this.intensityEqualize },
        pointSize: { value: this.pointSize },
        intensityRange: { value: new THREE.Vector2(...intensityRange) },
        heightRange: { value: new THREE.Vector2(...heightRange) },
        fixedColor: { value: this.fixedColor },
        noiseColor: { value: this.noiseColor },
        highlightNoise: { value: this.highlightNoise && message.hasNoise },
        opacity: { value: 1 },
      },
      vertexShader: `
        attribute float intensity;
        attribute float equalizedIntensity;
        attribute float noise;
        uniform int colorMode;
        uniform int colorMap;
        uniform bool intensityEqualize;
        uniform float pointSize;
        uniform vec2 intensityRange;
        uniform vec2 heightRange;
        uniform vec3 fixedColor;
        uniform vec3 noiseColor;
        uniform bool highlightNoise;
        varying vec3 pointColor;

        vec3 ramp(vec3 a, vec3 b, vec3 c, vec3 d, float value) {
          float t = clamp(value, 0.0, 1.0);
          if (t < 0.3333) return mix(a, b, t * 3.0);
          if (t < 0.6667) return mix(b, c, (t - 0.3333) * 3.0);
          return mix(c, d, (t - 0.6667) * 3.0);
        }

        vec3 gradient(float value) {
          float t = clamp(value, 0.0, 1.0);
          if (colorMap == 1) return ramp(vec3(0.267,0.005,0.329), vec3(0.191,0.407,0.556), vec3(0.208,0.719,0.473), vec3(0.993,0.906,0.144), t);
          if (colorMap == 2) return ramp(vec3(0.050,0.030,0.528), vec3(0.494,0.012,0.658), vec3(0.973,0.586,0.252), vec3(0.940,0.975,0.131), t);
          if (colorMap == 3) return ramp(vec3(0.001,0.000,0.014), vec3(0.341,0.062,0.430), vec3(0.865,0.283,0.173), vec3(0.988,0.998,0.645), t);
          if (colorMap == 4) return ramp(vec3(0.001,0.000,0.014), vec3(0.317,0.071,0.485), vec3(0.716,0.215,0.475), vec3(0.987,0.991,0.750), t);
          if (colorMap == 5) return vec3(t);
          if (colorMap == 6) return ramp(vec3(0.0), vec3(1.0,0.0,0.0), vec3(1.0,1.0,0.0), vec3(1.0), t);
          if (colorMap == 7) return clamp(vec3(1.5-abs(4.0*t-3.0), 1.5-abs(4.0*t-2.0), 1.5-abs(4.0*t-1.0)), 0.0, 1.0);
          if (colorMap == 8) return vec3(1.0, t, 1.0 - t);
          if (colorMap == 9) return vec3(1.0, t, 0.0);
          return ramp(vec3(0.190,0.072,0.232), vec3(0.251,0.252,0.633), vec3(0.276,0.788,0.690), vec3(0.987,0.906,0.144), t);
        }

        void main() {
          if (colorMode == 0) {
            #ifdef USE_COLOR
              pointColor = color;
            #else
              pointColor = vec3(0.0);
            #endif
          } else if (colorMode == 1) {
            float span = max(intensityRange.y - intensityRange.x, 1e-6);
            float value = intensityEqualize ? equalizedIntensity :
              (intensity - intensityRange.x) / span;
            pointColor = gradient(value);
          } else if (colorMode == 2) {
            float span = max(heightRange.y - heightRange.x, 1e-6);
            pointColor = gradient((position.z - heightRange.x) / span);
          } else {
            pointColor = fixedColor;
          }
          if (highlightNoise && noise > 0.5) pointColor = noiseColor;
          gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
          gl_PointSize = pointSize;
        }
      `,
      fragmentShader: `
        varying vec3 pointColor;
        uniform float opacity;
        void main() {
          vec2 centered = gl_PointCoord - vec2(0.5);
          if (dot(centered, centered) > 0.25) discard;
          gl_FragColor = vec4(pointColor, opacity);
        }
      `,
      vertexColors: message.hasColor,
    });
    if (!append) {
      this.clearCloud();
      this.clearRoi();
      this.clearMeasurement();
    } else if (replacedLayer) {
      // Keep ROI, measurements, active source identity, and camera intact
      // while atomically replacing a refreshed source with the same key.
      this.disposeLayer(replacedLayer);
      this.gpuBytesInUse = Math.max(0, this.gpuBytesInUse - replacedLayer.gpuBytes);
      this.cpuBytesInUse = Math.max(0, this.cpuBytesInUse - replacedLayer.cpuBytes);
      this.roiBytesInUse = Math.max(0, this.roiBytesInUse - replacedLayer.roiBytes);
      this.layers.delete(sourceKey);
    }
    this.message = message;
    this.lodCloud = undefined;
    this.roiIndices = undefined;
    this.cloud = new THREE.Points(geometry, material);
    const group = new THREE.Group();
    group.name = name;
    if (replacementTransform) applyLayerTransform(group, replacementTransform);
    group.add(this.cloud);
    if (primaryLodIndices) {
      const lodGeometry = gatherGeometry(message, primaryLodIndices, equalizedIntensities);
      this.lodCloud = new THREE.Points(lodGeometry, material.clone());
      this.lodCloud.visible = false;
      group.add(this.lodCloud);
    }
    this.scene.add(group);
    this.updatePhysicalPointSize();
    // Exact finite-point statistics are cached once per layer. Framing may
    // sample for FOV, but its target must remain the real arithmetic centroid.
    const statistics = this.updateBounds(message);
    const layer: PointLayer = {
      sourceKey,
      name,
      group,
      cloud: this.cloud,
      lodCloud: this.lodCloud,
      fullIndex: renderQuality === "full" ? this.cloud.geometry.getIndex() ?? undefined : undefined,
      message,
      equalizedIntensities,
      center: statistics.centroid,
      boundsCenter: message.bounds
        ? new THREE.Vector3(...message.bounds.min).add(
          new THREE.Vector3(...message.bounds.max),
        ).multiplyScalar(0.5)
        : statistics.centroid.clone(),
      finitePointCount: statistics.finitePointCount,
      renderQuality,
      renderIndices: lodOnlyIndices,
      gpuBytes,
      cpuBytes,
      roiBytes: 0,
      radius: this.radius,
      style: normalizeLayerStyle(replacementStyle ?? this.currentStyle(), message),
    };
    if (previousLayerOrder) {
      this.insertReplacementInLayerOrder(sourceKey, layer, previousLayerOrder);
    } else {
      this.layers.set(sourceKey, layer);
    }
    this.gpuBytesInUse += gpuBytes;
    this.cpuBytesInUse += cpuBytes;
    this.pendingGpuLayerKeys.add(sourceKey);
    this.applyLayerStyle(layer);
    const preservedActive = replacedLayer && previousActiveKey !== undefined &&
      previousActiveKey !== sourceKey
      ? this.layers.get(previousActiveKey)
      : undefined;
    this.adoptLayer(preservedActive ?? layer);
    if (this.roi) this.scheduleRoiFilter(this.roi);
    if (append && fitAfterAppend) this.fitVisible();
    else this.setView(firstCloud ? "iso" : "fit");
    this.invalidate();
    return this.colorMode;
  }

  /** Add a host-decoded cloud without exposing its URI to the webview. */
  addLayer(
    message: DecodedCloudMessage,
    sourceKey: string,
    name: string,
  ): ColorMode {
    return this.show(message, sourceKey, name, true);
  }

  /** Replace an existing source without resetting the user camera. */
  replaceLayer(
    message: DecodedCloudMessage,
    sourceKey: string,
    name: string,
  ): ColorMode {
    return this.show(message, sourceKey, name, true, false);
  }

  getLayers(): LayerSummary[] {
    return [...this.layers.values()].map((layer) => ({
      sourceKey: layer.sourceKey,
      name: layer.name,
      visible: layer.style.visible,
      opacity: layer.style.opacity,
      pointSize: layer.style.pointSize,
      fixedColor: `#${layer.style.fixedColor.getHexString()}`,
      noiseColor: `#${layer.style.noiseColor.getHexString()}`,
      colorMode: layer.style.colorMode,
      colorMap: layer.style.colorMap,
      intensityEqualize: layer.style.intensityEqualize,
      highlightNoise: layer.style.highlightNoise,
      transform: layerTransform(layer.group),
      pointCount: layer.message.pointCount,
      finitePointCount: layer.finitePointCount,
      hasColor: layer.message.hasColor,
      hasIntensity: layer.message.hasIntensity,
      hasNoise: layer.message.hasNoise,
      noiseCount: layer.message.noiseCount,
      bounds: layer.message.bounds,
      renderQuality: layer.renderQuality,
    }));
  }

  getActiveLayerKey(): string | undefined {
    return this.activeLayerKey;
  }

  hasLayer(sourceKey: string): boolean {
    return this.layers.has(sourceKey);
  }

  getPickingScope(): PickingScope {
    const visible = this.visibleLayers();
    if (visible.length <= 4) {
      return {
        degraded: false,
        enabled: visible.length > 0,
        visibleLayerCount: visible.length,
        candidateLimitPerLayer: maximumPickingCandidates,
      };
    }
    const active = this.activeLayer();
    return {
      degraded: true,
      enabled: !!active?.style.visible,
      activeLayerName: active?.name,
      visibleLayerCount: visible.length,
      candidateLimitPerLayer: maximumPickingCandidates,
    };
  }

  setActiveLayer(sourceKey: string): boolean {
    const layer = this.layers.get(sourceKey);
    if (!layer) return false;
    this.adoptLayer(layer);
    this.updateReferenceForLayers([layer]);
    this.invalidate();
    return true;
  }

  setLayerVisible(sourceKey: string, visible: boolean): boolean {
    const layer = this.layers.get(sourceKey);
    if (!layer) return false;
    layer.style.visible = visible;
    this.invalidate();
    return true;
  }

  setLayerOpacity(sourceKey: string, opacity: number): boolean {
    const layer = this.layers.get(sourceKey);
    if (!layer || !Number.isFinite(opacity)) return false;
    layer.style.opacity = THREE.MathUtils.clamp(opacity, 0, 1);
    this.applyLayerStyle(layer);
    this.invalidate();
    return true;
  }

  setLayerPointSize(sourceKey: string, size: number): boolean {
    const layer = this.layers.get(sourceKey);
    if (!layer || !Number.isFinite(size)) return false;
    layer.style.pointSize = THREE.MathUtils.clamp(size, 0, 5);
    if (sourceKey === this.activeLayerKey) this.pointSize = layer.style.pointSize;
    this.applyLayerStyle(layer);
    this.invalidate();
    return true;
  }

  setLayerFixedColor(
    sourceKey: string,
    color: THREE.ColorRepresentation,
  ): boolean {
    const layer = this.layers.get(sourceKey);
    if (!layer) return false;
    layer.style.fixedColor.set(color);
    if (sourceKey === this.activeLayerKey) this.fixedColor.copy(layer.style.fixedColor);
    this.applyLayerStyle(layer);
    this.invalidate();
    return true;
  }

  setLayerTransform(sourceKey: string, transform: LayerTransform): boolean {
    const layer = this.layers.get(sourceKey);
    if (!layer || !isRenderableLayerTransform(transform, layer.message.bounds)) {
      return false;
    }
    layer.group.position.fromArray(transform.position);
    layer.group.rotation.set(
      THREE.MathUtils.degToRad(transform.rotation[0]),
      THREE.MathUtils.degToRad(transform.rotation[1]),
      THREE.MathUtils.degToRad(transform.rotation[2]),
    );
    layer.group.scale.fromArray(transform.scale);
    layer.group.updateMatrixWorld(true);
    if (this.roi) this.scheduleRoiFilter(this.roi);
    this.updateReferenceForLayers(this.visibleLayers());
    this.invalidate();
    return true;
  }

  removeLayer(sourceKey: string): boolean {
    const layer = this.layers.get(sourceKey);
    if (!layer) return false;
    this.disposeLayer(layer);
    this.gpuBytesInUse = Math.max(0, this.gpuBytesInUse - layer.gpuBytes);
    this.cpuBytesInUse = Math.max(0, this.cpuBytesInUse - layer.cpuBytes);
    this.roiBytesInUse = Math.max(0, this.roiBytesInUse - layer.roiBytes);
    this.layers.delete(sourceKey);
    this.pendingGpuLayerKeys.delete(sourceKey);
    if (this.activeLayerKey === sourceKey) {
      const next = this.layers.values().next().value as PointLayer | undefined;
      if (next) this.adoptLayer(next);
      else this.clearActiveLayer();
    }
    // Measurements store immutable world coordinates; deleting a source leaves
    // them visible/detached instead of silently discarding review evidence.
    if (this.layers.size === 0) this.clearRoi();
    else if (this.roi) this.scheduleRoiFilter(this.roi);
    this.updateReferenceForLayers(this.visibleLayers());
    this.invalidate();
    return true;
  }

  fitVisible(): void {
    const layers = this.visibleLayers();
    const direction = this.camera.position.clone().sub(this.controls.target);
    this.fitCamera(direction.lengthSq() > 0 ? direction.normalize() :
      new THREE.Vector3(1, -1, 1).normalize(), layers);
    this.invalidate();
  }

  private currentStyle(): LayerStyle {
    return {
      colorMode: this.colorMode,
      colorMap: this.colorMap,
      intensityEqualize: this.intensityEqualize,
      pointSize: this.pointSize,
      fixedColor: this.fixedColor.clone(),
      noiseColor: this.noiseColor.clone(),
      highlightNoise: this.highlightNoise,
      opacity: 1,
      visible: true,
    };
  }

  private activeLayer(): PointLayer | undefined {
    return this.activeLayerKey ? this.layers.get(this.activeLayerKey) : undefined;
  }

  private insertReplacementInLayerOrder(
    sourceKey: string,
    replacement: PointLayer,
    previousOrder: readonly string[],
  ): void {
    const retained = new Map(this.layers);
    this.layers.clear();
    for (const key of previousOrder) {
      if (key === sourceKey) {
        this.layers.set(key, replacement);
        continue;
      }
      const layer = retained.get(key);
      if (layer) this.layers.set(key, layer);
    }
    // Defensive: transformations may change the map only synchronously, but
    // retain any key not represented in a stale captured order.
    for (const [key, layer] of retained) {
      if (!this.layers.has(key)) this.layers.set(key, layer);
    }
  }

  private adoptLayer(layer: PointLayer): void {
    this.activeLayerKey = layer.sourceKey;
    this.cloud = layer.cloud;
    this.lodCloud = layer.lodCloud;
    this.message = layer.message;
    this.equalizedIntensities = layer.equalizedIntensities;
    this.roiIndices = layer.roiIndices;
    this.framingPositions = layer.message.positions;
    this.center.copy(layer.center).applyMatrix4(layer.group.matrixWorld);
    this.radius = layer.radius * Math.max(
      Math.abs(layer.group.scale.x), Math.abs(layer.group.scale.y),
      Math.abs(layer.group.scale.z), 1e-6,
    );
    this.colorMode = layer.style.colorMode;
    this.colorMap = layer.style.colorMap;
    this.intensityEqualize = layer.style.intensityEqualize;
    this.pointSize = layer.style.pointSize;
    this.fixedColor.copy(layer.style.fixedColor);
    this.noiseColor.copy(layer.style.noiseColor);
    this.highlightNoise = layer.style.highlightNoise;
    this.applyLayerStyle(layer);
  }

  private clearActiveLayer(): void {
    this.activeLayerKey = undefined;
    this.cloud = undefined;
    this.lodCloud = undefined;
    this.message = undefined;
    this.equalizedIntensities = undefined;
    this.roiIndices = undefined;
    this.framingPositions = undefined;
    this.center.set(0, 0, 0);
    this.radius = 1;
  }

  private applyLayerStyle(layer: PointLayer): void {
    for (const cloud of [layer.cloud, layer.lodCloud]) {
      if (!cloud) continue;
      const uniforms = cloud.material.uniforms;
      uniforms.colorMode.value = colorModeValue[layer.style.colorMode];
      uniforms.colorMap.value = colorMapValue[layer.style.colorMap];
      uniforms.intensityEqualize.value = layer.style.intensityEqualize;
      uniforms.pointSize.value = Math.min(
        layer.style.pointSize * this.renderer.getPixelRatio(), 5,
      );
      uniforms.fixedColor.value = layer.style.fixedColor;
      uniforms.noiseColor.value = layer.style.noiseColor;
      uniforms.highlightNoise.value =
        layer.style.highlightNoise && layer.message.hasNoise;
      uniforms.opacity.value = layer.style.opacity;
      cloud.material.transparent = layer.style.opacity < 0.999;
      cloud.material.depthWrite = layer.style.opacity >= 0.999;
      cloud.material.needsUpdate = true;
    }
  }

  private visibleLayers(): PointLayer[] {
    return [...this.layers.values()].filter((layer) => layer.style.visible);
  }

  setColorMode(mode: ColorMode): void {
    this.colorMode = mode;
    const layer = this.activeLayer();
    if (layer) layer.style.colorMode = mode;
    if (this.cloud) {
      this.cloud.material.uniforms.colorMode.value = colorModeValue[mode];
    }
    if (this.lodCloud) {
      this.lodCloud.material.uniforms.colorMode.value = colorModeValue[mode];
    }
    this.invalidate();
  }

  setColorMap(colorMap: ColorMap): void {
    this.colorMap = colorMap;
    const layer = this.activeLayer();
    if (layer) layer.style.colorMap = colorMap;
    this.updateCloudUniform("colorMap", colorMapValue[colorMap]);
    this.invalidate();
  }

  setIntensityEqualization(equalize: boolean): void {
    this.intensityEqualize = equalize;
    const layer = this.activeLayer();
    if (layer) layer.style.intensityEqualize = equalize;
    this.updateCloudUniform("intensityEqualize", equalize);
    this.invalidate();
  }

  setPointSize(size: number): void {
    if (!Number.isFinite(size)) return;
    this.pointSize = Math.max(0, Math.min(size, 5));
    const layer = this.activeLayer();
    if (layer) layer.style.pointSize = this.pointSize;
    this.updatePhysicalPointSize();
    this.invalidate();
  }

  setFixedColor(color: THREE.ColorRepresentation): void {
    this.fixedColor = new THREE.Color(color);
    const layer = this.activeLayer();
    if (layer) layer.style.fixedColor.copy(this.fixedColor);
    this.updateCloudUniform("fixedColor", this.fixedColor);
    this.invalidate();
  }

  setNoiseColor(color: THREE.ColorRepresentation): void {
    this.noiseColor = new THREE.Color(color);
    const layer = this.activeLayer();
    if (layer) layer.style.noiseColor.copy(this.noiseColor);
    this.updateCloudUniform("noiseColor", this.noiseColor);
    this.invalidate();
  }

  setNoiseHighlight(visible: boolean): void {
    this.highlightNoise = visible;
    const layer = this.activeLayer();
    if (layer) layer.style.highlightNoise = visible;
    this.updateCloudUniform("highlightNoise", visible);
    this.invalidate();
  }

  setAxesVisible(visible: boolean): void {
    this.axesVisible = visible;
    this.axes.visible = visible;
    this.container.dataset.axesVisible = String(visible);
    this.invalidate();
  }

  setGridVisible(visible: boolean): void {
    this.gridVisible = visible;
    for (const grid of this.grids) grid.visible = visible;
    this.container.dataset.gridVisible = String(visible);
    this.invalidate();
  }

  getGridSpacing(): number {
    return this.gridSpacing;
  }

  setBackground(color: THREE.ColorRepresentation): void {
    this.customBackground = true;
    this.scene.background = new THREE.Color(color);
    this.updateReferenceHelpers(this.referenceMinimum, this.referenceMaximum);
    this.invalidate();
  }

  useThemeBackground(): string {
    this.customBackground = false;
    return this.syncThemeBackground() ?? "#1e1e1e";
  }

  syncThemeBackground(): string | null {
    if (this.customBackground) return null;
    const color = new THREE.Color(readThemeBackground());
    this.scene.background = color;
    this.updateReferenceHelpers(this.referenceMinimum, this.referenceMaximum);
    this.invalidate();
    return `#${color.getHexString()}`;
  }

  setView(view: StandardView): void {
    const directions: Record<Exclude<StandardView, "fit">, THREE.Vector3> = {
      top: new THREE.Vector3(0, 0, 1),
      front: new THREE.Vector3(0, -1, 0),
      left: new THREE.Vector3(-1, 0, 0),
      right: new THREE.Vector3(1, 0, 0),
      iso: new THREE.Vector3(1, -1, 1).normalize(),
    };
    const direction = view === "fit"
      ? this.camera.position.clone().sub(this.controls.target).normalize()
      : directions[view];
    const active = this.activeLayer();
    this.fitCamera(
      direction.lengthSq() > 0 ? direction : directions.iso,
      active ? [active] : this.visibleLayers(),
    );
    this.invalidate();
  }

  /** Enable click-only CPU picking. While enabled, left clicks do not orbit. */
  setMeasurementEnabled(enabled: boolean): void {
    this.measurementEnabled = enabled;
    this.measurementPointer = undefined;
    this.container.dataset.measurementEnabled = String(enabled);
  }

  setMeasurementPickHandler(
    handler: ((pick: PointPick | undefined) => void) | undefined,
  ): void {
    this.measurementPickHandler = handler;
  }

  setMeasurement(points: readonly PointPick[]): void {
    this.clearMeasurement();
    const valid = points.filter((point) =>
      point.point.length === 3 && point.point.every(Number.isFinite),
    );
    if (valid.length === 0) return;

    const positions = new Float32Array(valid.length * 3);
    valid.forEach((point, index) => positions.set(point.point, index * 3));
    const markerGeometry = new THREE.BufferGeometry();
    markerGeometry.setAttribute(
      "position",
      new THREE.BufferAttribute(positions, 3),
    );
    const markers = new THREE.Points(
      markerGeometry,
      new THREE.PointsMaterial({
        color: 0xffca28,
        size: 12,
        sizeAttenuation: false,
        depthTest: false,
      }),
    );
    markers.renderOrder = 2;
    this.measurementGroup.add(markers);
    if (valid.length < 2) {
      this.invalidate();
      return;
    }
    const lineGeometry = new THREE.BufferGeometry();
    lineGeometry.setAttribute(
      "position",
      new THREE.BufferAttribute(positions.slice(0, 6), 3),
    );
    const line = new THREE.Line(
      lineGeometry,
      new THREE.LineBasicMaterial({
        color: 0xffca28,
        depthTest: false,
      }),
    );
    line.renderOrder = 1;
    this.measurementGroup.add(line);
    this.invalidate();
  }

  clearMeasurement(): void {
    for (const child of [...this.measurementGroup.children]) {
      this.measurementGroup.remove(child);
      const rendered = child as THREE.Line | THREE.Points;
      rendered.geometry?.dispose();
      if (rendered.material) disposeMaterial(rendered.material);
    }
    this.invalidate();
  }

  /** Apply or clear a closed world-space AABB crop. */
  setRoi(roi: RoiBox | undefined): boolean {
    if (this.layers.size === 0) return false;
    if (roi && !isValidRoi(roi)) return false;
    if (roi && !this.canReserveRoiIndices()) {
      this.rendererWarningHandler?.(
        "ROI review budget is exhausted; remove a layer before filtering",
      );
      return false;
    }
    this.clearRoi();
    if (!roi) {
      this.invalidate();
      return true;
    }

    this.roi = copyRoi(roi);
    this.roiHelper = new THREE.Box3Helper(
      new THREE.Box3(
        new THREE.Vector3(...roi.min),
        new THREE.Vector3(...roi.max),
      ),
      0xffa000,
    );
    this.roiHelper.renderOrder = 3;
    for (const material of materialList(this.roiHelper.material)) {
      material.depthTest = false;
    }
    this.scene.add(this.roiHelper);
    this.scheduleRoiFilter(this.roi);
    this.invalidate();
    return true;
  }

  getRoi(): RoiBox | undefined {
    return this.roi ? copyRoi(this.roi) : undefined;
  }

  isRoiFiltering(): boolean {
    return this.roiFiltering;
  }

  setRoiChangeHandler(handler: (() => void) | undefined): void {
    this.roiChangeHandler = handler;
  }

  setRendererWarningHandler(handler: ((message: string) => void) | undefined): void {
    this.rendererWarningHandler = handler;
  }

  getVisiblePointCount(): number {
    return this.visibleLayers().reduce((total, layer) =>
      total + (layer.roiIndices?.length ??
        layer.finitePointCount), 0);
  }

  getCameraBookmark(): CameraBookmark {
    return {
      position: this.camera.position.toArray() as [number, number, number],
      target: this.controls.target.toArray() as [number, number, number],
      up: this.camera.up.toArray() as [number, number, number],
      fov: this.camera.fov,
    };
  }

  restoreCameraBookmark(bookmark: CameraBookmark): boolean {
    if (!isValidBookmark(bookmark)) return false;
    const position = new THREE.Vector3(...bookmark.position);
    const target = new THREE.Vector3(...bookmark.target);
    const up = new THREE.Vector3(...bookmark.up).normalize();
    if (position.distanceToSquared(target) <= 1e-12 || up.lengthSq() <= 1e-12)
      return false;
    this.camera.position.copy(position);
    this.camera.up.copy(up);
    this.camera.fov = bookmark.fov;
    this.controls.target.copy(target);
    this.camera.lookAt(target);
    this.camera.updateProjectionMatrix();
    this.camera.updateMatrixWorld();
    this.controls.update();
    this.invalidate();
    return true;
  }

  /** Build a bounded world-space PLY for an extension-host filesystem save. */
  prepareVisiblePlyExport(): { blob?: Blob; count: number; error?: string } {
    if (this.roiFiltering) {
      return { count: 0, error: "ROI is still filtering." };
    }
    const pointCount = this.getVisiblePointCount();
    if (pointCount > maximumWebviewExportPoints) {
      return {
        count: 0,
        error: `PLY export is limited to ${maximumWebviewExportPoints.toLocaleString()} points; narrow the ROI.`,
      };
    }
    const result = this.createVisiblePly();
    if (result.count === 0) return { count: 0 };
    return result;
  }

  /** Compatibility helper for standalone browser harnesses. */
  downloadVisiblePly(filename: string): { count: number; error?: string } {
    const result = this.prepareVisiblePlyExport();
    if (!result.blob || result.count === 0) return result;
    const anchor = document.createElement("a");
    const uri = URL.createObjectURL(result.blob);
    anchor.href = uri;
    anchor.download = safePlyFilename(filename);
    anchor.hidden = true;
    document.body.append(anchor);
    anchor.click();
    anchor.remove();
    window.setTimeout(() => URL.revokeObjectURL(uri), 0);
    return { count: result.count };
  }

  canDownloadVisiblePly(): boolean {
    const pointCount = this.getVisiblePointCount();
    return !this.roiFiltering && pointCount > 0 &&
      pointCount <= maximumWebviewExportPoints;
  }

  getBrowserExportPointLimit(): number {
    return maximumWebviewExportPoints;
  }

  /** Semantic state only; source paths stay host-owned. */
  getReviewShareLayers(): Array<Omit<ReviewShareLayer, "source_path">> {
    return [...this.layers.values()].map((layer) => {
      layer.group.updateMatrix();
      return {
        source_key: layer.sourceKey,
        local_to_world: matrixRows(layer.group.matrix, 4),
        style: {
          color_by: colorModeValue[layer.style.colorMode],
          color_map: colorMapValue[layer.style.colorMap],
          point_size: layer.style.pointSize,
          opacity: layer.style.opacity,
          scalar_min: finiteRange(layer.message.intensities)[0],
          scalar_max: finiteRange(layer.message.intensities)[1],
          fixed_color: layer.style.fixedColor.toArray() as [number, number, number],
          noise_color: layer.style.noiseColor.toArray() as [number, number, number],
          highlight_noise: layer.style.highlightNoise,
          intensity_equalize: layer.style.intensityEqualize,
        },
        visible: layer.style.visible,
      };
    });
  }

  getReviewShareCamera(bookmark: CameraBookmark): ReviewShareBookmark["camera"] {
    this.camera.updateMatrixWorld();
    const basis = matrixRows(this.camera.matrixWorld, 3);
    const target = [...bookmark.target] as [number, number, number];
    return {
      target,
      rotation_center: target,
      camera_to_world: basis,
      distance: Math.max(
        new THREE.Vector3(...bookmark.position).distanceTo(
          new THREE.Vector3(...bookmark.target),
        ),
        1e-9,
      ),
      fov_y_degrees: bookmark.fov,
    };
  }

  /**
   * Capture the actual WebGL viewport. Persistence deliberately remains in
   * the extension host: callers receive bytes, never a browser download URL.
   */
  capturePng(): Promise<Blob> {
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
    return new Promise((resolve, reject) => {
      this.renderer.domElement.toBlob((blob) => {
        if (!blob) {
          reject(new Error("WebGL canvas could not encode PNG"));
          return;
        }
        resolve(blob);
      }, "image/png");
    });
  }

  private fitCamera(
    direction: THREE.Vector3,
    layers: readonly PointLayer[],
  ): void {
    if (layers.length === 0) return;
    const center = new THREE.Vector3();
    let finitePointCount = 0;
    for (const layer of layers) {
      const count = layer.roiIndices !== undefined
        ? layer.roiFinitePointCount ?? 0
        : layer.finitePointCount;
      const localCentroid = layer.roiIndices !== undefined
        ? layer.roiCentroid
        : layer.center;
      if (!localCentroid || count === 0) continue;
      layer.group.updateMatrixWorld(true);
      center.addScaledVector(
        localCentroid.clone().applyMatrix4(layer.group.matrixWorld),
        count,
      );
      finitePointCount += count;
    }
    if (finitePointCount === 0) return;
    center.multiplyScalar(1 / finitePointCount);
    const radii: number[] = [];
    this.visitLayerSamples(layers, (world) => {
      radii.push(world.distanceTo(center));
    });
    const robustRadius = radii.length > 0
      ? percentile(radii, framingPercentile)
      : 1;
    const distance = Math.max(robustRadius * 2.8, 0.01);
    this.center.copy(center);
    this.radius = Math.max(robustRadius, 0.01);
    this.camera.up.set(0, 0, 1);
    if (Math.abs(direction.z) > 0.999) {
      this.camera.up.set(0, 1, 0);
    }
    this.camera.position.copy(center).addScaledVector(direction, distance);
    this.controls.target.copy(center);
    this.camera.lookAt(center);
    this.camera.updateMatrixWorld();
    const right = new THREE.Vector3().setFromMatrixColumn(this.camera.matrixWorld, 0);
    const up = new THREE.Vector3().setFromMatrixColumn(this.camera.matrixWorld, 1);
    const back = new THREE.Vector3().setFromMatrixColumn(this.camera.matrixWorld, 2);
    const slopes: number[] = [];
    this.visitLayerSamples(layers, (world) => {
      const pointOffset = world.clone().sub(center);
      const depth = distance - back.dot(pointOffset);
      if (depth <= 1e-9) return;
      slopes.push(Math.max(
        Math.abs(right.dot(pointOffset)) / (depth * this.camera.aspect),
        Math.abs(up.dot(pointOffset)) / depth,
      ));
    });
    const slope = slopes.length > 0 ? percentile(slopes, framingPercentile) :
      Math.tan(THREE.MathUtils.degToRad(25));
    this.camera.fov = THREE.MathUtils.clamp(
      THREE.MathUtils.radToDeg(2 * Math.atan(slope / framingFill)),
      minimumAutoFov,
      maximumAutoFov,
    );
    this.camera.updateProjectionMatrix();
    this.camera.near = Math.max(this.radius / 1000, 0.001);
    this.camera.far = Math.max(this.radius * 100, 100);
    this.controls.minDistance = this.radius * 0.001;
    this.controls.maxDistance = this.radius * 100;
    this.updateReferenceForLayers(layers);
    this.controls.update();
  }

  private visitLayerSamples(
    layers: readonly PointLayer[],
    visitor: (world: THREE.Vector3, layer: PointLayer) => void,
  ): void {
    const local = new THREE.Vector3();
    const world = new THREE.Vector3();
    // The old per-layer cap let a many-layer review spend N × 100k samples
    // during every fit/reference update. Allocate one uniform budget across
    // all participating layers instead.
    const candidates = layers.map((layer) =>
      layer.roiIndices ?? layer.message.pointOrder,
    );
    const sampleCounts = allocateGlobalSampleBudget(
      candidates.map((indices) => indices.length),
      maximumFramingSamples,
    );
    let visited = 0;
    for (const [layerIndex, layer] of layers.entries()) {
      layer.group.updateMatrixWorld(true);
      const indices = candidates[layerIndex];
      const sampleCount = sampleCounts[layerIndex];
      for (let sample = 0; sample < sampleCount; ++sample) {
        const candidate = Math.min(
          indices.length - 1,
          Math.floor(sample * indices.length / sampleCount),
        );
        const offset = indices[candidate] * 3;
        const x = layer.message.positions[offset];
        const y = layer.message.positions[offset + 1];
        const z = layer.message.positions[offset + 2];
        if (!Number.isFinite(x + y + z)) continue;
        visitor(world.copy(local.set(x, y, z)).applyMatrix4(layer.group.matrixWorld), layer);
        ++visited;
      }
    }
    // Existing browser smoke diagnostics intentionally expose bounded work.
    this.container.dataset.framingSamples = String(visited);
  }

  showTrajectories(
    paths: Array<Array<[number, number, number]>>,
    currentPose?: number[],
  ): void {
    if (paths !== this.trajectoryPaths) {
      this.clearTrajectories();
      this.trajectoryPaths = paths;
      const palette = [0xffcc33, 0x33ccff];
      paths.forEach((path, index) => {
        if (path.length < 2) return;
        const coordinates = new Float32Array(path.length * 3);
        path.forEach(([x, y, z], pointIndex) => {
          coordinates.set([x, y, z], pointIndex * 3);
        });
        const geometry = new THREE.BufferGeometry();
        geometry.setAttribute("position", new THREE.BufferAttribute(coordinates, 3));
        const line = new THREE.Line(
          geometry,
          new THREE.LineBasicMaterial({ color: palette[index % palette.length] }),
        );
        this.trajectories.push(line);
        this.trajectoryGroup.add(line);
      });
    }
    this.trajectoryGroup.matrix.copy(
      currentPose
        ? new THREE.Matrix4().fromArray(currentPose).invert()
        : new THREE.Matrix4(),
    );
    this.trajectoryGroup.matrixWorldNeedsUpdate = true;
    this.invalidate();
  }

  dispose(): void {
    if (this.frame !== 0) cancelAnimationFrame(this.frame);
    this.frame = 0;
    this.observer.disconnect();
    this.themeObserver.disconnect();
    this.renderer.domElement.removeEventListener(
      "pointerdown",
      this.beginRoll,
      true,
    );
    this.renderer.domElement.removeEventListener(
      "pointermove",
      this.continueRoll,
      true,
    );
    this.renderer.domElement.removeEventListener(
      "pointerup",
      this.endRoll,
      true,
    );
    this.renderer.domElement.removeEventListener(
      "pointercancel",
      this.endRoll,
      true,
    );
    this.renderer.domElement.removeEventListener(
      "pointerdown",
      this.beginMeasurement,
      true,
    );
    this.renderer.domElement.removeEventListener(
      "pointermove",
      this.continueMeasurement,
      true,
    );
    this.renderer.domElement.removeEventListener(
      "pointerup",
      this.endMeasurement,
      true,
    );
    this.renderer.domElement.removeEventListener(
      "pointercancel",
      this.cancelMeasurement,
      true,
    );
    this.controls.removeEventListener("change", this.invalidate);
    this.controls.dispose();
    this.clearCloud();
    this.clearRoi();
    this.clearMeasurement();
    this.clearTrajectories();
    this.clearGrid();
    this.scene.remove(this.axes);
    this.scene.remove(this.measurementGroup);
    for (const arrow of this.axisArrows) {
      arrow.line.geometry.dispose();
      disposeMaterial(arrow.line.material);
      arrow.cone.geometry.dispose();
      disposeMaterial(arrow.cone.material);
    }
    this.renderer.dispose();
    this.renderer.forceContextLoss();
    this.renderer.domElement.remove();
  }

  private updateBounds(message: DecodedCloudMessage): PointStatistics {
    this.framingPositions = message.positions;
    const statistics = pointStatistics(message.positions);
    if (!message.bounds) {
      this.center.copy(statistics.centroid);
      this.radius = 1;
      this.referenceMinimum.set(-5, -5, -5);
      this.referenceMaximum.set(5, 5, 5);
      this.updateReferenceHelpers();
      return statistics;
    }
    const min = new THREE.Vector3(...message.bounds.min);
    const max = new THREE.Vector3(...message.bounds.max);
    this.referenceMinimum.copy(min);
    this.referenceMaximum.copy(max);
    this.center.set(
      statistics.finitePointCount > 0 ? statistics.centroid.x : (min.x + max.x) * 0.5,
      statistics.finitePointCount > 0 ? statistics.centroid.y : (min.y + max.y) * 0.5,
      statistics.finitePointCount > 0 ? statistics.centroid.z : (min.z + max.z) * 0.5,
    );
    this.radius = Math.max(min.distanceTo(max) * 0.5, 0.01);
    this.camera.near = Math.max(this.radius / 1000, 0.001);
    this.camera.far = Math.max(this.radius * 100, 100);
    this.camera.updateProjectionMatrix();
    this.controls.minDistance = this.radius * 0.001;
    this.controls.maxDistance = this.radius * 100;
    this.updateReferenceHelpers(min, max);
    return statistics;
  }

  private updateReferenceForLayers(layers: readonly PointLayer[]): void {
    if (layers.length === 0) {
      this.referenceMinimum.set(-5, -5, -5);
      this.referenceMaximum.set(5, 5, 5);
      this.updateReferenceHelpers();
      return;
    }
    const minimum = new THREE.Vector3(
      Number.POSITIVE_INFINITY,
      Number.POSITIVE_INFINITY,
      Number.POSITIVE_INFINITY,
    );
    const maximum = new THREE.Vector3(
      Number.NEGATIVE_INFINITY,
      Number.NEGATIVE_INFINITY,
      Number.NEGATIVE_INFINITY,
    );
    this.visitLayerSamples(layers, (world) => {
      minimum.min(world);
      maximum.max(world);
    });
    if (!Number.isFinite(minimum.x + minimum.y + minimum.z) ||
        !Number.isFinite(maximum.x + maximum.y + maximum.z)) return;
    this.referenceMinimum.copy(minimum);
    this.referenceMaximum.copy(maximum);
    this.updateReferenceHelpers(minimum, maximum);
  }

  private updateReferenceHelpers(
    minimum = new THREE.Vector3(-5, -5, -5),
    maximum = new THREE.Vector3(5, 5, 5),
  ): void {
    const palette = this.updateReferenceColors();
    const sceneHalfExtent = Math.max(
      Math.abs(minimum.x),
      Math.abs(maximum.x),
      Math.abs(minimum.y),
      Math.abs(maximum.y),
      Math.abs(minimum.z),
      Math.abs(maximum.z),
      0.5,
    );
    this.gridSpacing = niceStep((sceneHalfExtent * 2) / 12);
    const gridHalfSize = Math.min(
      Math.max(1, Math.ceil(sceneHalfExtent / this.gridSpacing)) *
        this.gridSpacing,
      maximumFloat32 / 2,
    );
    const gridSize = gridHalfSize * 2;
    const divisions = Math.max(2, Math.round(gridSize / this.gridSpacing));

    this.clearGrid();
    const rotations = [
      new THREE.Euler(Math.PI * 0.5, 0, 0),
      new THREE.Euler(0, 0, 0),
      new THREE.Euler(0, 0, Math.PI * 0.5),
    ];
    for (const rotation of rotations) {
      const grid = new THREE.GridHelper(
        gridSize,
        divisions,
        palette.centerColor,
        palette.gridColor,
      );
      grid.rotation.copy(rotation);
      grid.visible = this.gridVisible;
      for (const material of materialList(grid.material)) {
        material.transparent = true;
        material.opacity = palette.opacity;
        material.depthWrite = false;
      }
      this.grids.push(grid);
    }
    this.container.dataset.gridPlanes = this.grids.map((grid) => {
      if (Math.abs(grid.rotation.x - Math.PI * 0.5) < 1e-6) return "xy";
      if (Math.abs(grid.rotation.z - Math.PI * 0.5) < 1e-6) return "yz";
      return "xz";
    }).join(",");
    this.container.dataset.gridContrast = palette.contrast.toFixed(3);
    this.scene.add(...this.grids);

    const axisLength = Math.min(
      Math.max(sceneHalfExtent, this.gridSpacing),
      maximumFloat32,
    );
    for (const arrow of this.axisArrows) {
      arrow.setLength(axisLength, axisLength * 0.12, axisLength * 0.055);
    }
    this.axes.visible = this.axesVisible;
    this.container.dataset.axesArrowheads = String(this.axisArrows.every(
      (arrow) =>
        arrow.cone.geometry.getAttribute("position")?.count > 0 &&
        arrow.cone.scale.toArray().every(Number.isFinite),
    ));
    this.container.dataset.gridSpacing = String(this.gridSpacing);
  }

  private updateReferenceColors(): {
    centerColor: THREE.ColorRepresentation;
    gridColor: THREE.ColorRepresentation;
    opacity: number;
    contrast: number;
  } {
    const background = this.scene.background instanceof THREE.Color
      ? this.scene.background
      : new THREE.Color("#1e1e1e");
    const luminance =
      background.r * 0.2126 + background.g * 0.7152 + background.b * 0.0722;
    const lightGrid = new THREE.Color(0xaab8c8);
    const darkGrid = new THREE.Color(0x4b5563);
    const lightContrast = contrastRatio(luminance, colorLuminance(lightGrid));
    const darkContrast = contrastRatio(luminance, colorLuminance(darkGrid));
    const lightBackground = darkContrast > lightContrast;
    const axisColors = [
      new THREE.Color(lightBackground ? "#ad3838" : "#ffaaaa"),
      new THREE.Color(lightBackground ? "#298c4a" : "#aaffc3"),
      new THREE.Color(lightBackground ? "#3361b3" : "#aaccff"),
    ];
    this.axisArrows.forEach((arrow, index) => {
      arrow.setColor(axisColors[index]);
    });
    this.container.dataset.axesPalette = axisColors.every(
      (color) => color.r >= 0.01 && color.g >= 0.01 && color.b >= 0.01,
    )
      ? "pastel"
      : "other";
    const centerColor = lightBackground ? 0x303846 : 0xdde5ef;
    const gridColor = lightBackground ? 0x4b5563 : 0xaab8c8;
    const opacity = lightBackground ? 0.72 : 0.42;
    const blendedLuminance =
      colorLuminance(new THREE.Color(gridColor)) * opacity +
      luminance * (1 - opacity);
    return {
      centerColor,
      gridColor,
      opacity,
      contrast: contrastRatio(luminance, blendedLuminance),
    };
  }

  private clearGrid(): void {
    for (const grid of this.grids) {
      this.scene.remove(grid);
      grid.geometry.dispose();
      disposeMaterial(grid.material);
    }
    this.grids.length = 0;
  }

  private clearCloud(): void {
    for (const layer of this.layers.values()) this.disposeLayer(layer);
    this.layers.clear();
    this.gpuBytesInUse = 0;
    this.cpuBytesInUse = 0;
    this.roiBytesInUse = 0;
    this.pendingGpuLayerKeys.clear();
    this.clearActiveLayer();
  }

  private clearRoi(): void {
    this.cancelRoiFilter();
    this.roi = undefined;
    this.clearRoiResults();
    if (!this.roiHelper) return;
    this.scene.remove(this.roiHelper);
    this.roiHelper.geometry.dispose();
    disposeMaterial(this.roiHelper.material);
    this.roiHelper = undefined;
  }

  /** Release retained full-ROI index buffers without removing the ROI box. */
  private clearRoiResults(): void {
    this.roiBytesInUse = 0;
    for (const layer of this.layers.values()) {
      layer.roiIndices = undefined;
      layer.roiBytes = 0;
      layer.roiCentroid = undefined;
      layer.roiFinitePointCount = undefined;
      if (layer.renderQuality === "lod") {
        this.replacePackedLayerGeometry(
          layer,
          layer.renderIndices ?? layer.message.lodIndices,
        );
      } else {
        this.updateFullLayerIndices(layer, layer.message.pointOrder);
        this.pendingGpuLayerKeys.add(layer.sourceKey);
        this.rebuildLayerLodGeometry(layer, layer.message.lodIndices);
      }
    }
    this.roiIndices = this.activeLayer()?.roiIndices;
  }

  private projectedRoiReserveBytes(
    sourceKey?: string,
    candidate?: DecodedCloudMessage,
    replacing = false,
  ): number {
    let total = 0;
    for (const [key, layer] of this.layers) {
      if (replacing && key === sourceKey) continue;
      total += layer.message.pointOrder.byteLength;
    }
    return total + (candidate?.pointOrder.byteLength ?? 0);
  }

  private canReserveRoiIndices(): boolean {
    return this.cpuBytesInUse + this.projectedRoiReserveBytes() <=
      this.cpuByteBudget;
  }

  private scheduleRoiFilter(roi: RoiBox): void {
    if (this.roiFilterTimer !== undefined) {
      window.clearTimeout(this.roiFilterTimer);
      this.roiFilterTimer = undefined;
    }
    // Transform edits can schedule a new generation while old full index
    // buffers remain retained. Release them before scanning so peak storage
    // stays inside the preflight reserve rather than doubling per layer.
    if (this.roiBytesInUse > 0) this.clearRoiResults();
    const generation = ++this.roiGeneration;
    this.roiFiltering = true;
    this.renderRoiPreviewAtMost10Hz(roi);
    this.roiChangeHandler?.();
    this.roiFilterTimer = window.setTimeout(() => {
      this.roiFilterTimer = undefined;
      this.renderRoiPreviewAtMost10Hz(roi);
      void this.applyRoiGeneration(roi, generation);
    }, 150);
  }

  private renderRoiPreviewAtMost10Hz(roi: RoiBox): void {
    const now = performance.now();
    if (now - this.lastRoiPreviewAt < 100) return;
    this.lastRoiPreviewAt = now;
    // A bounded LOD preview is ready while the full point-order scan yields to
    // animation frames. Calls during a gizmo drag are capped at 10 Hz.
    for (const layer of this.layers.values()) {
      const preview = filterClosedWorldRoi(layer, this.lodPreviewIndices(layer), roi);
      if (layer.renderQuality === "lod") {
        this.replacePackedLayerGeometry(layer, preview.indices);
      } else {
        this.rebuildLayerLodGeometry(layer, preview.indices);
      }
    }
  }

  private async applyRoiGeneration(roi: RoiBox, generation: number): Promise<void> {
    const results: Array<{
      layer: PointLayer;
      full: RoiFilterResult;
      lod: RoiFilterResult;
    }> = [];
    try {
      for (const layer of this.layers.values()) {
        const full = await filterClosedWorldRoiCooperatively(
          layer,
          layer.message.pointOrder,
          roi,
          () => generation === this.roiGeneration,
        );
        if (generation !== this.roiGeneration) return;
        const lod = filterClosedWorldRoi(layer, this.lodPreviewIndices(layer), roi);
        results.push({ layer, full, lod });
      }
      if (generation !== this.roiGeneration) return;
      for (const { layer, full, lod } of results) {
        this.applyRoiResult(layer, full, lod);
      }
      this.roiIndices = this.activeLayer()?.roiIndices;
      this.roiFiltering = false;
      this.roiChangeHandler?.();
      this.invalidate();
    } catch {
      if (generation !== this.roiGeneration) return;
      this.roiFiltering = false;
      this.roiChangeHandler?.();
    }
  }

  private applyRoiResult(
    layer: PointLayer,
    full: RoiFilterResult,
    lod: RoiFilterResult,
  ): void {
    this.roiBytesInUse = Math.max(0, this.roiBytesInUse - layer.roiBytes) +
      full.storageBytes;
    layer.roiIndices = full.indices;
    layer.roiBytes = full.storageBytes;
    layer.roiCentroid = full.centroid;
    layer.roiFinitePointCount = full.finitePointCount;
    if (layer.renderQuality === "lod") {
      this.replacePackedLayerGeometry(layer, lod.indices);
    } else {
      this.updateFullLayerIndices(layer, layer.roiIndices);
      this.pendingGpuLayerKeys.add(layer.sourceKey);
      this.rebuildLayerLodGeometry(
        layer,
        lod.indices,
      );
    }
  }

  private lodPreviewIndices(layer: PointLayer): Uint32Array {
    const candidates = layer.renderQuality === "lod"
      ? layer.renderIndices
      : layer.message.lodIndices;
    if (candidates && candidates.length > 0) return candidates;
    return uniformlySampleIndices(layer.message.pointOrder, maximumFramingSamples);
  }

  private cancelRoiFilter(): void {
    ++this.roiGeneration;
    if (this.roiFilterTimer !== undefined) {
      window.clearTimeout(this.roiFilterTimer);
      this.roiFilterTimer = undefined;
    }
    this.roiFiltering = false;
  }

  private rebuildLayerLodGeometry(
    layer: PointLayer,
    indices: ArrayLike<number>,
  ): void {
    if (!layer.lodCloud) return;
    const next = gatherGeometry(layer.message, indices, layer.equalizedIntensities);
    layer.lodCloud.geometry.dispose();
    layer.lodCloud.geometry = next;
    this.pendingGpuLayerKeys.add(layer.sourceKey);
  }

  private replacePackedLayerGeometry(
    layer: PointLayer,
    indices: ArrayLike<number>,
  ): void {
    const next = gatherGeometry(layer.message, indices, layer.equalizedIntensities);
    const previousBytes = layer.gpuBytes;
    layer.cloud.geometry.dispose();
    layer.cloud.geometry = next;
    // LOD-only geometry is physically rebuilt for ROI preview/reset. Keep
    // accounting aligned with its current packed buffer so Clear ROI cannot
    // silently grow GPU use beyond the next Add-layer budget calculation.
    if (layer.renderQuality === "lod") {
      layer.gpuBytes = estimatedPackedGpuBytes(layer.message, indices.length);
      this.gpuBytesInUse = Math.max(
        0,
        this.gpuBytesInUse - previousBytes + layer.gpuBytes,
      );
    }
    this.pendingGpuLayerKeys.add(layer.sourceKey);
  }

  private updateFullLayerIndices(
    layer: PointLayer,
    indices: ArrayLike<number>,
  ): void {
    const attribute = layer.fullIndex;
    if (!attribute || indices.length > attribute.count) {
      throw new Error("full ROI index capacity is invalid");
    }
    (attribute.array as Uint32Array).set(indices, 0);
    // Avoid resending the full N-point index buffer for a narrow ROI.
    attribute.clearUpdateRanges();
    attribute.addUpdateRange(0, indices.length);
    attribute.needsUpdate = true;
    layer.cloud.geometry.setDrawRange(0, indices.length);
  }

  private createVisiblePly(): { blob: Blob; count: number } {
    let count = 0;
    const layers = this.visibleLayers();
    for (const layer of layers) {
      layer.group.updateMatrixWorld(true);
      const world = new THREE.Vector3();
      for (const index of layer.roiIndices ?? layer.message.pointOrder) {
        const offset = index * 3;
        world.fromArray(layer.message.positions, offset).applyMatrix4(layer.group.matrixWorld);
        if (Number.isFinite(world.x + world.y + world.z)) ++count;
      }
    }
    const properties = [
      "property float x", "property float y", "property float z",
      "property uchar red", "property uchar green", "property uchar blue",
      "property float intensity", "property uchar noise",
    ];
    const chunks: BlobPart[] = [[
      "ply", "format ascii 1.0", `element vertex ${count}`,
      ...properties, "end_header", "",
    ].join("\n")];
    let lines: string[] = [];
    const world = new THREE.Vector3();
    for (const layer of layers) {
      layer.group.updateMatrixWorld(true);
      const message = layer.message;
      const fallback = layer.style.fixedColor;
      for (const index of layer.roiIndices ?? message.pointOrder) {
        const offset = index * 3;
        world.fromArray(message.positions, offset).applyMatrix4(layer.group.matrixWorld);
        if (!Number.isFinite(world.x + world.y + world.z)) continue;
        const color = index * 3;
        const red = message.hasColor ? message.colors[color] : Math.round(fallback.r * 255);
        const green = message.hasColor ? message.colors[color + 1] : Math.round(fallback.g * 255);
        const blue = message.hasColor ? message.colors[color + 2] : Math.round(fallback.b * 255);
        const row = [
          String(world.x), String(world.y), String(world.z),
          String(red), String(green), String(blue),
          String(message.hasIntensity ? message.intensities[index] : 0),
          String(message.hasNoise ? message.noises[index] : 0),
        ];
        lines.push(`${row.join(" ")}\n`);
        if (lines.length >= 8_192) {
          chunks.push(lines.join(""));
          lines = [];
        }
      }
    }
    if (lines.length) chunks.push(lines.join(""));
    return { blob: new Blob(chunks, { type: "application/octet-stream" }), count };
  }

  private disposeLayer(layer: PointLayer): void {
    this.scene.remove(layer.group);
    layer.cloud.geometry.dispose();
    layer.cloud.material.dispose();
    if (layer.lodCloud) {
      layer.lodCloud.geometry.dispose();
      layer.lodCloud.material.dispose();
    }
  }

  private updateCloudUniform(name: string, value: unknown): void {
    for (const cloud of [this.cloud, this.lodCloud]) {
      const uniform = cloud?.material.uniforms[name];
      if (uniform) uniform.value = value;
    }
  }

  private clearTrajectories(): void {
    for (const line of this.trajectories) {
      this.trajectoryGroup.remove(line);
      line.geometry.dispose();
      (line.material as THREE.Material).dispose();
    }
    this.trajectories.length = 0;
    this.trajectoryPaths = undefined;
  }

  private resize(): void {
    const width = Math.max(this.container.clientWidth, 1);
    const height = Math.max(this.container.clientHeight, 1);
    this.renderer.setSize(width, height, false);
    this.updatePhysicalPointSize();
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    this.invalidate();
  }

  private render(): void {
    this.frame = 0;
    this.controls.update();
    // Preserve active-layer compatibility for browser smoke coverage.
    if (this.cloud) {
      const useLod =
        this.lodCloud !== undefined &&
        this.camera.position.distanceTo(this.controls.target) >
        this.radius * 2.5;
      const showPoints = this.pointSize > 0;
      this.cloud.visible = showPoints && !useLod;
      if (this.lodCloud) this.lodCloud.visible = showPoints && useLod;
    }
    const opaque: PointLayer[] = [];
    const transparent: PointLayer[] = [];
    const roiPreview = this.roiFiltering && this.roi !== undefined;
    for (const layer of this.layers.values()) {
      const useLod = layer.lodCloud !== undefined &&
        this.camera.position.distanceTo(this.controls.target) > layer.radius * 2.5;
      const showPoints = layer.style.visible && layer.style.pointSize > 0;
      layer.cloud.visible = showPoints && !useLod &&
        (!roiPreview || layer.renderQuality === "lod");
      if (layer.lodCloud) {
        layer.lodCloud.visible = showPoints && (useLod || roiPreview);
      }
      (layer.style.opacity < 0.999 ? transparent : opaque).push(layer);
    }
    // v1 transparency is layer-granularity painter ordering. Interleaving
    // point sets cannot be correctly sorted without OIT.
    opaque.forEach((layer, index) => this.setLayerRenderOrder(layer, index));
    this.camera.updateMatrixWorld();
    this.camera.matrixWorldInverse.copy(this.camera.matrixWorld).invert();
    transparent.sort((left, right) =>
      this.cameraDepth(right) - this.cameraDepth(left)
    ).forEach((layer, index) => this.setLayerRenderOrder(layer, opaque.length + index));
    try {
      this.renderer.render(this.scene, this.camera);
      const context = this.renderer.getContext();
      if (context.getError() === context.OUT_OF_MEMORY) {
        // An already-rejected minimum-LOD layer can keep reporting OOM. Only
        // recovery work for a geometry changed since a successful frame.
        if (this.pendingGpuLayerKeys.size > 0) {
          this.recoverGpuAllocationFailure();
        }
      } else {
        this.pendingGpuLayerKeys.clear();
      }
    } catch (error) {
      const detail = error instanceof Error ? error.message : String(error);
      if (/out of memory|allocation/i.test(detail)) {
        if (this.pendingGpuLayerKeys.size > 0) {
          this.recoverGpuAllocationFailure();
        }
      } else {
        this.rendererWarningHandler?.(detail);
      }
    }
  }

  private readonly invalidate = (): void => {
    if (this.frame === 0) this.frame = requestAnimationFrame(this.render);
  };

  private updatePhysicalPointSize(): void {
    for (const layer of this.layers.values()) this.applyLayerStyle(layer);
  }

  private setLayerRenderOrder(layer: PointLayer, order: number): void {
    layer.cloud.renderOrder = order;
    if (layer.lodCloud) layer.lodCloud.renderOrder = order;
  }

  private worldBoundsCenter(layer: PointLayer): THREE.Vector3 {
    layer.group.updateMatrixWorld(true);
    return layer.boundsCenter.clone().applyMatrix4(layer.group.matrixWorld);
  }

  private cameraDepth(layer: PointLayer): number {
    // Camera-space -Z is positive in front of a PerspectiveCamera. Sort far
    // to near; Euclidean distance is wrong for points beside the camera axis.
    return -this.worldBoundsCenter(layer).applyMatrix4(this.camera.matrixWorldInverse).z;
  }

  private recoverGpuAllocationFailure(): void {
    const candidates = [...this.pendingGpuLayerKeys]
      .map((sourceKey) => this.layers.get(sourceKey))
      .filter((layer): layer is PointLayer => layer !== undefined)
      .sort((left, right) => right.gpuBytes - left.gpuBytes);
    const layer = candidates[0] ?? this.activeLayer();
    if (!layer || !this.demoteLayerGpuGeometry(layer)) {
      const name = layer?.name ?? "active layer";
      // Preserve every existing review layer, especially the active one. A
      // renderer allocation failure is not user intent to delete evidence.
      this.pendingGpuLayerKeys.clear();
      this.rendererWarningHandler?.(
        `GPU allocation failed; retained ${name} at minimum LOD`,
      );
      return;
    }
    this.pendingGpuLayerKeys.clear();
    this.pendingGpuLayerKeys.add(layer.sourceKey);
    this.rendererWarningHandler?.(
      `GPU allocation failed; retried ${layer.name} with a smaller LOD`,
    );
  }

  private demoteLayerGpuGeometry(layer: PointLayer): boolean {
    // `renderIndices` remains a global LOD basis. When an ROI is active we
    // filter that reduced global basis again, never restore points outside
    // the closed world-space crop while recovering from an upload failure.
    const source = layer.renderIndices ?? this.lodPreviewIndices(layer);
    if (!source || source.length <= minimumGpuLodPoints) return false;
    const globalIndices = uniformlySampleIndices(
      source,
      Math.max(minimumGpuLodPoints, Math.floor(source.length / 2)),
    );
    if (globalIndices.length >= source.length) return false;
    const nextIndices = this.roi
      ? filterClosedWorldRoi(layer, globalIndices, this.roi).indices
      : globalIndices;
    if (layer.lodCloud) {
      layer.group.remove(layer.lodCloud);
      layer.lodCloud.geometry.dispose();
      layer.lodCloud.material.dispose();
      layer.lodCloud = undefined;
    }
    layer.renderQuality = "lod";
    layer.renderIndices = globalIndices;
    this.replacePackedLayerGeometry(layer, nextIndices);
    if (layer.sourceKey === this.activeLayerKey) this.adoptLayer(layer);
    this.invalidate();
    return true;
  }

  private readonly beginRoll = (event: PointerEvent): void => {
    if (event.button !== 0 || !event.shiftKey) return;
    this.rolling = true;
    this.rollPointer = event.pointerId;
    this.previousRollX = event.clientX;
    this.renderer.domElement.setPointerCapture(event.pointerId);
    event.preventDefault();
    event.stopImmediatePropagation();
  };

  private readonly continueRoll = (event: PointerEvent): void => {
    if (!this.rolling || event.pointerId !== this.rollPointer) return;
    const delta = event.clientX - this.previousRollX;
    this.previousRollX = event.clientX;
    const axis = this.controls.target
      .clone()
      .sub(this.camera.position)
      .normalize();
    const angle =
      -2 * Math.PI * delta /
      Math.max(this.renderer.domElement.clientWidth, 1);
    this.camera.up.applyAxisAngle(axis, angle).normalize();
    this.camera.lookAt(this.controls.target);
    this.invalidate();
    event.preventDefault();
    event.stopImmediatePropagation();
  };

  private readonly endRoll = (event: PointerEvent): void => {
    if (!this.rolling || event.pointerId !== this.rollPointer) return;
    this.rolling = false;
    this.rollPointer = -1;
    if (this.renderer.domElement.hasPointerCapture(event.pointerId)) {
      this.renderer.domElement.releasePointerCapture(event.pointerId);
    }
    event.preventDefault();
    event.stopImmediatePropagation();
  };

  private readonly beginMeasurement = (event: PointerEvent): void => {
    if (!this.measurementEnabled || event.button !== 0 || event.shiftKey) return;
    this.measurementPointer = {
      id: event.pointerId,
      x: event.clientX,
      y: event.clientY,
      moved: false,
    };
    this.renderer.domElement.setPointerCapture(event.pointerId);
    event.preventDefault();
    event.stopImmediatePropagation();
  };

  private readonly continueMeasurement = (event: PointerEvent): void => {
    const pointer = this.measurementPointer;
    if (!pointer || event.pointerId !== pointer.id) return;
    if (Math.hypot(event.clientX - pointer.x, event.clientY - pointer.y) > 4) {
      pointer.moved = true;
    }
    event.preventDefault();
    event.stopImmediatePropagation();
  };

  private readonly endMeasurement = (event: PointerEvent): void => {
    const pointer = this.measurementPointer;
    if (!pointer || event.pointerId !== pointer.id) return;
    this.measurementPointer = undefined;
    if (this.renderer.domElement.hasPointerCapture(event.pointerId)) {
      this.renderer.domElement.releasePointerCapture(event.pointerId);
    }
    if (!pointer.moved) this.measurementPickHandler?.(
      this.pickNearest(event.clientX, event.clientY),
    );
    event.preventDefault();
    event.stopImmediatePropagation();
  };

  private readonly cancelMeasurement = (event: PointerEvent): void => {
    const pointer = this.measurementPointer;
    if (!pointer || event.pointerId !== pointer.id) return;
    this.measurementPointer = undefined;
    if (this.renderer.domElement.hasPointerCapture(event.pointerId)) {
      this.renderer.domElement.releasePointerCapture(event.pointerId);
    }
    event.preventDefault();
    event.stopImmediatePropagation();
  };

  private pickNearest(clientX: number, clientY: number): PointPick | undefined {
    const bounds = this.renderer.domElement.getBoundingClientRect();
    if (bounds.width <= 0 || bounds.height <= 0) return undefined;
    const visible = this.visibleLayers();
    // More than four visible layers uses only a visible active layer. The
    // Inspector reports this explicitly; silently probing all layers would
    // make a click stall on dense reviews.
    const layers = visible.length <= 4 ? visible :
      visible.filter((layer) => layer.sourceKey === this.activeLayerKey);
    if (layers.length === 0) return undefined;
    let nearest: PointPick | undefined;
    let nearestDistanceSquared = Number.POSITIVE_INFINITY;
    let nearestDepth = Number.POSITIVE_INFINITY;
    const world = new THREE.Vector3();
    const local = new THREE.Vector3();
    const projected = new THREE.Vector3();
    this.camera.updateMatrixWorld();
    for (const layer of layers) {
      layer.group.updateMatrixWorld(true);
      const candidates = layer.roiIndices ?? layer.message.pointOrder;
      const stride = Math.max(1, Math.ceil(candidates.length / maximumPickingCandidates));
      const maximumDistanceSquared = Math.max(12, layer.style.pointSize * 4) ** 2;
      for (let candidate = 0; candidate < candidates.length; candidate += stride) {
        const index = candidates[candidate];
        const offset = index * 3;
        const x = layer.message.positions[offset];
        const y = layer.message.positions[offset + 1];
        const z = layer.message.positions[offset + 2];
        if (!Number.isFinite(x + y + z)) continue;
        world.copy(local.set(x, y, z)).applyMatrix4(layer.group.matrixWorld);
        projected.copy(world).project(this.camera);
        if (projected.z < -1 || projected.z > 1) continue;
        const screenX = bounds.left + (projected.x + 1) * bounds.width * 0.5;
        const screenY = bounds.top + (1 - projected.y) * bounds.height * 0.5;
        const distanceSquared = (screenX - clientX) ** 2 + (screenY - clientY) ** 2;
        if (distanceSquared > Math.min(nearestDistanceSquared, maximumDistanceSquared)) continue;
        const depth = this.camera.position.distanceToSquared(world);
        if (distanceSquared === nearestDistanceSquared && depth >= nearestDepth)
          continue;
        nearestDistanceSquared = distanceSquared;
        nearestDepth = depth;
        nearest = {
          index,
          point: [world.x, world.y, world.z],
          sourceKey: layer.sourceKey,
          layerName: layer.name,
        };
      }
    }
    return nearest;
  }
}

function isValidRoi(roi: RoiBox): boolean {
  return roi.min.length === 3 && roi.max.length === 3 &&
    roi.min.every(Number.isFinite) && roi.max.every(Number.isFinite) &&
    roi.min.every((value, axis) => value <= roi.max[axis]);
}

function copyRoi(roi: RoiBox): RoiBox {
  return { min: [...roi.min] as RoiBox["min"], max: [...roi.max] as RoiBox["max"] };
}

function filterClosedWorldRoi(
  layer: PointLayer,
  candidates: ArrayLike<number>,
  roi: RoiBox,
): RoiFilterResult {
  // Preview candidates are bounded, but use typed storage here too so the
  // ROI path never converts a JS number[] into another full index allocation.
  const included = new Uint32Array(candidates.length);
  let includedCount = 0;
  const local = new THREE.Vector3();
  const world = new THREE.Vector3();
  const centroid = new THREE.Vector3();
  layer.group.updateMatrixWorld(true);
  for (let candidate = 0; candidate < candidates.length; ++candidate) {
    const index = candidates[candidate];
    const offset = index * 3;
    const x = layer.message.positions[offset];
    const y = layer.message.positions[offset + 1];
    const z = layer.message.positions[offset + 2];
    if (!Number.isFinite(x + y + z)) continue;
    world.copy(local.set(x, y, z)).applyMatrix4(layer.group.matrixWorld);
    if (world.x >= roi.min[0] && world.x <= roi.max[0] &&
        world.y >= roi.min[1] && world.y <= roi.max[1] &&
        world.z >= roi.min[2] && world.z <= roi.max[2]) {
      included[includedCount++] = index;
      // Keep centroid local; framing applies the layer matrix exactly once.
      centroid.add(local);
    }
  }
  const finitePointCount = includedCount;
  if (finitePointCount > 0) centroid.multiplyScalar(1 / finitePointCount);
  return {
    indices: included.subarray(0, includedCount),
    centroid,
    finitePointCount,
    storageBytes: included.byteLength,
  };
}

async function filterClosedWorldRoiCooperatively(
  layer: PointLayer,
  candidates: ArrayLike<number>,
  roi: RoiBox,
  isCurrent: () => boolean,
): Promise<RoiFilterResult> {
  // Reserve exactly one uint32 slot per candidate. The caller preflights this
  // worst-case backing storage; return a subarray so no second full copy is
  // made when an ROI contains every point.
  const included = new Uint32Array(candidates.length);
  let includedCount = 0;
  const local = new THREE.Vector3();
  const world = new THREE.Vector3();
  const centroid = new THREE.Vector3();
  layer.group.updateMatrixWorld(true);
  const matrix = layer.group.matrixWorld.clone();
  const sliceSize = 20_000;
  for (let start = 0; start < candidates.length; start += sliceSize) {
    if (!isCurrent()) throw new Error("stale ROI generation");
    const end = Math.min(candidates.length, start + sliceSize);
    for (let candidate = start; candidate < end; ++candidate) {
      const index = candidates[candidate];
      const offset = index * 3;
      const x = layer.message.positions[offset];
      const y = layer.message.positions[offset + 1];
      const z = layer.message.positions[offset + 2];
      if (!Number.isFinite(x + y + z)) continue;
      world.copy(local.set(x, y, z)).applyMatrix4(matrix);
      if (world.x >= roi.min[0] && world.x <= roi.max[0] &&
          world.y >= roi.min[1] && world.y <= roi.max[1] &&
          world.z >= roi.min[2] && world.z <= roi.max[2]) {
        included[includedCount++] = index;
        centroid.add(local);
      }
    }
    if (end < candidates.length) await nextFrame();
  }
  const finitePointCount = includedCount;
  if (finitePointCount > 0) centroid.multiplyScalar(1 / finitePointCount);
  return {
    indices: included.subarray(0, includedCount),
    centroid,
    finitePointCount,
    storageBytes: included.byteLength,
  };
}

function nextFrame(): Promise<void> {
  return new Promise((resolve) => window.requestAnimationFrame(() => resolve()));
}

function layerTransform(group: THREE.Group): LayerTransform {
  return {
    position: group.position.toArray() as LayerTransform["position"],
    rotation: [
      THREE.MathUtils.radToDeg(group.rotation.x),
      THREE.MathUtils.radToDeg(group.rotation.y),
      THREE.MathUtils.radToDeg(group.rotation.z),
    ],
    scale: group.scale.toArray() as LayerTransform["scale"],
  };
}

function applyLayerTransform(group: THREE.Group, transform: LayerTransform): void {
  group.position.fromArray(transform.position);
  group.rotation.set(
    THREE.MathUtils.degToRad(transform.rotation[0]),
    THREE.MathUtils.degToRad(transform.rotation[1]),
    THREE.MathUtils.degToRad(transform.rotation[2]),
  );
  group.scale.fromArray(transform.scale);
  group.updateMatrixWorld(true);
}

/** Three stores affine matrices column-major; share files serialize rows. */
function matrixRows(matrix: THREE.Matrix4, rows: number): number[][] {
  const values = matrix.elements;
  return Array.from({ length: rows }, (_, row) =>
    Array.from({ length: rows }, (_, column) => values[column * 4 + row]));
}

function copyLayerStyle(style: LayerStyle): LayerStyle {
  return {
    colorMode: style.colorMode,
    colorMap: style.colorMap,
    intensityEqualize: style.intensityEqualize,
    pointSize: style.pointSize,
    fixedColor: style.fixedColor.clone(),
    noiseColor: style.noiseColor.clone(),
    highlightNoise: style.highlightNoise,
    opacity: style.opacity,
    visible: style.visible,
  };
}

function normalizeLayerStyle(
  style: LayerStyle,
  message: DecodedCloudMessage,
): LayerStyle {
  const normalized = copyLayerStyle(style);
  if ((normalized.colorMode === "rgb" && !message.hasColor) ||
      (normalized.colorMode === "intensity" && !message.hasIntensity)) {
    normalized.colorMode = message.defaultColorMode;
  }
  normalized.highlightNoise &&= message.hasNoise;
  return normalized;
}

/**
 * Validate both user fields and the final GPU matrix. JavaScript accepts
 * finite values far beyond Float32/WebGL range; accepting them would make a
 * layer disappear or poison fit/ROI with infinities after matrix upload.
 */
function isRenderableLayerTransform(
  transform: LayerTransform,
  bounds: CloudBounds | null,
): boolean {
  const maximumCoordinate = maximumFloat32 / 8;
  const maximumRotationDegrees = 360_000;
  return transform.position.length === 3 && transform.rotation.length === 3 &&
    transform.scale.length === 3 && transform.position.every(Number.isFinite) &&
    transform.position.every((value) => Math.abs(value) <= maximumCoordinate) &&
    transform.rotation.every((value) => Number.isFinite(value) &&
      Math.abs(value) <= maximumRotationDegrees) &&
    transform.scale.every((value) => Number.isFinite(value) && value > 1e-6 &&
      value <= 1e6) && isRenderableTransformMatrix(transform, bounds,
      maximumCoordinate);
}

function isRenderableTransformMatrix(
  transform: LayerTransform,
  bounds: CloudBounds | null,
  maximumCoordinate: number,
): boolean {
  const matrix = new THREE.Matrix4().compose(
    new THREE.Vector3(...transform.position),
    new THREE.Quaternion().setFromEuler(new THREE.Euler(
      THREE.MathUtils.degToRad(transform.rotation[0]),
      THREE.MathUtils.degToRad(transform.rotation[1]),
      THREE.MathUtils.degToRad(transform.rotation[2]),
    )),
    new THREE.Vector3(...transform.scale),
  );
  if (!matrix.elements.every((value) => Number.isFinite(value) &&
      Math.abs(value) <= maximumCoordinate)) return false;
  if (!bounds || !bounds.min.every(Number.isFinite) ||
      !bounds.max.every(Number.isFinite)) return bounds === null;
  const world = new THREE.Vector3();
  for (const x of [bounds.min[0], bounds.max[0]]) {
    for (const y of [bounds.min[1], bounds.max[1]]) {
      for (const z of [bounds.min[2], bounds.max[2]]) {
        world.set(x, y, z).applyMatrix4(matrix);
        if (!Number.isFinite(world.x + world.y + world.z) ||
            Math.abs(world.x) > maximumCoordinate ||
            Math.abs(world.y) > maximumCoordinate ||
            Math.abs(world.z) > maximumCoordinate) return false;
      }
    }
  }
  return true;
}

function pointStatistics(positions: Float32Array): PointStatistics {
  const centroid = new THREE.Vector3();
  let finitePointCount = 0;
  for (let offset = 0; offset + 2 < positions.length; offset += 3) {
    const x = positions[offset];
    const y = positions[offset + 1];
    const z = positions[offset + 2];
    if (!Number.isFinite(x + y + z)) continue;
    centroid.x += x;
    centroid.y += y;
    centroid.z += z;
    ++finitePointCount;
  }
  if (finitePointCount > 0) centroid.multiplyScalar(1 / finitePointCount);
  return { centroid, finitePointCount };
}

function isValidBookmark(bookmark: CameraBookmark): boolean {
  return bookmark.position.length === 3 && bookmark.target.length === 3 &&
    bookmark.up.length === 3 && bookmark.position.every(Number.isFinite) &&
    bookmark.target.every(Number.isFinite) && bookmark.up.every(Number.isFinite) &&
    Number.isFinite(bookmark.fov) && bookmark.fov > 0 && bookmark.fov < 180;
}

function safePlyFilename(filename: string): string {
  const stem = filename.replace(/[\\/:*?"<>|\u0000-\u001f]/gu, "_")
    .replace(/\.+$/u, "").trim() || "point-cloud-roi";
  return stem.toLowerCase().endsWith(".ply") ? stem : `${stem}.ply`;
}

function percentile(values: number[], fraction: number): number {
  values.sort((left, right) => left - right);
  return values[Math.min(values.length - 1,
    Math.max(0, Math.floor((values.length - 1) * fraction)))];
}

function finiteRange(values: Float32Array): [number, number] {
  let minimum = Number.POSITIVE_INFINITY;
  let maximum = Number.NEGATIVE_INFINITY;
  for (const value of values) {
    if (!Number.isFinite(value)) continue;
    minimum = Math.min(minimum, value);
    maximum = Math.max(maximum, value);
  }
  return Number.isFinite(minimum) ? [minimum, maximum] : [0, 1];
}

function niceStep(target: number): number {
  if (!Number.isFinite(target) || target <= 0) return 1;
  const magnitude = 10 ** Math.floor(Math.log10(target));
  const normalized = target / magnitude;
  const factor = normalized <= 1 ? 1 : normalized <= 2 ? 2 : normalized <= 5 ? 5 : 10;
  return factor * magnitude;
}

function materialList(
  material: THREE.Material | THREE.Material[],
): THREE.Material[] {
  return Array.isArray(material) ? material : [material];
}

function disposeMaterial(material: THREE.Material | THREE.Material[]): void {
  for (const item of materialList(material)) item.dispose();
}

function colorLuminance(color: THREE.Color): number {
  return color.r * 0.2126 + color.g * 0.7152 + color.b * 0.0722;
}

function contrastRatio(left: number, right: number): number {
  return (Math.max(left, right) + 0.05) / (Math.min(left, right) + 0.05);
}

function readThemeBackground(): string {
  const value = getComputedStyle(document.body)
    .getPropertyValue("--vscode-editor-background")
    .trim();
  return value || "#1e1e1e";
}

function rendererGpuBudget(): number {
  // WebGL exposes no portable VRAM query. deviceMemory is only a coarse hint,
  // so retain the documented 512 MiB fallback and let per-layer LOD avoid
  // speculative full-buffer uploads.
  const deviceMemory = (navigator as Navigator & { deviceMemory?: number })
    .deviceMemory;
  if (!Number.isFinite(deviceMemory) || !deviceMemory || deviceMemory <= 0)
    return 512 * mebibyte;
  return Math.min(
    1024 * mebibyte,
    Math.max(128 * mebibyte, Math.floor(deviceMemory * 0.25 * 1024 * mebibyte)),
  );
}

function rendererCpuBudget(): number {
  // Full CPU snapshots are retained for world-space ROI and export. Apply the
  // same conservative, cross-platform RAM estimate before a layer joins the
  // retained scene, rather than discovering pressure after several decodes.
  return rendererGpuBudget();
}

function estimatedCpuBytes(message: DecodedCloudMessage): number {
  const buffers = new Set<ArrayBufferLike>();
  for (const view of [
    message.positions,
    message.colors,
    message.intensities,
    message.noises,
    message.pointOrder,
    message.chunkRanges,
    message.lodIndices,
  ]) {
    buffers.add(view.buffer);
  }
  // Equalized intensity and full-quality dynamic index are retained copies.
  return [...buffers].reduce((total, buffer) => total + buffer.byteLength, 0) +
    message.intensities.byteLength + message.pointOrder.byteLength;
}

function packedGpuBytesPerPoint(message: DecodedCloudMessage): number {
  // position + optional RGB + intensity + equalized intensity + noise
  return 12 + (message.hasColor ? 3 : 0) + 4 + 4 + 1;
}

function estimatedFullGpuBytes(message: DecodedCloudMessage): number {
  return packedGpuBytesPerPoint(message) * message.pointOrder.length +
    message.pointOrder.byteLength;
}

function estimatedPackedGpuBytes(
  message: DecodedCloudMessage,
  pointCount: number,
): number {
  return packedGpuBytesPerPoint(message) * pointCount;
}

/** Allocate a deterministic, uniform global sampling budget across layers. */
function allocateGlobalSampleBudget(
  sizes: readonly number[],
  maximum: number,
): number[] {
  const normalized = sizes.map((size) =>
    Number.isSafeInteger(size) && size > 0 ? size : 0,
  );
  const total = normalized.reduce((sum, size) => sum + size, 0);
  if (total <= maximum) return normalized;
  const quotas = normalized.map((size, index) => {
    const exact = size * maximum / total;
    return { index, count: Math.floor(exact), remainder: exact % 1 };
  });
  let remaining = maximum - quotas.reduce((sum, quota) => sum + quota.count, 0);
  quotas.sort((left, right) => right.remainder - left.remainder ||
    left.index - right.index);
  for (const quota of quotas) {
    if (remaining === 0) break;
    if (quota.count >= normalized[quota.index]) continue;
    ++quota.count;
    --remaining;
  }
  quotas.sort((left, right) => left.index - right.index);
  return quotas.map((quota) => quota.count);
}

function uniformlySampleIndices(
  source: Uint32Array,
  maximum: number,
): Uint32Array {
  if (source.length === 0) return new Uint32Array();
  const count = Math.max(1, Math.min(source.length, Math.floor(maximum)));
  if (count === source.length) return source;
  const result = new Uint32Array(count);
  const stride = source.length / count;
  for (let index = 0; index < count; ++index) {
    result[index] = source[Math.min(source.length - 1, Math.floor(index * stride))];
  }
  return result;
}

function fullGeometry(
  message: DecodedCloudMessage,
  equalizedIntensities: Float32Array,
): THREE.BufferGeometry {
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute(
    "position",
    new THREE.BufferAttribute(message.positions, 3),
  );
  if (message.hasColor) {
    geometry.setAttribute(
      "color",
      new THREE.BufferAttribute(message.colors, 3, true),
    );
  }
  geometry.setAttribute(
    "intensity",
    new THREE.BufferAttribute(message.intensities, 1),
  );
  geometry.setAttribute(
    "equalizedIntensity",
    new THREE.BufferAttribute(equalizedIntensities, 1),
  );
  geometry.setAttribute(
    "noise",
    new THREE.BufferAttribute(
      message.hasNoise ? message.noises : new Uint8Array(message.pointOrder.length),
      1,
    ),
  );
  // Full layers retain decoded pointOrder for ROI/export. A separate stable
  // dynamic index lets crop edits update in place instead of replacing a
  // BufferAttribute whose old WebGL buffer would survive until geometry dispose.
  const index = new THREE.BufferAttribute(new Uint32Array(message.pointOrder), 1);
  index.setUsage(THREE.DynamicDrawUsage);
  geometry.setIndex(index);
  geometry.setDrawRange(0, message.pointOrder.length);
  return geometry;
}

function gatherGeometry(
  message: DecodedCloudMessage,
  indices: ArrayLike<number>,
  equalizedIntensities: Float32Array,
): THREE.BufferGeometry {
  const positions = new Float32Array(indices.length * 3);
  const colors = message.hasColor
    ? new Uint8Array(indices.length * 3)
    : undefined;
  const intensities = new Float32Array(indices.length);
  const equalized = new Float32Array(indices.length);
  const noises = new Uint8Array(indices.length);
  for (let target = 0; target < indices.length; ++target) {
    const source = indices[target];
    positions.set(
      message.positions.subarray(source * 3, source * 3 + 3),
      target * 3,
    );
    colors?.set(
      message.colors.subarray(source * 3, source * 3 + 3),
      target * 3,
    );
    intensities[target] = message.intensities[source];
    equalized[target] = equalizedIntensities[source];
    if (message.hasNoise) noises[target] = message.noises[source];
  }
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute("position", new THREE.BufferAttribute(positions, 3));
  if (colors) {
    geometry.setAttribute("color", new THREE.BufferAttribute(colors, 3, true));
  }
  geometry.setAttribute(
    "intensity",
    new THREE.BufferAttribute(intensities, 1),
  );
  geometry.setAttribute(
    "equalizedIntensity",
    new THREE.BufferAttribute(equalized, 1),
  );
  geometry.setAttribute("noise", new THREE.BufferAttribute(noises, 1));
  return geometry;
}

function equalizeIntensities(values: Float32Array): Float32Array {
  const result = new Float32Array(values.length);
  let minimum = Number.POSITIVE_INFINITY;
  let maximum = Number.NEGATIVE_INFINITY;
  let finiteCount = 0;
  for (const value of values) {
    if (!Number.isFinite(value)) continue;
    minimum = Math.min(minimum, value);
    maximum = Math.max(maximum, value);
    ++finiteCount;
  }
  if (finiteCount < 2) return result;
  if (minimum === maximum) return result;
  const bins = new Uint32Array(256);
  for (const value of values) {
    if (!Number.isFinite(value)) continue;
    const bin = Math.min(255, Math.max(0, Math.floor(
      (value - minimum) * 255 / (maximum - minimum),
    )));
    ++bins[bin];
  }
  let cumulative = 0;
  const cdf = new Float32Array(256);
  for (let index = 0; index < bins.length; ++index) {
    cumulative += bins[index];
    cdf[index] = cumulative / finiteCount;
  }
  for (let index = 0; index < values.length; ++index) {
    const value = values[index];
    if (!Number.isFinite(value)) continue;
    const bin = Math.min(255, Math.max(0, Math.floor(
      (value - minimum) * 255 / (maximum - minimum),
    )));
    result[index] = cdf[bin];
  }
  return result;
}
