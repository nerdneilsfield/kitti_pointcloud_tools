import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import type { DecodedCloudMessage } from "../src/protocol";

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

/** A point picked from the currently displayed single cloud. */
export interface PointPick {
  index: number;
  point: [number, number, number];
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

  show(message: DecodedCloudMessage): ColorMode {
    const firstCloud = this.cloud === undefined;
    this.clearCloud();
    this.clearRoi();
    this.clearMeasurement();
    this.message = message;
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
    const equalizedIntensities = equalizeIntensities(message.intensities);
    this.equalizedIntensities = equalizedIntensities;
    geometry.setAttribute(
      "equalizedIntensity",
      new THREE.BufferAttribute(equalizedIntensities, 1),
    );
    geometry.setAttribute(
      "noise",
      new THREE.BufferAttribute(
        message.hasNoise ? message.noises : new Uint8Array(message.pointCount),
        1,
      ),
    );
    geometry.setIndex(new THREE.BufferAttribute(message.pointOrder, 1));

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
        void main() {
          vec2 centered = gl_PointCoord - vec2(0.5);
          if (dot(centered, centered) > 0.25) discard;
          gl_FragColor = vec4(pointColor, 1.0);
        }
      `,
      vertexColors: message.hasColor,
    });
    this.cloud = new THREE.Points(geometry, material);
    this.scene.add(this.cloud);
    if (message.pointCount > 100_000) {
      const lodGeometry = gatherGeometry(message, message.lodIndices, equalizedIntensities);
      this.lodCloud = new THREE.Points(lodGeometry, material.clone());
      this.lodCloud.visible = false;
      this.scene.add(this.lodCloud);
    }
    this.updatePhysicalPointSize();
    this.updateBounds(message);
    this.setView(firstCloud ? "iso" : "fit");
    this.invalidate();
    return this.colorMode;
  }

  setColorMode(mode: ColorMode): void {
    this.colorMode = mode;
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
    this.updateCloudUniform("colorMap", colorMapValue[colorMap]);
    this.invalidate();
  }

  setIntensityEqualization(equalize: boolean): void {
    this.intensityEqualize = equalize;
    this.updateCloudUniform("intensityEqualize", equalize);
    this.invalidate();
  }

  setPointSize(size: number): void {
    if (!Number.isFinite(size)) return;
    this.pointSize = Math.max(0, Math.min(size, 5));
    this.updatePhysicalPointSize();
    this.invalidate();
  }

  setFixedColor(color: THREE.ColorRepresentation): void {
    this.fixedColor = new THREE.Color(color);
    this.updateCloudUniform("fixedColor", this.fixedColor);
    this.invalidate();
  }

  setNoiseColor(color: THREE.ColorRepresentation): void {
    this.noiseColor = new THREE.Color(color);
    this.updateCloudUniform("noiseColor", this.noiseColor);
    this.invalidate();
  }

  setNoiseHighlight(visible: boolean): void {
    this.highlightNoise = visible;
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
      ? this.camera.position.clone().sub(this.center).normalize()
      : directions[view];
    this.fitCamera(direction.lengthSq() > 0 ? direction : directions.iso);
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
    if (!this.message) return false;
    if (roi && !isValidRoi(roi)) return false;
    this.clearRoi();
    if (!roi) {
      this.cloud?.geometry.setIndex(new THREE.BufferAttribute(
        this.message.pointOrder,
        1,
      ));
      this.rebuildLodGeometry(this.message.lodIndices);
      this.invalidate();
      return true;
    }

    this.roi = copyRoi(roi);
    this.roiIndices = filterClosedRoi(this.message.positions, this.message.pointOrder, roi);
    this.cloud?.geometry.setIndex(new THREE.BufferAttribute(this.roiIndices, 1));
    this.rebuildLodGeometry(filterClosedRoi(
      this.message.positions,
      this.message.lodIndices,
      roi,
    ));
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
    this.invalidate();
    return true;
  }

  getRoi(): RoiBox | undefined {
    return this.roi ? copyRoi(this.roi) : undefined;
  }

  getVisiblePointCount(): number {
    if (!this.message) return 0;
    if (this.roiIndices) return this.roiIndices.length;
    return countFinitePoints(this.message.positions, this.message.pointOrder);
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

  /** Download currently visible finite points in portable ASCII PLY form. */
  downloadVisiblePly(filename: string): number {
    const result = this.createVisiblePly();
    if (result.count === 0) return 0;
    const anchor = document.createElement("a");
    const uri = URL.createObjectURL(result.blob);
    anchor.href = uri;
    anchor.download = safePlyFilename(filename);
    anchor.hidden = true;
    document.body.append(anchor);
    anchor.click();
    anchor.remove();
    window.setTimeout(() => URL.revokeObjectURL(uri), 0);
    return result.count;
  }

  private fitCamera(direction: THREE.Vector3): void {
    const positions = this.framingPositions;
    const pointCount = positions ? Math.floor(positions.length / 3) : 0;
    const stride = Math.max(1, Math.ceil(pointCount / maximumFramingSamples));
    const radii: number[] = [];
    if (positions) {
      for (let index = 0; index < pointCount; index += stride) {
        const offset = index * 3;
        const x = positions[offset];
        const y = positions[offset + 1];
        const z = positions[offset + 2];
        if (!Number.isFinite(x + y + z)) continue;
        radii.push(Math.hypot(x - this.center.x, y - this.center.y, z - this.center.z));
      }
    }
    const robustRadius = radii.length > 0
      ? percentile(radii, framingPercentile)
      : this.radius;
    const distance = Math.max(robustRadius * 2.8, 0.01);
    this.camera.up.set(0, 0, 1);
    if (Math.abs(direction.z) > 0.999) {
      this.camera.up.set(0, 1, 0);
    }
    this.camera.position.copy(this.center).addScaledVector(direction, distance);
    this.controls.target.copy(this.center);
    this.camera.lookAt(this.center);
    this.camera.updateMatrixWorld();
    const right = new THREE.Vector3().setFromMatrixColumn(this.camera.matrixWorld, 0);
    const up = new THREE.Vector3().setFromMatrixColumn(this.camera.matrixWorld, 1);
    const back = new THREE.Vector3().setFromMatrixColumn(this.camera.matrixWorld, 2);
    const slopes: number[] = [];
    if (positions) {
      for (let index = 0; index < pointCount; index += stride) {
        const offset = index * 3;
        const x = positions[offset];
        const y = positions[offset + 1];
        const z = positions[offset + 2];
        if (!Number.isFinite(x + y + z)) continue;
        const pointOffset = new THREE.Vector3(x, y, z).sub(this.center);
        const depth = distance - back.dot(pointOffset);
        if (depth <= 1e-9) continue;
        slopes.push(Math.max(
          Math.abs(right.dot(pointOffset)) / (depth * this.camera.aspect),
          Math.abs(up.dot(pointOffset)) / depth,
        ));
      }
    }
    const slope = slopes.length > 0 ? percentile(slopes, framingPercentile) :
      Math.tan(THREE.MathUtils.degToRad(25));
    this.camera.fov = THREE.MathUtils.clamp(
      THREE.MathUtils.radToDeg(2 * Math.atan(slope / framingFill)),
      minimumAutoFov,
      maximumAutoFov,
    );
    this.camera.updateProjectionMatrix();
    this.controls.update();
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

  private updateBounds(message: DecodedCloudMessage): void {
    this.framingPositions = message.positions;
    if (!message.bounds) {
      this.center.set(0, 0, 0);
      this.radius = 1;
      this.referenceMinimum.set(-5, -5, -5);
      this.referenceMaximum.set(5, 5, 5);
      this.updateReferenceHelpers();
      return;
    }
    const min = new THREE.Vector3(...message.bounds.min);
    const max = new THREE.Vector3(...message.bounds.max);
    this.referenceMinimum.copy(min);
    this.referenceMaximum.copy(max);
    let sumX = 0;
    let sumY = 0;
    let sumZ = 0;
    let finitePoints = 0;
    for (let index = 0; index + 2 < message.positions.length; index += 3) {
      const x = message.positions[index];
      const y = message.positions[index + 1];
      const z = message.positions[index + 2];
      if (!Number.isFinite(x + y + z)) continue;
      sumX += x;
      sumY += y;
      sumZ += z;
      ++finitePoints;
    }
    this.center.set(
      finitePoints > 0 ? sumX / finitePoints : (min.x + max.x) * 0.5,
      finitePoints > 0 ? sumY / finitePoints : (min.y + max.y) * 0.5,
      finitePoints > 0 ? sumZ / finitePoints : (min.z + max.z) * 0.5,
    );
    this.radius = Math.max(min.distanceTo(max) * 0.5, 0.01);
    this.camera.near = Math.max(this.radius / 1000, 0.001);
    this.camera.far = Math.max(this.radius * 100, 100);
    this.camera.updateProjectionMatrix();
    this.controls.minDistance = this.radius * 0.001;
    this.controls.maxDistance = this.radius * 100;
    this.updateReferenceHelpers(min, max);
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
    for (const cloud of [this.cloud, this.lodCloud]) {
      if (!cloud) continue;
      this.scene.remove(cloud);
      cloud.geometry.dispose();
      cloud.material.dispose();
    }
    this.cloud = undefined;
    this.lodCloud = undefined;
    this.message = undefined;
    this.equalizedIntensities = undefined;
    this.roiIndices = undefined;
  }

  private clearRoi(): void {
    this.roi = undefined;
    this.roiIndices = undefined;
    if (!this.roiHelper) return;
    this.scene.remove(this.roiHelper);
    this.roiHelper.geometry.dispose();
    disposeMaterial(this.roiHelper.material);
    this.roiHelper = undefined;
  }

  private rebuildLodGeometry(indices: ArrayLike<number>): void {
    if (!this.lodCloud || !this.message || !this.equalizedIntensities) return;
    const next = gatherGeometry(this.message, indices, this.equalizedIntensities);
    this.lodCloud.geometry.dispose();
    this.lodCloud.geometry = next;
  }

  private createVisiblePly(): { blob: Blob; count: number } {
    const message = this.message;
    if (!message) return { blob: new Blob(), count: 0 };
    const indices = this.roiIndices ?? message.pointOrder;
    let count = 0;
    for (const index of indices) {
      const offset = index * 3;
      if (Number.isFinite(
        message.positions[offset] + message.positions[offset + 1] +
        message.positions[offset + 2],
      )) ++count;
    }
    const properties = ["property float x", "property float y", "property float z"];
    if (message.hasColor) {
      properties.push("property uchar red", "property uchar green", "property uchar blue");
    }
    if (message.hasIntensity) properties.push("property float intensity");
    if (message.hasNoise) properties.push("property uchar noise");
    const chunks: BlobPart[] = [[
      "ply", "format ascii 1.0", `element vertex ${count}`,
      ...properties, "end_header", "",
    ].join("\n")];
    let lines: string[] = [];
    for (const index of indices) {
      const offset = index * 3;
      const x = message.positions[offset];
      const y = message.positions[offset + 1];
      const z = message.positions[offset + 2];
      if (!Number.isFinite(x + y + z)) continue;
      const row = [String(x), String(y), String(z)];
      if (message.hasColor) {
        const color = index * 3;
        row.push(String(message.colors[color]), String(message.colors[color + 1]),
          String(message.colors[color + 2]));
      }
      if (message.hasIntensity) row.push(String(message.intensities[index]));
      if (message.hasNoise) row.push(String(message.noises[index]));
      lines.push(`${row.join(" ")}\n`);
      if (lines.length >= 8_192) {
        chunks.push(lines.join(""));
        lines = [];
      }
    }
    if (lines.length) chunks.push(lines.join(""));
    return { blob: new Blob(chunks, { type: "application/octet-stream" }), count };
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
    if (this.cloud) {
      const useLod =
        this.lodCloud !== undefined &&
        this.camera.position.distanceTo(this.controls.target) >
        this.radius * 2.5;
      const showPoints = this.pointSize > 0;
      this.cloud.visible = showPoints && !useLod;
      if (this.lodCloud) this.lodCloud.visible = showPoints && useLod;
    }
    this.renderer.render(this.scene, this.camera);
  }

  private readonly invalidate = (): void => {
    if (this.frame === 0) this.frame = requestAnimationFrame(this.render);
  };

  private updatePhysicalPointSize(): void {
    if (this.cloud) {
      this.cloud.material.uniforms.pointSize.value = Math.min(
        this.pointSize * this.renderer.getPixelRatio(),
        5,
      );
    }
    if (this.lodCloud) {
      this.lodCloud.material.uniforms.pointSize.value = Math.min(
        this.pointSize * this.renderer.getPixelRatio(),
        5,
      );
    }
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
    const message = this.message;
    if (!message) return undefined;
    const bounds = this.renderer.domElement.getBoundingClientRect();
    if (bounds.width <= 0 || bounds.height <= 0) return undefined;
    const candidates = this.roiIndices ?? message.pointOrder;
    const stride = Math.max(1, Math.ceil(candidates.length / maximumFramingSamples));
    const maximumDistanceSquared = Math.max(12, this.pointSize * 4) ** 2;
    let nearest: PointPick | undefined;
    let nearestDistanceSquared = maximumDistanceSquared;
    let nearestDepth = Number.POSITIVE_INFINITY;
    const world = new THREE.Vector3();
    const projected = new THREE.Vector3();
    this.camera.updateMatrixWorld();
    for (let candidate = 0; candidate < candidates.length; candidate += stride) {
      const index = candidates[candidate];
      const offset = index * 3;
      const x = message.positions[offset];
      const y = message.positions[offset + 1];
      const z = message.positions[offset + 2];
      if (!Number.isFinite(x + y + z)) continue;
      world.set(x, y, z);
      projected.copy(world).project(this.camera);
      if (projected.z < -1 || projected.z > 1) continue;
      const screenX = bounds.left + (projected.x + 1) * bounds.width * 0.5;
      const screenY = bounds.top + (1 - projected.y) * bounds.height * 0.5;
      const distanceSquared = (screenX - clientX) ** 2 + (screenY - clientY) ** 2;
      if (distanceSquared > nearestDistanceSquared) continue;
      const depth = this.camera.position.distanceToSquared(world);
      if (distanceSquared === nearestDistanceSquared && depth >= nearestDepth)
        continue;
      nearestDistanceSquared = distanceSquared;
      nearestDepth = depth;
      nearest = { index, point: [x, y, z] };
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

function filterClosedRoi(
  positions: Float32Array,
  candidates: ArrayLike<number>,
  roi: RoiBox,
): Uint32Array {
  const included: number[] = [];
  for (let candidate = 0; candidate < candidates.length; ++candidate) {
    const index = candidates[candidate];
    const offset = index * 3;
    const x = positions[offset];
    const y = positions[offset + 1];
    const z = positions[offset + 2];
    if (!Number.isFinite(x + y + z)) continue;
    if (x >= roi.min[0] && x <= roi.max[0] &&
        y >= roi.min[1] && y <= roi.max[1] &&
        z >= roi.min[2] && z <= roi.max[2]) {
      included.push(index);
    }
  }
  return Uint32Array.from(included);
}

function countFinitePoints(
  positions: Float32Array,
  candidates: ArrayLike<number>,
): number {
  let count = 0;
  for (let candidate = 0; candidate < candidates.length; ++candidate) {
    const offset = candidates[candidate] * 3;
    if (Number.isFinite(
      positions[offset] + positions[offset + 1] + positions[offset + 2],
    )) ++count;
  }
  return count;
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
