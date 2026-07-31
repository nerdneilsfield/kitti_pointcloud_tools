import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import type { DecodedCloudMessage } from "../src/protocol";

export type ColorMode = "rgb" | "intensity" | "height" | "fixed";
export type StandardView = "fit" | "top" | "front" | "left" | "right" | "iso";

const colorModeValue: Record<ColorMode, number> = {
  rgb: 0,
  intensity: 1,
  height: 2,
  fixed: 3,
};
const maximumFloat32 = 3.4028234663852886e38;

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
  private cloud?: THREE.Points<THREE.BufferGeometry, THREE.ShaderMaterial>;
  private lodCloud?: THREE.Points<THREE.BufferGeometry, THREE.ShaderMaterial>;
  private readonly trajectories: THREE.Line[] = [];
  private center = new THREE.Vector3();
  private referenceMinimum = new THREE.Vector3(-5, -5, -5);
  private referenceMaximum = new THREE.Vector3(5, 5, 5);
  private radius = 1;
  private frame = 0;
  private colorMode: ColorMode = "height";
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

  constructor(private readonly container: HTMLElement) {
    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
    this.container.append(this.renderer.domElement);
    this.scene.background = new THREE.Color(readThemeBackground());
    this.axes.add(...this.axisArrows);
    this.updateReferenceColors();
    this.axes.visible = false;
    this.scene.add(this.axes);
    this.camera.up.set(0, 0, 1);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.08;
    this.controls.mouseButtons.LEFT = THREE.MOUSE.ROTATE;
    this.controls.mouseButtons.MIDDLE = THREE.MOUSE.DOLLY;
    this.controls.mouseButtons.RIGHT = THREE.MOUSE.PAN;
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
    this.render = this.render.bind(this);
    this.frame = requestAnimationFrame(this.render);
  }

  show(message: DecodedCloudMessage): ColorMode {
    const firstCloud = this.cloud === undefined;
    this.clearCloud();
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
      "noise",
      new THREE.BufferAttribute(
        message.hasNoise ? message.noises : new Uint8Array(message.pointCount),
        1,
      ),
    );
    geometry.setIndex(new THREE.BufferAttribute(message.pointOrder, 1));
    for (let index = 0; index < message.chunkRanges.length; index += 2) {
      geometry.addGroup(
        message.chunkRanges[index],
        message.chunkRanges[index + 1],
        0,
      );
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
        pointSize: { value: this.pointSize },
        intensityRange: { value: new THREE.Vector2(...intensityRange) },
        heightRange: { value: new THREE.Vector2(...heightRange) },
        fixedColor: { value: this.fixedColor },
        noiseColor: { value: this.noiseColor },
        highlightNoise: { value: this.highlightNoise && message.hasNoise },
      },
      vertexShader: `
        attribute float intensity;
        attribute float noise;
        uniform int colorMode;
        uniform float pointSize;
        uniform vec2 intensityRange;
        uniform vec2 heightRange;
        uniform vec3 fixedColor;
        uniform vec3 noiseColor;
        uniform bool highlightNoise;
        varying vec3 pointColor;

        vec3 gradient(float value) {
          float t = clamp(value, 0.0, 1.0);
          return clamp(vec3(
            1.5 - abs(4.0 * t - 3.0),
            1.5 - abs(4.0 * t - 2.0),
            1.5 - abs(4.0 * t - 1.0)
          ), 0.0, 1.0);
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
            pointColor = gradient((intensity - intensityRange.x) / span);
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
      const lodGeometry = gatherGeometry(message, message.lodIndices);
      this.lodCloud = new THREE.Points(lodGeometry, material.clone());
      this.lodCloud.visible = false;
      this.scene.add(this.lodCloud);
    }
    this.updatePhysicalPointSize();
    this.updateBounds(message);
    if (firstCloud) this.setView("iso");
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
  }

  setPointSize(size: number): void {
    this.pointSize = Math.max(1, Math.min(size, 32));
    this.updatePhysicalPointSize();
  }

  setFixedColor(color: THREE.ColorRepresentation): void {
    this.fixedColor = new THREE.Color(color);
    this.updateCloudUniform("fixedColor", this.fixedColor);
  }

  setNoiseColor(color: THREE.ColorRepresentation): void {
    this.noiseColor = new THREE.Color(color);
    this.updateCloudUniform("noiseColor", this.noiseColor);
  }

  setNoiseHighlight(visible: boolean): void {
    this.highlightNoise = visible;
    this.updateCloudUniform("highlightNoise", visible);
  }

  setAxesVisible(visible: boolean): void {
    this.axesVisible = visible;
    this.axes.visible = visible;
    this.container.dataset.axesVisible = String(visible);
  }

  setGridVisible(visible: boolean): void {
    this.gridVisible = visible;
    for (const grid of this.grids) grid.visible = visible;
    this.container.dataset.gridVisible = String(visible);
  }

  getGridSpacing(): number {
    return this.gridSpacing;
  }

  setBackground(color: THREE.ColorRepresentation): void {
    this.customBackground = true;
    this.scene.background = new THREE.Color(color);
    this.updateReferenceHelpers(this.referenceMinimum, this.referenceMaximum);
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
    return `#${color.getHexString()}`;
  }

  setView(view: StandardView): void {
    const verticalHalfFov = THREE.MathUtils.degToRad(this.camera.fov * 0.5);
    const horizontalHalfFov = Math.atan(
      Math.tan(verticalHalfFov) * this.camera.aspect,
    );
    const limitingHalfFov = Math.min(verticalHalfFov, horizontalHalfFov);
    const distance = this.radius / Math.sin(limitingHalfFov) * 1.1;
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
    this.camera.up.set(0, 0, 1);
    if (Math.abs(direction.z) > 0.999) {
      this.camera.up.set(0, 1, 0);
    }
    this.camera.position.copy(this.center).addScaledVector(direction, distance);
    this.controls.target.copy(this.center);
    this.camera.lookAt(this.center);
    this.camera.updateProjectionMatrix();
    this.controls.update();
  }

  showTrajectories(
    paths: Array<Array<[number, number, number]>>,
    currentPose?: number[],
  ): void {
    this.clearTrajectories();
    const inversePose = currentPose
      ? new THREE.Matrix4().fromArray(currentPose).invert()
      : new THREE.Matrix4();
    const palette = [0xffcc33, 0x33ccff];
    paths.forEach((path, index) => {
      if (path.length < 2) return;
      const geometry = new THREE.BufferGeometry().setFromPoints(
        path.map(([x, y, z]) =>
          new THREE.Vector3(x, y, z).applyMatrix4(inversePose)
        ),
      );
      const line = new THREE.Line(
        geometry,
        new THREE.LineBasicMaterial({ color: palette[index % palette.length] }),
      );
      this.trajectories.push(line);
      this.scene.add(line);
    });
  }

  dispose(): void {
    cancelAnimationFrame(this.frame);
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
    this.controls.dispose();
    this.clearCloud();
    this.clearTrajectories();
    this.clearGrid();
    this.scene.remove(this.axes);
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
    this.center.copy(min).add(max).multiplyScalar(0.5);
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
  }

  private updateCloudUniform(name: string, value: unknown): void {
    for (const cloud of [this.cloud, this.lodCloud]) {
      const uniform = cloud?.material.uniforms[name];
      if (uniform) uniform.value = value;
    }
  }

  private clearTrajectories(): void {
    for (const line of this.trajectories) {
      this.scene.remove(line);
      line.geometry.dispose();
      (line.material as THREE.Material).dispose();
    }
    this.trajectories.length = 0;
  }

  private resize(): void {
    const width = Math.max(this.container.clientWidth, 1);
    const height = Math.max(this.container.clientHeight, 1);
    this.renderer.setSize(width, height, false);
    this.updatePhysicalPointSize();
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
  }

  private render(): void {
    this.controls.update();
    if (this.cloud && this.lodCloud) {
      const useLod =
        this.camera.position.distanceTo(this.controls.target) >
        this.radius * 2.5;
      this.cloud.visible = !useLod;
      this.lodCloud.visible = useLod;
    }
    this.renderer.render(this.scene, this.camera);
    this.frame = requestAnimationFrame(this.render);
  }

  private updatePhysicalPointSize(): void {
    if (this.cloud) {
      this.cloud.material.uniforms.pointSize.value =
        this.pointSize * this.renderer.getPixelRatio();
    }
    if (this.lodCloud) {
      this.lodCloud.material.uniforms.pointSize.value =
        this.pointSize * this.renderer.getPixelRatio();
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
): THREE.BufferGeometry {
  const positions = new Float32Array(indices.length * 3);
  const colors = message.hasColor
    ? new Uint8Array(indices.length * 3)
    : undefined;
  const intensities = new Float32Array(indices.length);
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
  geometry.setAttribute("noise", new THREE.BufferAttribute(noises, 1));
  return geometry;
}
