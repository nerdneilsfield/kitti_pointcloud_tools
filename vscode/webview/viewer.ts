import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import type { DecodedCloudMessage } from "../src/protocol";

export type ColorMode = "rgb" | "intensity" | "height";
export type StandardView = "fit" | "top" | "front" | "left" | "right" | "iso";

const colorModeValue: Record<ColorMode, number> = {
  rgb: 0,
  intensity: 1,
  height: 2,
};

export class PointCloudViewer {
  private readonly scene = new THREE.Scene();
  private readonly camera = new THREE.PerspectiveCamera(50, 1, 0.01, 10000);
  private readonly renderer: THREE.WebGLRenderer;
  private readonly controls: OrbitControls;
  private readonly observer: ResizeObserver;
  private readonly themeObserver: MutationObserver;
  private cloud?: THREE.Points<THREE.BufferGeometry, THREE.ShaderMaterial>;
  private lodCloud?: THREE.Points<THREE.BufferGeometry, THREE.ShaderMaterial>;
  private readonly trajectories: THREE.Line[] = [];
  private center = new THREE.Vector3();
  private radius = 1;
  private frame = 0;
  private colorMode: ColorMode = "height";
  private pointSize = 1.5;
  private customBackground = false;
  private rolling = false;
  private rollPointer = -1;
  private previousRollX = 0;

  constructor(private readonly container: HTMLElement) {
    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
    this.container.append(this.renderer.domElement);
    this.scene.background = new THREE.Color(readThemeBackground());
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
    this.colorMode = message.defaultColorMode;
    const material = new THREE.ShaderMaterial({
      uniforms: {
        colorMode: { value: colorModeValue[this.colorMode] },
        pointSize: { value: this.pointSize },
        intensityRange: { value: new THREE.Vector2(...intensityRange) },
        heightRange: { value: new THREE.Vector2(...heightRange) },
      },
      vertexShader: `
        attribute float intensity;
        uniform int colorMode;
        uniform float pointSize;
        uniform vec2 intensityRange;
        uniform vec2 heightRange;
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
          } else {
            float span = max(heightRange.y - heightRange.x, 1e-6);
            pointColor = gradient((position.z - heightRange.x) / span);
          }
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

  setBackground(color: THREE.ColorRepresentation): void {
    this.customBackground = true;
    this.scene.background = new THREE.Color(color);
  }

  useThemeBackground(): string {
    this.customBackground = false;
    return this.syncThemeBackground() ?? "#1e1e1e";
  }

  syncThemeBackground(): string | null {
    if (this.customBackground) return null;
    const color = new THREE.Color(readThemeBackground());
    this.scene.background = color;
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
    this.renderer.dispose();
    this.renderer.forceContextLoss();
    this.renderer.domElement.remove();
  }

  private updateBounds(message: DecodedCloudMessage): void {
    if (!message.bounds) {
      this.center.set(0, 0, 0);
      this.radius = 1;
      return;
    }
    const min = new THREE.Vector3(...message.bounds.min);
    const max = new THREE.Vector3(...message.bounds.max);
    this.center.copy(min).add(max).multiplyScalar(0.5);
    this.radius = Math.max(min.distanceTo(max) * 0.5, 0.01);
    this.camera.near = Math.max(this.radius / 1000, 0.001);
    this.camera.far = Math.max(this.radius * 100, 100);
    this.camera.updateProjectionMatrix();
    this.controls.minDistance = this.radius * 0.001;
    this.controls.maxDistance = this.radius * 100;
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
  return geometry;
}
