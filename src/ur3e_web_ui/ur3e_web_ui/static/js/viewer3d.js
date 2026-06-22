import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import { GLTFLoader } from "three/examples/jsm/loaders/GLTFLoader.js";
import { TransformControls } from "three/examples/jsm/controls/TransformControls.js";
import URDFLoader from "urdf-loader";

const ROS_TO_THREE_Q = new THREE.Quaternion().setFromEuler(new THREE.Euler(-Math.PI / 2, 0, 0, "XYZ"));
const THREE_TO_ROS_Q = ROS_TO_THREE_Q.clone().invert();
const BASE_TO_BASE_LINK_Q = new THREE.Quaternion().setFromEuler(new THREE.Euler(0, 0, Math.PI, "XYZ"));
const BASE_LINK_TO_BASE_Q = BASE_TO_BASE_LINK_Q.clone().invert();

// Velocity gizmo: zero-rotation reference points along base +Z (THREE +Y = up), and
// the throwable speed is clamped so the ball stays in a sane range (m/s).
const VEL_REF_DIR = new THREE.Vector3(0, 1, 0);
const VEL_SPEED_MIN = 0.2;
const VEL_SPEED_MAX = 9.0;

export class Viewer3D {
  constructor(container) {
    this.container = container;
    this.robot = null;
    this.ghost = null;
    this.replayGhost = null;
    this.policyGhost = null;
    this.policyGhostEnabled = true;
    this.toolMesh = null;
    this.cameraFrame = null;
    this.ballGroup = null;
    this.ballMarker = null;
    this.ballPath = null;
    this.ballLaunchFrame = null;
    this.ballLaunchControls = null;
    this.ballLaunchCallback = null;
    this.ballLaunchUpdateMuted = false;
    this.ballLaunchVelocity = null;
    this.ballLaunchPath = null;
    // Mouse control of v0: "move" drags p0 (translate), "aim" rotates the velocity
    // direction (rotate gizmo), "speed" scales its norm (scale gizmo). Only one
    // gizmo is shown at a time. _velDir is a unit vector in THREE space, _velSpeed
    // its magnitude in m/s; together they encode v0 (base) via _velBase().
    this.ballVelocityHandle = null;
    this.ballVelocityControls = null;
    this.ballLaunchMode = "move";
    this._velDir = new THREE.Vector3(0, 1, 0);
    this._velSpeed = 0;
    this._velScaleRefSpeed = 0;
    this._velDragging = false;
    this._flightHorizon = 2.0;  // predicted-arc length = test_ball restart_after_s (s)
    this.ghostMaterialApplied = false;
    this.replayGhostMaterialApplied = false;
    this.policyGhostMaterialApplied = false;
    this.preview = null; // {plan, referencePlan, startMs, onProgress, onDone}
    this.targetFrame = null;
    this.targetCallback = null;
    this.targetUpdateMuted = false;

    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x14171c);

    this.camera = new THREE.PerspectiveCamera(50, 1, 0.01, 20);
    this.camera.position.set(0.7, 0.55, 0.7);

    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setPixelRatio(window.devicePixelRatio);
    container.appendChild(this.renderer.domElement);

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.target.set(0, 0.25, 0);

    this.scene.add(new THREE.HemisphereLight(0xdfe7f5, 0x202830, 1.1));
    const sun = new THREE.DirectionalLight(0xffffff, 1.4);
    sun.position.set(1.5, 2.5, 1.0);
    this.scene.add(sun);

    const grid = new THREE.GridHelper(2, 20, 0x3a4252, 0x262c38);
    this.scene.add(grid);

    this.buildTargetFrame();
    this.buildBall();
    this.buildBallLaunchFrame();

    new ResizeObserver(() => this.resize()).observe(container);
    this.resize();
    this.renderer.setAnimationLoop(() => this.tick());
  }

  resize() {
    const width = this.container.clientWidth || 1;
    const height = this.container.clientHeight || 1;
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(width, height);
  }

  loadRobot(urdfText) {
    const loader = new URDFLoader();
    loader.packages = { ur_description: "/pkg/ur_description" };

    this.robot = loader.parse(urdfText);
    this.robot.rotation.x = -Math.PI / 2; // ROS Z-up -> three.js Y-up
    this.scene.add(this.robot);

    this.ghost = loader.parse(urdfText);
    this.ghost.rotation.x = -Math.PI / 2;
    this.ghost.visible = false;
    this.scene.add(this.ghost);

    this.replayGhost = loader.parse(urdfText);
    this.replayGhost.rotation.x = -Math.PI / 2;
    this.replayGhost.visible = false;
    this.scene.add(this.replayGhost);

    this.policyGhost = loader.parse(urdfText);
    this.policyGhost.rotation.x = -Math.PI / 2;
    this.policyGhost.visible = false;
    this.scene.add(this.policyGhost);
  }

  buildTargetFrame() {
    const material = new THREE.MeshStandardMaterial({
      color: 0xe0a93e,
      emissive: 0x3a2500,
      roughness: 0.45,
    });
    this.targetFrame = new THREE.Group();
    this.targetFrame.position.set(0.25, 0.35, 0.15);
    this.targetFrame.visible = false;
    this.targetFrame.add(new THREE.Mesh(new THREE.SphereGeometry(0.018, 24, 12), material));

    const ringMaterial = new THREE.MeshBasicMaterial({ color: 0xe0a93e, transparent: true, opacity: 0.55 });
    const ring = new THREE.Mesh(new THREE.TorusGeometry(0.07, 0.0025, 8, 48), ringMaterial);
    ring.rotation.x = Math.PI / 2;
    this.targetFrame.add(ring);
    this.addTargetTriad();
    this.addTargetRotationRings();
    this.scene.add(this.targetFrame);

    this.targetControls = new TransformControls(this.camera, this.renderer.domElement);
    this.targetControls.setSize(0.75);
    this.targetControls.setMode("translate");
    this.targetControls.setSpace("local");
    this.targetControls.attach(this.targetFrame);
    this.targetControls.visible = false;
    this.targetControls.addEventListener("dragging-changed", (event) => {
      this.controls.enabled = !event.value;
    });
    this.targetControls.addEventListener("objectChange", () => {
      if (!this.targetCallback || this.targetUpdateMuted) return;
      this.targetCallback(this.getTargetPose());
    });
    this.scene.add(this.targetControls);
  }

  addTargetTriad() {
    const origin = new THREE.Vector3(0, 0, 0);
    const axes = [
      { label: "X", direction: new THREE.Vector3(1, 0, 0), color: 0xff4b4b, labelPosition: [0.28, 0, 0] },
      { label: "Y", direction: new THREE.Vector3(0, 1, 0), color: 0x41c97f, labelPosition: [0, 0.28, 0] },
      { label: "Z", direction: new THREE.Vector3(0, 0, 1), color: 0x4fa3ff, labelPosition: [0, 0, 0.28] },
    ];
    for (const axis of axes) {
      const arrow = new THREE.ArrowHelper(axis.direction, origin, 0.24, axis.color, 0.06, 0.035);
      this.targetFrame.add(arrow);
      const label = this.makeAxisLabel(axis.label, axis.color);
      label.position.set(...axis.labelPosition);
      this.targetFrame.add(label);
    }
  }

  addTargetRotationRings() {
    const rings = [
      { color: 0xff4b4b, rotation: [0, Math.PI / 2, 0] },
      { color: 0x41c97f, rotation: [Math.PI / 2, 0, 0] },
      { color: 0x4fa3ff, rotation: [0, 0, 0] },
    ];
    for (const ringSpec of rings) {
      const material = new THREE.MeshBasicMaterial({
        color: ringSpec.color,
        transparent: true,
        opacity: 0.35,
        depthTest: false,
      });
      const ring = new THREE.Mesh(new THREE.TorusGeometry(0.18, 0.003, 8, 72), material);
      ring.rotation.set(...ringSpec.rotation);
      this.targetFrame.add(ring);
    }
  }

  makeAxisLabel(text, color) {
    const canvas = document.createElement("canvas");
    canvas.width = 96;
    canvas.height = 96;
    const ctx = canvas.getContext("2d");
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    ctx.fillStyle = "#14171c";
    ctx.beginPath();
    ctx.arc(48, 48, 26, 0, Math.PI * 2);
    ctx.fill();
    ctx.strokeStyle = `#${color.toString(16).padStart(6, "0")}`;
    ctx.lineWidth = 6;
    ctx.stroke();
    ctx.fillStyle = "#ffffff";
    ctx.font = "bold 38px system-ui, sans-serif";
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.fillText(text, 48, 50);

    const texture = new THREE.CanvasTexture(canvas);
    const material = new THREE.SpriteMaterial({ map: texture, transparent: true, depthTest: false });
    const sprite = new THREE.Sprite(material);
    sprite.scale.set(0.07, 0.07, 0.07);
    return sprite;
  }

  setLiveJoints(names, positions) {
    if (!this.robot) return;
    for (let i = 0; i < names.length; i++) {
      const joint = this.robot.joints[names[i]];
      if (joint) joint.setJointValue(positions[i]);
    }
  }

  async loadToolMesh(config, linkName = "tool0") {
    if (!this.robot) throw new Error("robot model not loaded yet");
    const link = this.robot.links && this.robot.links[linkName];
    if (!link) throw new Error(`${linkName} link not found in the URDF`);
    if (this.toolMesh) {
      this.toolMesh.removeFromParent();
      this.toolMesh = null;
    }
    const gltf = await new GLTFLoader().loadAsync(config.glb_url);
    const mesh = gltf.scene;
    const material = new THREE.MeshBasicMaterial({
      color: 0xdce7ee,
      transparent: false,
      opacity: 1.0,
      depthWrite: true,
      side: THREE.DoubleSide,
      toneMapped: false,
    });
    mesh.traverse((child) => {
      if (child.isMesh) {
        child.material = material;
        child.castShadow = false;
        child.receiveShadow = false;
      }
    });
    // Children of a URDF link live in ROS axes (the Y-up fix is applied at
    // the robot root), so the mount transform is plain ROS xyz/rpy.
    const scale = config.scale || 1.0;
    mesh.scale.set(scale, scale, scale);
    mesh.position.set(...(config.xyz_m || [0, 0, 0]));
    const [roll, pitch, yaw] = config.rpy_rad || [0, 0, 0];
    mesh.quaternion.setFromEuler(new THREE.Euler(roll, pitch, yaw, "ZYX"));
    mesh.visible = false;
    link.add(mesh);
    this.toolMesh = mesh;
    return mesh;
  }

  setToolMeshVisible(visible) {
    if (this.toolMesh) this.toolMesh.visible = visible;
  }

  setCameraFrame(pose) {
    // pose: {xyz: [m], quat_xyzw: [..]} of the optical frame in the UR
    // `base` frame (the hand-eye result). Same base -> base_link -> three.js
    // conversion as the TCP target frame.
    if (!this.cameraFrame) this.buildCameraFrame();
    const [x, y, z] = pose.xyz;
    this.cameraFrame.position.set(-x, z, y);
    const [qx, qy, qz, qw] = pose.quat_xyzw;
    const baseQuat = new THREE.Quaternion(qx, qy, qz, qw);
    const baseLinkQuat = BASE_TO_BASE_LINK_Q.clone().multiply(baseQuat);
    this.cameraFrame.quaternion.copy(ROS_TO_THREE_Q.clone().multiply(baseLinkQuat));
    this.cameraFrame.visible = true;
  }

  setCameraFrameVisible(visible) {
    if (this.cameraFrame) this.cameraFrame.visible = visible;
  }

  buildCameraFrame() {
    this.cameraFrame = new THREE.Group();
    this.cameraFrame.visible = false;

    const axes = [
      { direction: new THREE.Vector3(1, 0, 0), color: 0xff4b4b },
      { direction: new THREE.Vector3(0, 1, 0), color: 0x41c97f },
      { direction: new THREE.Vector3(0, 0, 1), color: 0x4fa3ff },
    ];
    for (const axis of axes) {
      this.cameraFrame.add(new THREE.ArrowHelper(axis.direction, new THREE.Vector3(), 0.15, axis.color, 0.04, 0.02));
    }

    // Wireframe frustum along +Z (the optical axis looks at the robot).
    const depth = 0.18;
    const halfWidth = 0.12;
    const halfHeight = 0.09;
    const corners = [
      [-halfWidth, -halfHeight, depth],
      [halfWidth, -halfHeight, depth],
      [halfWidth, halfHeight, depth],
      [-halfWidth, halfHeight, depth],
    ];
    const points = [];
    for (let i = 0; i < 4; i++) {
      points.push(new THREE.Vector3(0, 0, 0), new THREE.Vector3(...corners[i]));
      points.push(new THREE.Vector3(...corners[i]), new THREE.Vector3(...corners[(i + 1) % 4]));
    }
    const geometry = new THREE.BufferGeometry().setFromPoints(points);
    const material = new THREE.LineBasicMaterial({ color: 0xf0d060, transparent: true, opacity: 0.8 });
    this.cameraFrame.add(new THREE.LineSegments(geometry, material));

    const label = this.makeAxisLabel("C", 0xf0d060);
    label.position.set(0, 0.06, 0);
    this.cameraFrame.add(label);
    this.scene.add(this.cameraFrame);
  }

  // base (x,y,z) -> three.js (-x, z, y): the base->base_link (pi about Z) then
  // ROS Z-up -> three.js Y-up convention used by the TCP/camera frames above.
  baseToThree(x, y, z) {
    return new THREE.Vector3(-x, z, y);
  }

  baseVectorToThree(x, y, z) {
    return new THREE.Vector3(-x, z, y);
  }

  buildBall() {
    this.ballGroup = new THREE.Group();
    this.ballGroup.visible = false;

    const ballMaterial = new THREE.MeshStandardMaterial({
      color: 0xff5d5d,
      emissive: 0x4a0000,
      roughness: 0.4,
    });
    this.ballMarker = new THREE.Mesh(new THREE.SphereGeometry(0.03, 24, 16), ballMaterial);
    this.ballGroup.add(this.ballMarker);

    const pathMaterial = new THREE.LineBasicMaterial({ color: 0xff9d5d, transparent: true, opacity: 0.85 });
    this.ballPath = new THREE.Line(new THREE.BufferGeometry(), pathMaterial);
    this.ballGroup.add(this.ballPath);

    this.scene.add(this.ballGroup);
  }

  buildBallLaunchFrame() {
    this.ballLaunchFrame = new THREE.Group();
    this.ballLaunchFrame.visible = false;

    const coreMaterial = new THREE.MeshStandardMaterial({
      color: 0x59d7ff,
      emissive: 0x063047,
      roughness: 0.38,
    });
    const core = new THREE.Mesh(new THREE.SphereGeometry(0.026, 24, 16), coreMaterial);
    this.ballLaunchFrame.add(core);

    const ringMaterial = new THREE.MeshBasicMaterial({ color: 0x59d7ff, transparent: true, opacity: 0.65 });
    const ring = new THREE.Mesh(new THREE.TorusGeometry(0.085, 0.003, 8, 48), ringMaterial);
    ring.rotation.x = Math.PI / 2;
    this.ballLaunchFrame.add(ring);

    const axes = [
      { label: "X", direction: new THREE.Vector3(1, 0, 0), color: 0xff4b4b, labelPosition: [0.18, 0, 0] },
      { label: "Y", direction: new THREE.Vector3(0, 1, 0), color: 0x41c97f, labelPosition: [0, 0.18, 0] },
      { label: "Z", direction: new THREE.Vector3(0, 0, 1), color: 0x4fa3ff, labelPosition: [0, 0, 0.18] },
    ];
    for (const axis of axes) {
      this.ballLaunchFrame.add(new THREE.ArrowHelper(axis.direction, new THREE.Vector3(), 0.16, axis.color, 0.04, 0.022));
      const label = this.makeAxisLabel(axis.label, axis.color);
      label.position.set(...axis.labelPosition);
      label.scale.set(0.052, 0.052, 0.052);
      this.ballLaunchFrame.add(label);
    }

    this.ballLaunchVelocity = new THREE.ArrowHelper(
      new THREE.Vector3(0, 1, 0),
      new THREE.Vector3(),
      0.25,
      0xffd166,
      0.07,
      0.035,
    );
    this.ballLaunchFrame.add(this.ballLaunchVelocity);

    const pathMaterial = new THREE.LineBasicMaterial({ color: 0xffd166, transparent: true, opacity: 0.55 });
    this.ballLaunchPath = new THREE.Line(new THREE.BufferGeometry(), pathMaterial);
    this.scene.add(this.ballLaunchPath);
    this.scene.add(this.ballLaunchFrame);

    this.ballLaunchControls = new TransformControls(this.camera, this.renderer.domElement);
    this.ballLaunchControls.setSize(0.65);
    this.ballLaunchControls.setMode("translate");
    this.ballLaunchControls.setSpace("world");
    this.ballLaunchControls.attach(this.ballLaunchFrame);
    this.ballLaunchControls.visible = false;
    this.ballLaunchControls.addEventListener("dragging-changed", (event) => {
      this.controls.enabled = !event.value;
    });
    this.ballLaunchControls.addEventListener("objectChange", () => {
      if (!this.ballLaunchCallback || this.ballLaunchUpdateMuted) return;
      this.ballLaunchCallback(this.getBallLaunchConfig());
    });
    this.scene.add(this.ballLaunchControls);

    // Velocity gizmo: rotate => aim v0, scale => set |v0|. Attached to an invisible
    // handle kept at the launch-frame origin; the yellow arrow stays the visual.
    this.ballVelocityHandle = new THREE.Object3D();
    this.scene.add(this.ballVelocityHandle);
    this.ballVelocityControls = new TransformControls(this.camera, this.renderer.domElement);
    this.ballVelocityControls.setSize(0.85);
    this.ballVelocityControls.setSpace("local");
    this.ballVelocityControls.setMode("rotate");
    this.ballVelocityControls.attach(this.ballVelocityHandle);
    this.ballVelocityControls.visible = false;
    this.ballVelocityControls.addEventListener("dragging-changed", (event) => {
      this.controls.enabled = !event.value;
      this._velDragging = event.value;
      if (!event.value) this._onVelocityDragEnd();
    });
    this.ballVelocityControls.addEventListener("objectChange", () => this._onVelocityGizmoChange());
    this.scene.add(this.ballVelocityControls);
  }

  // move | aim | speed — pick which gizmo drives the launch frame (one at a time).
  setBallLaunchMode(mode) {
    this.ballLaunchMode = mode === "aim" || mode === "speed" ? mode : "move";
    if (this.ballLaunchMode === "speed" && this.ballVelocityControls) {
      this.ballVelocityControls.setMode("scale");
      this._velScaleRefSpeed = this._velSpeed || VEL_SPEED_MIN;
      this.ballVelocityHandle.scale.set(1, 1, 1);
    } else if (this.ballLaunchMode === "aim" && this.ballVelocityControls) {
      this.ballVelocityControls.setMode("rotate");
    }
    this._syncVelocityHandle();
    this._applyLaunchControlsVisibility();
  }

  _applyLaunchControlsVisibility() {
    const shown = !!(this.ballLaunchFrame && this.ballLaunchFrame.visible);
    const moveOn = shown && this.ballLaunchMode === "move";
    const velOn = shown && (this.ballLaunchMode === "aim" || this.ballLaunchMode === "speed");
    if (this.ballLaunchControls) {
      this.ballLaunchControls.visible = moveOn;
      this.ballLaunchControls.enabled = moveOn; // only the active gizmo grabs the mouse
    }
    if (this.ballVelocityControls) {
      this.ballVelocityControls.visible = velOn;
      this.ballVelocityControls.enabled = velOn;
    }
  }

  // Keep the (invisible) velocity handle co-located with the frame and oriented to
  // _velDir. Skipped mid-drag so the rotate gizmo's twist is not snapped under the user.
  _syncVelocityHandle() {
    if (!this.ballVelocityHandle || !this.ballLaunchFrame) return;
    this.ballVelocityHandle.position.copy(this.ballLaunchFrame.position);
    if (!this._velDragging) {
      this.ballVelocityHandle.quaternion.setFromUnitVectors(VEL_REF_DIR, this._velDir);
      this.ballVelocityHandle.scale.set(1, 1, 1);
    }
    this.ballVelocityHandle.updateMatrixWorld();
  }

  _onVelocityGizmoChange() {
    if (this.ballLaunchUpdateMuted) return;
    if (this.ballLaunchMode === "aim") {
      this._velDir = VEL_REF_DIR.clone().applyQuaternion(this.ballVelocityHandle.quaternion).normalize();
    } else if (this.ballLaunchMode === "speed") {
      const scale = this.ballVelocityHandle.scale;
      const s = (Math.abs(scale.x) + Math.abs(scale.y) + Math.abs(scale.z)) / 3;
      this._velSpeed = THREE.MathUtils.clamp(this._velScaleRefSpeed * s, VEL_SPEED_MIN, VEL_SPEED_MAX);
    }
    this.updateBallLaunchVelocity(this._velBase());
    if (this.ballLaunchFrame) {
      const p = this.ballLaunchFrame.position;
      this.updateBallLaunchPath([-p.x, p.z, p.y], this._velBase());
    }
    if (this.ballLaunchCallback) this.ballLaunchCallback(this.getBallLaunchConfig());
  }

  _onVelocityDragEnd() {
    if (this.ballLaunchMode === "speed") {
      this._velScaleRefSpeed = this._velSpeed;
      this.ballVelocityHandle.scale.set(1, 1, 1);
    }
    this._syncVelocityHandle();
  }

  // v0 in base coords from the current direction/speed (THREE<->base is an involution).
  _velBase() {
    const v = this._velDir.clone().multiplyScalar(this._velSpeed);
    return [-v.x, v.z, v.y];
  }

  enableBallLaunchFrame(callback) {
    this.ballLaunchCallback = callback;
    if (this.ballLaunchFrame) {
      this.ballLaunchFrame.visible = true;
      if (this.ballLaunchPath) this.ballLaunchPath.visible = true;
      this._syncVelocityHandle();
      this._applyLaunchControlsVisibility();
    }
  }

  setBallLaunchFrameVisible(visible) {
    if (this.ballLaunchFrame) this.ballLaunchFrame.visible = visible;
    if (this.ballLaunchPath) this.ballLaunchPath.visible = visible;
    this._applyLaunchControlsVisibility();
  }

  setBallLaunchConfig(config) {
    if (!this.ballLaunchFrame || !config || !config.p0 || !config.v0) return;
    this.ballLaunchUpdateMuted = true;
    if (config.flight_s != null) this._flightHorizon = config.flight_s;
    const [x, y, z] = config.p0;
    this.ballLaunchFrame.position.copy(this.baseToThree(x, y, z));
    // During a velocity-gizmo drag the gizmo owns _velDir/_velSpeed; only ingest v0
    // from config when set programmatically (panel inputs, reset, initial load).
    if (!this._velDragging) {
      const vThree = this.baseVectorToThree(config.v0[0], config.v0[1], config.v0[2]);
      this._velSpeed = vThree.length();
      this._velDir = this._velSpeed > 1e-6 ? vThree.clone().normalize() : new THREE.Vector3(0, 1, 0);
      this._velScaleRefSpeed = this._velSpeed;
    }
    this.updateBallLaunchVelocity(config.v0);
    this.updateBallLaunchPath(config.p0, config.v0);
    this.ballLaunchFrame.visible = true;
    if (this.ballLaunchPath) this.ballLaunchPath.visible = true;
    this._syncVelocityHandle();
    this._applyLaunchControlsVisibility();
    this.ballLaunchUpdateMuted = false;
  }

  updateBallLaunchVelocity(v0) {
    if (!this.ballLaunchVelocity) return;
    const [vx, vy, vz] = v0;
    const velocity = this.baseVectorToThree(vx, vy, vz);
    const speed = velocity.length();
    const direction = speed > 1e-6 ? velocity.clone().normalize() : new THREE.Vector3(0, 1, 0);
    this.ballLaunchVelocity.setDirection(direction);
    this.ballLaunchVelocity.setLength(Math.max(0.08, Math.min(0.65, speed * 0.12)), 0.07, 0.035);
  }

  updateBallLaunchPath(p0, v0) {
    if (!this.ballLaunchPath) return;
    const [x, y, z] = p0;
    const [vx, vy, vz] = v0;
    const points = [];
    const horizon = this._flightHorizon || 1.2;  // arc spans the configured flight time
    const steps = Math.max(36, Math.round(horizon * 24));
    const g = -9.81;
    for (let i = 0; i <= steps; i++) {
      const t = (i / steps) * horizon;
      points.push(this.baseToThree(
        x + vx * t,
        y + vy * t,
        z + vz * t + 0.5 * g * t * t,
      ));
    }
    this.ballLaunchPath.geometry.setFromPoints(points);
  }

  getBallLaunchConfig() {
    const position = this.ballLaunchFrame.position;
    return {
      p0: [-position.x, position.z, position.y],
      v0: this._velBase(),
    };
  }

  // catch: {ball_base:[x,y,z], ball_vel_base:[vx,vy,vz]} in the robot base frame
  // (CatchTelemetry). Draws the ball marker and a short ballistic prediction arc.
  setCatch(catchInfo) {
    if (!catchInfo || !catchInfo.ball_base) {
      this.setBallVisible(false);
      this.setPolicyGhostVisible(false);
      return;
    }
    if (!this.ballGroup) this.buildBall();
    const [bx, by, bz] = catchInfo.ball_base;
    this.ballMarker.position.copy(this.baseToThree(bx, by, bz));

    const v = catchInfo.ball_vel_base || [0, 0, 0];
    const points = [];
    const horizon = 1.0;
    const steps = 30;
    const g = -9.81;
    for (let i = 0; i <= steps; i++) {
      const t = (i / steps) * horizon;
      const px = bx + v[0] * t;
      const py = by + v[1] * t;
      const pz = bz + v[2] * t + 0.5 * g * t * t;
      points.push(this.baseToThree(px, py, pz));
    }
    this.ballPath.geometry.setFromPoints(points);
    this.ballGroup.visible = true;

    // Policy ghost: the joint pose the network commands this tick (joint_target),
    // overlaid on the live robot so you watch it react to the thrown ball.
    if (catchInfo.joint_target && catchInfo.joint_target.length && catchInfo.joint_names) {
      this.applyGhostMaterial("policy");
      this.setGhostJoints(catchInfo.joint_names, catchInfo.joint_target, this.policyGhost);
      this.setPolicyGhostVisible(this.policyGhostEnabled);
    }
  }

  setBallVisible(visible) {
    if (this.ballGroup) this.ballGroup.visible = visible;
  }

  setPolicyGhostVisible(visible) {
    if (this.policyGhost) this.policyGhost.visible = visible;
  }

  setPolicyGhostEnabled(enabled) {
    this.policyGhostEnabled = enabled;
    if (!enabled) this.setPolicyGhostVisible(false);
  }

  applyGhostMaterial(kind = "primary") {
    const ghost =
      kind === "reference" ? this.replayGhost : kind === "policy" ? this.policyGhost : this.ghost;
    const flagName =
      kind === "reference"
        ? "replayGhostMaterialApplied"
        : kind === "policy"
          ? "policyGhostMaterialApplied"
          : "ghostMaterialApplied";
    if (!ghost || this[flagName]) return;
    // policy = green (the pose the network commands), reference = amber, primary = blue.
    const color = kind === "reference" ? 0xe0a93e : kind === "policy" ? 0x35d39a : 0x4fa3ff;
    const opacity = kind === "policy" ? 0.5 : kind === "reference" ? 0.3 : 0.35;
    const material = new THREE.MeshStandardMaterial({
      color,
      transparent: true,
      opacity,
      depthWrite: false,
    });
    let meshCount = 0;
    ghost.traverse((child) => {
      if (child.isMesh) {
        child.material = material;
        meshCount += 1;
      }
    });
    // Meshes load asynchronously; only latch once they exist.
    if (meshCount > 0) this[flagName] = true;
  }

  playPreview(plan, { referencePlan = null, onProgress, onDone } = {}) {
    if (!this.ghost) return;
    this.applyGhostMaterial("primary");
    this.ghost.visible = true;
    if (this.replayGhost) {
      this.applyGhostMaterial("reference");
      this.replayGhost.visible = !!referencePlan;
    }
    this.preview = { plan, referencePlan, startMs: performance.now(), onProgress, onDone };
  }

  showGhostPlanEnd(plan) {
    if (!this.ghost || !plan || !plan.positions || plan.positions.length === 0) return;
    this.applyGhostMaterial("primary");
    this.ghost.visible = true;
    if (this.replayGhost) this.replayGhost.visible = false;
    this.preview = null;
    this.setGhostJoints(plan.joint_names, plan.positions[plan.positions.length - 1], this.ghost);
  }

  stopPreview() {
    if (this.ghost) this.ghost.visible = false;
    if (this.replayGhost) this.replayGhost.visible = false;
    const preview = this.preview;
    this.preview = null;
    if (preview && preview.onDone) preview.onDone();
  }

  // Live execution overlay: the real robot model is driven by /joint_states,
  // while these ghosts are driven directly by the execution progress fraction
  // (elapsed_s / total_s). blue = the commanded trajectory being executed
  // (should track the real robot), orange = a reference source (the raw policy
  // command), so the command-vs-realized gap is visible during the real move.
  showExecutionGhosts(plan, referencePlan, fraction) {
    if (!this.ghost || !plan) return;
    this.preview = null; // the execution clock drives the ghosts, not the preview loop
    this.applyGhostMaterial("primary");
    this.ghost.visible = true;
    this.setGhostJointsAtFraction(this.ghost, plan, fraction);
    if (this.replayGhost) {
      if (referencePlan) {
        this.applyGhostMaterial("reference");
        this.replayGhost.visible = true;
        this.setGhostJointsAtFraction(this.replayGhost, referencePlan, fraction);
      } else {
        this.replayGhost.visible = false;
      }
    }
  }

  setGhostJointsAtFraction(ghost, plan, fraction) {
    const times = plan.time_from_start_s;
    if (!ghost || !times || times.length === 0) return;
    const total = times[times.length - 1];
    this.setGhostJointsAtTime(ghost, plan, Math.max(0, Math.min(1, fraction)) * total);
  }

  hideExecutionGhosts() {
    if (this.preview) return; // a preview is mid-flight; let it own the ghosts
    if (this.ghost) this.ghost.visible = false;
    if (this.replayGhost) this.replayGhost.visible = false;
  }

  tickPreview() {
    if (!this.preview) return;
    const { plan, referencePlan, startMs, onProgress } = this.preview;
    const total = plan.time_from_start_s[plan.time_from_start_s.length - 1];
    const t = (performance.now() - startMs) / 1000;

    if (t >= total) {
      this.setGhostJoints(plan.joint_names, plan.positions[plan.positions.length - 1], this.ghost);
      if (referencePlan && this.replayGhost) {
        this.setGhostJoints(
          referencePlan.joint_names,
          referencePlan.positions[referencePlan.positions.length - 1],
          this.replayGhost,
        );
      }
      if (onProgress) onProgress(1);
      this.stopPreview();
      return;
    }

    this.setGhostJointsAtTime(this.ghost, plan, t);
    if (referencePlan && this.replayGhost) this.setGhostJointsAtTime(this.replayGhost, referencePlan, t);
    if (onProgress) onProgress(t / total);
  }

  setGhostJointsAtTime(ghost, plan, t) {
    const times = plan.time_from_start_s;
    if (!ghost || !times || times.length === 0 || !plan.positions || plan.positions.length === 0) return;
    const total = times[times.length - 1];
    if (t >= total || plan.positions.length === 1) {
      this.setGhostJoints(plan.joint_names, plan.positions[plan.positions.length - 1], ghost);
      return;
    }

    // plan.positions[i] corresponds to times[i]; first point may be at t=0.
    let segment = 0;
    while (segment < times.length - 1 && times[segment + 1] < t) segment += 1;
    const t0 = times[segment];
    const t1 = times[segment + 1];
    const alpha = t1 > t0 ? (t - t0) / (t1 - t0) : 1;
    const a = plan.positions[segment];
    const b = plan.positions[segment + 1];
    const interpolated = a.map((value, i) => value + (b[i] - value) * alpha);
    this.setGhostJoints(plan.joint_names, interpolated, ghost);
  }

  setGhostJoints(names, positions, ghost = this.ghost) {
    if (!ghost) return;
    for (let i = 0; i < names.length; i++) {
      const joint = ghost.joints[names[i]];
      if (joint) joint.setJointValue(positions[i]);
    }
  }

  enableTarget(callback) {
    this.targetCallback = callback;
    if (this.targetFrame) {
      this.targetFrame.visible = true;
      this.targetControls.visible = true;
    }
  }

  setTargetMode(mode) {
    if (!this.targetControls) return;
    this.targetControls.setMode(mode === "rotate" ? "rotate" : "translate");
    this.targetControls.setSpace("local");
  }

  setTargetPose(pose) {
    if (!this.targetFrame) return;
    this.targetUpdateMuted = true;
    const [x, y, z] = pose.xyz_m;
    this.targetFrame.position.set(-x, z, y);

    const [roll, pitch, yaw] = pose.rpy_rad;
    // ROS RPY is extrinsic X-Y-Z, i.e. R = Rz(yaw)*Ry(pitch)*Rx(roll): three.js order "ZYX".
    const baseQuat = new THREE.Quaternion().setFromEuler(new THREE.Euler(roll, pitch, yaw, "ZYX"));
    const baseLinkQuat = BASE_TO_BASE_LINK_Q.clone().multiply(baseQuat);
    this.targetFrame.quaternion.copy(ROS_TO_THREE_Q.clone().multiply(baseLinkQuat));
    this.targetFrame.visible = true;
    this.targetControls.visible = true;
    this.targetUpdateMuted = false;
  }

  getTargetPose() {
    const position = this.targetFrame.position;
    const baseLinkQuat = THREE_TO_ROS_Q.clone().multiply(this.targetFrame.quaternion);
    const baseQuat = BASE_LINK_TO_BASE_Q.clone().multiply(baseLinkQuat);
    const rpy = new THREE.Euler().setFromQuaternion(baseQuat, "ZYX");
    return {
      xyz_m: [-position.x, position.z, position.y],
      rpy_rad: [rpy.x, rpy.y, rpy.z],
    };
  }

  tick() {
    this.tickPreview();
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
  }
}
