// main.js – improved rigid-body spinning-top simulation
// ------------------------------------------------------------

import * as THREE from 'three';
import { OrbitControls } from 'https://unpkg.com/three@0.150.1/examples/jsm/controls/OrbitControls';
import { GLTFLoader } from 'https://unpkg.com/three@0.150.1/examples/jsm/loaders/GLTFLoader';

// ────────────────────────────────────────────────────────────
// Scene - Camera - Renderer
// ────────────────────────────────────────────────────────────
const scene = new THREE.Scene();

// Visual helpers – world-frame XYZ, body-frame XYZ
const axesWorld = new THREE.AxesHelper(25);
scene.add(axesWorld);
const axesBody = new THREE.AxesHelper(25);
scene.add(axesBody);

// Lighting
const light1 = new THREE.PointLight(0xffffff, 5, 200);
light1.position.set(0, 50, 50);
scene.add(light1);
const light2 = new THREE.PointLight(0xffffff, 5, 200);
light2.position.set(0, -50, -50);
scene.add(light2);

// Camera
const sizes = {
  width: window.innerWidth,
  height: window.innerHeight,
};
const camera = new THREE.PerspectiveCamera(45, sizes.width / sizes.height, 0.1, 1000.0);
camera.position.z = 80;
scene.add(camera);

// Renderer
const canvas = document.querySelector('.webgl');
const renderer = new THREE.WebGLRenderer({ canvas });
renderer.setSize(sizes.width, sizes.height);

// Orbit controls
const controls = new OrbitControls(camera, canvas);
controls.enableDamping = true;

// Responsive resize
window.addEventListener('resize', () => {
  sizes.width = window.innerWidth;
  sizes.height = window.innerHeight;

  camera.aspect = sizes.width / sizes.height;
  camera.updateProjectionMatrix();
  renderer.setSize(sizes.width, sizes.height);
});

// ────────────────────────────────────────────────────────────
// Physics constants & state
// ────────────────────────────────────────────────────────────
const dt = 1e-5;               // physics time-step (s)
const substeps = 100;          // physics steps per rendered frame → 60 FPS

// Physical parameters of the top (edit as needed)
const m = 0.05;                   // mass (kg)
const g = 9.81;                   // gravity (m/s²)
const h_cm = 0.025;               // CM height above tip (m)
const rB = new THREE.Vector3(0, h_cm, 0);           // vector tip→CM, body frame

// Inertia tensor (diagonal) in body frame (kg·m²)
const Ixx = 2.6e-5;
const Iyy = 4.8e-5;
const Izz = 2.6e-5;
const IB = new THREE.Matrix3().set(Ixx, 0, 0,
                                   0, Iyy, 0,
                                   0, 0, Izz);
const IBInv = IB.clone().invert();

// Initial orientation (world) : 10° tilt about x-axis
const qW = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), THREE.MathUtils.degToRad(8));
// Initial angular velocity in body frame (rad/s)
const wB = new THREE.Vector3(0, 30, 0);            // spin about body y-axis

// temporary variables
const tmpM  = new THREE.Matrix3();

// ────────────────────────────────────────────────────────────
// GLTF loading
// ────────────────────────────────────────────────────────────
let modelNut; // will hold the loaded GLTF scene
const loader = new GLTFLoader();
loader.load('spinningtop.gltf', (gltf) => {
  modelNut = gltf;
  // y-axis (up) : tip at y=0, CM at +h_cm
  gltf.scene.position.set(0, 0, 0);
  scene.add(gltf.scene);
});

// ────────────────────────────────────────────────────────────
// Physics integration (semi-implicit Euler)
// ────────────────────────────────────────────────────────────
function stepPhysics() {
  // 1) Compute gravity torque in body frame
  const qInv = qW.clone().invert();
  const gW = new THREE.Vector3(0, -g, 0);
  const rW = rB.clone().applyQuaternion(qW);           // CM vector in world frame
  const tauW = rW.clone().cross(gW).multiplyScalar(m); // τ = r × (m g)
  const tauB = tauW.clone().applyQuaternion(qInv);     // world → body

  // 2) ẇ = I⁻¹ (τ − w×I w)
  const Iw = wB.clone().applyMatrix3(IB);
  const wCrossIw = wB.clone().cross(Iw);
  const wDotB = tauB.clone().sub(wCrossIw).applyMatrix3(IBInv);

  // 3) Semi-implicit Euler — update w first
  wB.addScaledVector(wDotB, dt);

  // 4) dq from exponential-map (ω is body-frame)    dq = [cosθ/2, ê sinθ/2]
  const wLen = wB.length();
  let dq;
  if (wLen > 1e-12) {
    const half = 0.5 * wLen * dt;
    const s = Math.sin(half) / wLen;
    dq = new THREE.Quaternion(wB.x * s, wB.y * s, wB.z * s, Math.cos(half));
  } else {
    dq = new THREE.Quaternion(0,0,0,1);
  }

  // 5) orientation update  (RIGHT multiply : body-frame ω)
  qW.multiply(dq).normalize();        //  q ← q ⊗ dq   (RIGHT mult.)
}

// ────────────────────────────────────────────────────────────
// Main animation loop
// ────────────────────────────────────────────────────────────
function animate() {
  requestAnimationFrame(animate);

  // Physics sub-stepping
  if (modelNut) {
    for (let i = 0; i < substeps; i++) {
      stepPhysics();
    }
    modelNut.scene.setRotationFromQuaternion(qW);
    axesBody.setRotationFromQuaternion(qW);
  }

  controls.update();
  renderer.render(scene, camera);
}
animate();

/*───────────────────────────────────────────
  (옵션) console diagnostics – energy & constraint
───────────────────────────────────────────*/
setInterval(() => {
  const R = quat_to_mat3(qW, tmpM);
  const Iv = wB.clone().applyMatrix3(IB);         // Iω (body)
  const T  = 0.5 * wB.dot(Iv);                    // rotational KE
  const y  = rB.clone().applyQuaternion(qW).y;    // COM height
  const V  = m * g * y;
  const C  = rB.clone().applyQuaternion(qW).length();
  console.log('E=', (T+V).toFixed(5), '  |C|=', C.toExponential(2));
}, 1000);


/* helper: quaternion → Matrix3 */
function quat_to_mat3(q, mOut){
  const e = q;   // alias
  const x2 = e.x + e.x,   y2 = e.y + e.y,   z2 = e.z + e.z;
  const xx = e.x * x2,    yy = e.y * y2,    zz = e.z * z2;
  const xy = e.x * y2,    xz = e.x * z2,    yz = e.y * z2;
  const wx = e.w * x2,    wy = e.w * y2,    wz = e.w * z2;
  mOut.set(1 - (yy + zz),    xy - wz,          xz + wy,
           xy + wz,          1 - (xx + zz),    yz - wx,
           xz - wy,          yz + wx,          1 - (xx + yy));
  return mOut;
}