// main.js – improved rigid-body spinning-top simulation  (ball-and-socket constraint 버전)
// ----------------------------------------------------------------------------------------

import * as THREE from 'three';
import { OrbitControls } from 'https://unpkg.com/three@0.150.1/examples/jsm/controls/OrbitControls';
import { GLTFLoader } from 'https://unpkg.com/three@0.150.1/examples/jsm/loaders/GLTFLoader';

// ────────────────────────────────────────────────────────────
// Scene - Camera - Renderer
// ────────────────────────────────────────────────────────────
const scene = new THREE.Scene();

// Visual helpers – world-frame XYZ, body-frame XYZ
const axesWorld = new THREE.AxesHelper(0.15);
scene.add(axesWorld);
const axesBody = new THREE.AxesHelper(0.1);
scene.add(axesBody);

// Lighting
const light1 = new THREE.PointLight(0xffffff, 5, 200);
light1.position.set(0, 50, 50);
scene.add(light1);
const light2 = new THREE.PointLight(0xffffff, 5, 200);
light2.position.set(0, -50, -50);
scene.add(light2);

// Camera
const sizes = { width: window.innerWidth, height: window.innerHeight };
const camera = new THREE.PerspectiveCamera(45, sizes.width / sizes.height, 0.1, 1000.0);
camera.position.z = 0.3;
camera.position.y = 0.3;
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
const dt = 1e-4;                      // physics time-step (s)
const substeps = 200;                 // physics steps per rendered frame → 60 FPS

const m = 0.05;
const g = 9.81;
const h_cm = -0.05;
const rB = new THREE.Vector3(0, h_cm, 0);

const Ixx = 2.6e-5;
const Iyy = 4.8e-5;
const Izz = 2.6e-5;
const IB = new THREE.Matrix3().set(Ixx, 0, 0, 0, Iyy, 0, 0, 0, Izz);
const IBInv = IB.clone().invert();

// Initial orientation (8° tilt about x)
const qW = new THREE.Quaternion().setFromAxisAngle(
            new THREE.Vector3(1, 0, 0),
            THREE.MathUtils.degToRad(20));

// Initial COM position so that C=0  (  x₀ = −R(q₀) rB  )          // 새 코드
const xW = rB.clone().applyQuaternion(qW).multiplyScalar(-1);       // 새 코드
const vW = new THREE.Vector3();                                     // 새 코드

// Initial angular velocity (body frame)
const wB = new THREE.Vector3(0, 80, 0);

// temporary matrices
const tmpM = new THREE.Matrix3();

// ────────────────────────────────────────────────────────────
// GLTF loading
// ────────────────────────────────────────────────────────────
let modelNut;
const loader = new GLTFLoader();
loader.load('spinningtop.gltf', (gltf) => {
  const root = gltf.scene;
  root.scale.setScalar(0.0033);
  root.position.set(0, 0, 0);
  modelNut = gltf;
  scene.add(root);
});

// ────────────────────────────────────────────────────────────
// Helper small functions
// ────────────────────────────────────────────────────────────
function skew(v){
  return new THREE.Matrix3().set(0,-v.z, v.y, v.z,0,-v.x, -v.y,v.x,0);
}

function computeAMatrix(rW, IWInv, out){
  // A = (1/m)E - [r]× I⁻¹ [r]×ᵀ
  const sR = skew(rW);
  out.copy(sR).multiply(IWInv).multiply(sR.clone().transpose());
  out.elements[0] += 1/m;
  out.elements[4] += 1/m;
  out.elements[8] += 1/m;
  return out;
}

/* quaternion→Matrix3 (기존) */
function quat_to_mat3(q, mOut){
  const x2=q.x+q.x, y2=q.y+q.y, z2=q.z+q.z;
  const xx=q.x*x2, yy=q.y*y2, zz=q.z*z2;
  const xy=q.x*y2, xz=q.x*z2, yz=q.y*z2;
  const wx=q.w*x2, wy=q.w*y2, wz=q.w*z2;
  mOut.set(1-(yy+zz), xy-wz, xz+wy,
           xy+wz, 1-(xx+zz), yz-wx,
           xz-wy, yz+wx, 1-(xx+yy));
  return mOut;
}

// ────────────────────────────────────────────────────────────
// Physics integration (ball-and-socket + Baumgarte)
// ────────────────────────────────────────────────────────────
const beta = 0.5, gamma = 5.0;

function stepPhysics() {
  /*--- helpers ------------------------------------------------*/
  const R = quat_to_mat3(qW, new THREE.Matrix3());
  const rW = rB.clone().applyQuaternion(qW);

  /*--- inertia in world ---------------------------------------*/
  const IW = R.clone().multiply(IB).multiply(R.clone().transpose());
  const IWInv = IW.clone().invert();

  /*--- external forces/torques --------------------------------*/
  const Fg = new THREE.Vector3(0, -m*g, 0);
  const Tg = rW.clone().cross(Fg);

  /*--- generalized velocity -----------------------------------*/
  const wW = wB.clone().applyMatrix3(R);

  /*--- constraint Jacobian J = [E , -[r]×] --------------------*/
  const J11 = new THREE.Matrix3().identity();
  const J12 = skew(rW).multiplyScalar(-1);

  /*--- Baumgarte bias b ---------------------------------------*/
  const Cpos = xW.clone().add(rW);
  const Jv = vW.clone().add( wW.clone().cross(rW) );
  const b = Jv.clone().multiplyScalar(-2*beta).addScaledVector(Cpos, -gamma*gamma);

  /*--- Schur complement A λ = b  (3×3) ------------------------*/
  const A = new THREE.Matrix3();
  computeAMatrix(rW, IWInv, A);
  const lambda = b.clone().applyMatrix3(A.clone().invert());

  /*--- constraint forces --------------------------------------*/
  const Fc = lambda;
  const Tc = rW.clone().cross(lambda);

  /*--- linear & angular accel ---------------------------------*/
  const aLin = Fg.clone().add(Fc).multiplyScalar(1/m);
  const wCrossIw = wW.clone().cross( wW.clone().applyMatrix3(IW) );
  const aAngW = Tg.clone().add(Tc).sub(wCrossIw).applyMatrix3(IWInv);

  /*--- semi-implicit Euler (linear) ---------------------------*/
  vW.addScaledVector(aLin, dt);
  xW.addScaledVector(vW,  dt);

  /*--- semi-implicit Euler (angular – body) -------------------*/
  wB.addScaledVector( aAngW.clone().applyMatrix3( R.clone().transpose() ), dt );

  /*--- orientation update -------------------------------------*/
  const wLen = wB.length();
  if (wLen > 1e-10){
    const half = 0.5*wLen*dt;
	const s = Math.sin(half)/wLen;
    const dq = new THREE.Quaternion(wB.x*s, wB.y*s, wB.z*s, Math.cos(half));
    qW.multiply(dq).normalize();
  }

  /*--- soft position projection (50 %) ------------------------*/
  const err = xW.clone().add(rB.clone().applyQuaternion(qW));
  xW.sub(err.multiplyScalar(0.05));

}

// ────────────────────────────────────────────────────────────
// Main animation loop
// ────────────────────────────────────────────────────────────
function animate() {
  requestAnimationFrame(animate);

  if (modelNut) {
    for (let i=0; i<substeps; ++i){ stepPhysics(); }
    modelNut.scene.position.copy(xW);
    modelNut.scene.setRotationFromQuaternion(qW);
    axesBody.position.copy(xW);
    axesBody.setRotationFromQuaternion(qW);
  }
  controls.update();
  renderer.render(scene, camera);
}
animate();

/*──────────────────────────────
  Diagnostics
──────────────────────────────*/
setInterval(()=>{
  // ---------- energy ----------
  const Rot = wB.clone().applyMatrix3(IB);         // body basis
  const T = 0.5*m*vW.lengthSq() + 0.5*wB.dot(Rot); // lin + rot
  const y = rB.clone().applyQuaternion(qW).y + xW.y;
  const V = m * g * y;                             // g:+Y up 기준
  const C = xW.clone().add( rB.clone().applyQuaternion(qW) ).length();
  console.log(`E=${(T+V).toFixed(5)}  |C|=${C.toExponential(2)}`);
},1000);

setInterval(()=>{
  // ---------- constraint & angular momentum ----------
  const Cnorm = xW.clone().add( rB.clone().applyQuaternion(qW) ).length();

  // world-frame L
  const Rmat = quat_to_mat3(qW, new THREE.Matrix3());
  const wW   = wB.clone().applyMatrix3(Rmat);
  const IW   = Rmat.clone().multiply(IB).multiply(Rmat.clone().transpose());
  const L    = wW.clone().applyMatrix3(IW);

  console.log(`‖C‖=${Cnorm.toExponential(2)}  ‖L‖=${L.length().toExponential(3)}`);
},1000);

