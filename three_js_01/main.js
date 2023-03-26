import * as THREE from 'three'
import { OrbitControls } from "https://unpkg.com/three@0.150.1/examples/jsm/controls/OrbitControls"
import { GLTFLoader } from 'https://unpkg.com/three@0.150.1/examples/jsm/loaders/GLTFLoader'

//Scene
const scene = new THREE.Scene()

//Axes Show
const axesHelper = new THREE.AxesHelper(20);
scene.add( axesHelper );

//Mesh Ball
const geometry_ball = new THREE.SphereGeometry(1, 64, 64)
const material_ball = new THREE.MeshStandardMaterial({
    color: "#f0ff83",
})
const mesh_ball = new THREE.Mesh(geometry_ball, material_ball)
scene.add(mesh_ball)

//Mesh Butterfly Nut
const material_nut = new THREE.MeshStandardMaterial({
    color: 0xffffff,
})

let model_nut
const loader = new GLTFLoader()
loader.load(
    'butterfly_nut.gltf',
    (gltfScene) => {
        model_nut = gltfScene
        scene.add(gltfScene.scene)
    }
)

//Dynamics Calculations
const htime = 0.01
const NutIB = new THREE.Matrix3()
NutIB.set(1, 0, 0,
          0, 2, 0,
          0, 0, 3)

const Nutq0_world = new THREE.Quaternion()
Nutq0_world.set(0, 0, 0, 1)
const Nutw0_body = new THREE.Vector3()
Nutw0_body.set(0, 0, 1)
const NutTorque_body = new THREE.Vector3()
NutTorque_body.set(0, 0, 0)

const Nut_q_current_world = new THREE.Quaternion()
Nut_q_current_world.copy(Nutq0_world)
const Nut_q_current_body = new THREE.Quaternion()
Nut_q_current_body.set(0, 0, 0, 1)
const Nut_w_current_body = new THREE.Vector3()
Nut_w_current_body.copy(Nutw0_body)

// calculate w_croxx_IB_mult_w
const Nut_Iw_current_body = new THREE.Vector3()
Nut_Iw_current_body.copy(Nut_w_current_body)
Nut_Iw_current_body.applyMatrix3(NutIB)
const Nut_w_x_Iw_current_body = new THREE.Vector3()
Nut_w_x_Iw_current_body.copy(Nut_w_current_body)
Nut_w_x_Iw_current_body.cross(Nut_Iw_current_body)

// calculate w_dot_body = IBInv * (Torque - w_croxx_IB_mult_w)
const Nut_Torque_sub_wIw = new THREE.Vector3()
Nut_Torque_sub_wIw.copy(NutTorque_body)
Nut_Torque_sub_wIw.sub(Nut_w_x_Iw_current_body)
const NutIBInv = new THREE.Matrix3()
NutIBInv.copy(NutIB)
NutIBInv.invert()
const Nut_w_dot_current_body = new THREE.Vector3()
Nut_w_dot_current_body.copy(Nut_Torque_sub_wIw)
Nut_w_dot_current_body.applyMatrix3(NutIBInv)

var qx, qy, qz, qw, wx, wy, wz
qx = Nut_q_current_body.x
qy = Nut_q_current_body.y
qz = Nut_q_current_body.z
qw = Nut_q_current_body.w
wx = Nut_w_current_body.x
wy = Nut_w_current_body.y
wz = Nut_w_current_body.z

const Nut_q_dot = new THREE.Quaternion()
Nut_q_dot.x = 0.5*(wx*qw + wz*qy - wy*qz)
Nut_q_dot.y = 0.5*(wy*qw - wz*qx + wx*qz)
Nut_q_dot.z = 0.5*(wz*qw + wy*qx - wx*qy)
Nut_q_dot.w = 0.5*(-1*wx*qx - wy*qy - wz*qz)

const Nut_q_next_body = new THREE.Quaternion()
Nut_q_next_body.x = Nut_q_current_body.x + htime*Nut_q_dot.x
Nut_q_next_body.y = Nut_q_current_body.y + htime*Nut_q_dot.y
Nut_q_next_body.z = Nut_q_current_body.z + htime*Nut_q_dot.z
Nut_q_next_body.w = Nut_q_current_body.w + htime*Nut_q_dot.w
Nut_q_next_body.normalize()

const Nut_w_next_body = new THREE.Vector3()
Nut_w_next_body.x = Nut_w_current_body.x + htime*Nut_w_dot_current_body.x
Nut_w_next_body.y = Nut_w_current_body.y + htime*Nut_w_dot_current_body.y
Nut_w_next_body.z = Nut_w_current_body.z + htime*Nut_w_dot_current_body.z



//Sizes
const sizes = {
    width: window.innerWidth,
    height: window.innerHeight,
}

//Light
const light1 = new THREE.PointLight(0xffffff, 1, 200)
light1.position.set(0, 50, 50)
scene.add(light1)
const light2 = new THREE.PointLight(0xffffff, 1, 200)
light2.position.set(0, -50, -50)
scene.add(light2)

//Camera
const camera = new THREE.PerspectiveCamera(45, sizes.width /sizes.height, 0.1, 1000.0)
camera.position.z = 60
scene.add(camera)

//Renderer
const canvas = document.querySelector('.webgl')
const renderer = new THREE.WebGLRenderer({canvas})
renderer.setSize(sizes.width, sizes.height)
renderer.render(scene, camera)

//Control
const control = new OrbitControls(camera, canvas)
control.enableDamping = true

//Resize
window.addEventListener('resize', () => {
    //Update Sizes
    sizes.width = window.innerWidth
    sizes.height = window.innerHeight
    //Update Camera
    camera.aspect = sizes.width / sizes.height
    camera.updateProjectionMatrix()
    renderer.setSize(sizes.width, sizes.height)
})

var frameCount = 0
var fpsInterval, startTime, now, then, elapsed

startLoopAtFPS(30) // max FPS is 60

function startLoopAtFPS(fps) {
    fpsInterval = 1000 / fps
    then = Date.now()
    startTime = then
    console.log(startTime)
    loop()
}

function loop() {
    // request another frame
    requestAnimationFrame(loop);

    // calc elapsed time since the last loop
    now = Date.now();
    elapsed = now - then;

    // if enough time has elapsed, draw the next frame
    if (elapsed > fpsInterval) {
        then = now - (elapsed % fpsInterval);

        // draw animating objects here...
        if (model_nut) {
            const qrot1 = new THREE.Quaternion()
            qrot1.setFromAxisAngle(new THREE.Vector3( 0, 1, 0 ), Math.PI / 200)
            model_nut.scene.applyQuaternion(qrot1)
        }
        control.update()
        renderer.render(scene, camera)
    
        // below code is used for testing, whether the frame is animating at the specified fps
        // var sinceStart = now - startTime;
        // var currentFps = Math.round((1000 / (sinceStart / ++frameCount)) * 100) / 100;
        // console.log(currentFps)
    }
}


