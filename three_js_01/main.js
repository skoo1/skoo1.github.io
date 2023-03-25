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
    '../models/butterfly_nut.gltf',
    (gltfScene) => {
        model_nut = gltfScene
        scene.add(gltfScene.scene)
    }
)

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
