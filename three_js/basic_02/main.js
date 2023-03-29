import * as THREE from 'three'
import { OrbitControls } from "https://unpkg.com/three@0.150.1/examples/jsm/controls/OrbitControls.js"
                               
//Scene
const scene = new THREE.Scene()

//Mesh
const geometry = new THREE.SphereGeometry(3, 64, 64)
const material = new THREE.MeshStandardMaterial({
    color: "#00ff83",
})
const mesh = new THREE.Mesh(geometry, material)
scene.add(mesh)

//Sizes
const sizes = {
    width: window.innerWidth,
    height: window.innerHeight*2/3,
}

//Light
const light = new THREE.PointLight(0xffffff, 1, 100)
light.position.set(0, 10, 10)
scene.add(light)

//Camera
const camera = new THREE.PerspectiveCamera(45, sizes.width /sizes.height, 0.1, 100.0)
camera.position.z = 20
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
    sizes.height = window.innerHeight*2/3
    //Update Camera
    camera.aspect = sizes.width / sizes.height
    camera.updateProjectionMatrix()
    renderer.setSize(sizes.width, sizes.height)
})

const loop = () => {
    control.update()
    renderer.render(scene, camera)
    window.requestAnimationFrame(loop)
}
loop()