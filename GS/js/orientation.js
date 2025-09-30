import * as THREE from "three";
import { GLTFLoader } from "three/addons/loaders/GLTFLoader.js";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";

var cubesat;



const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(
  75,
  1,
  0.1,
  1000
);

// camera.position.set(0, 20, 100);

const loader = new GLTFLoader();




 var ambient = new THREE.AmbientLight(0x404040, 2.0);
 scene.add(ambient);
 var keyLight = new THREE.DirectionalLight("0x404040", 1.0);
 keyLight.position.set(-100, 0, 100);
 keyLight.castShadow = true;
 var fillLight = new THREE.DirectionalLight("0x404040", 1.0);
 fillLight.position.set(100, 0, 100);
 var backLight = new THREE.DirectionalLight(0x404040, 1.0);
 backLight.position.set(100, 0, -100).normalize();
 scene.add(keyLight);
 scene.add(fillLight);
 scene.add(backLight);  

loader.load(
  "./models/cubesat.glb",
  function (gltf) {
    console.log("Adding CS to scene");
    cubesat = gltf.scene;
    scene.add(cubesat);

    cubesat.position.set(0,0,0);
  },
  function (xhr) {
    console.log((xhr.loaded / xhr.total) * 100 + "% loaded");
  },
  undefined,
  function (error) {
    console.error(error);
  }
);


const renderer = new THREE.WebGLRenderer({
    antialias: true,
});


renderer.setPixelRatio(window.devicePixelRatio);
renderer.shadowMap.enabled = true;
renderer.toneMappingExposure = 2.3;
renderer.gammaFactor = 0;


const controls = new OrbitControls(camera, renderer.domElement);
controls.update();

var cH = document.getElementById("orientationDisp").clientHeight;
var cW = document.getElementById("orientationDisp").clientWidth;


renderer.setSize(cW, cW, false);
camera.aspect = 1;
camera.updateProjectionMatrix();
document.getElementById("orientationDisp").appendChild(renderer.domElement);


// camera.position.z = 5;
camera.position.set(100, 100, 100);

var y = 0, p = 0, r = 0;

function animate() {

  if(cubesat)  cubesat.rotation.set(o.yaw, o.pitch, o.roll);

  controls.update();

  requestAnimationFrame(animate);

  renderer.render(scene, camera);
}

animate();

controls.addEventListener( "change", event => {  
    console.log( controls.object.position ); 
})