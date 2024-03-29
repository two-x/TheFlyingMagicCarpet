/*

8-bit RGB332 color picker, using colors laid out in HSV color cylinder.

Copyright (c) Roger Cheng

MIT License

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

*/

//////////////////////////////////////////////////////////////////////////
//
//  Three.js (https://threejs.org/) objects

import * as THREE from './three.js';
import {OrbitControls} from './OrbitControls.js';

/** Three.js Scene */
var scene;

/** Three.js Camera, usually PerspectiveCamera */
var camera;

/** Three.js Renderer */
var renderer;

/** Three.js Raycaster */
var rayCaster;

/** Three.js timer object */
var clock;

/** Three.js Orbit control */
var orbitControl;

//////////////////////////////////////////////////////////////////////////
//
//  HTML DOM manipulation

/** Collection of all on-screen text objects */
var onScreenText;

/** Text object for RGB332 hex code */
var rgb332Text;

/** Map object from Three.js mesh to RGb332 class */
var colorMap;

//////////////////////////////////////////////////////////////////////////
//
//  Pointer hit-testing

/** Three.js Group of all the color blocks */
var hsvCyl;

/** Three.js Mesh object at location of PointerDown event */
var downTarget;

/** Three.js Vector2 representing normalized pointer event location */
var pointerLocation;

//////////////////////////////////////////////////////////////////////////
//
//  Animating HSV/RGB spaces

var toRGBanim;
var toHSVanim;
var animMixers;

var button;
var isHSV;

/** Called once upon HTML DOM initialization. */
function begin() {
  // Initialize Three.js environment, copied from tutorial.
  scene = new THREE.Scene();

  var aspect = window.innerWidth / window.innerHeight;
  camera = new THREE.PerspectiveCamera( 75, aspect, 0.1, 1000 );
  camera.position.z = 35;

  renderer = new THREE.WebGLRenderer( { antialias: true } );
  renderer.setSize( window.innerWidth, window.innerHeight );
  renderer.setAnimationLoop(animate);
  document.body.appendChild( renderer.domElement );

  // Set up raycaster and location we'll use with it for pointer hit testing.
  rayCaster = new THREE.Raycaster();
  pointerLocation = new THREE.Vector2();

  // Create clock
  clock = new THREE.Clock();

  // Orbit control allows user to navigate the 3D space
  orbitControl = new OrbitControls( camera, renderer.domElement );

  // Find text elements we'll be updating as we go.
  onScreenText = document.getElementsByClassName("onScreenText");
  rgb332Text = document.getElementById("rgb332code");

  // Start listening to pointer events.
  window.addEventListener( 'pointerdown', down_handler, false );
  window.addEventListener( 'pointerup', up_handler, false );

  // Connect the color model switch button
  button = document.getElementById("modelSwitch");
  button.addEventListener('click', switchColorModel);
  isHSV = true;
}

/** Update Three.js objects whenever window is resized */
function resizeView() {
  renderer.setSize( window.innerWidth, window.innerHeight );
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
}

/** Remember target at location of pointer down */
function down_handler(pointerEvent) {
  downTarget = pointerObject(pointerEvent);
}

/** If target at location of pointer up is the same object, treat as click. */
function up_handler(pointerEvent) {
  var upTarget = pointerObject(pointerEvent);

  if (upTarget != null && upTarget == downTarget) {
    updateBackground(colorMap.get(upTarget));
  }
}

/** Find Three.js Mesh object at the location of pointer event */
function pointerObject(pointerEvent) {
  if (pointerEvent.isPrimary) {
    // From https://threejs.org/docs/index.html#api/en/core/Raycaster
    pointerLocation.x = ( pointerEvent.clientX / window.innerWidth ) * 2 - 1;
    pointerLocation.y = - ( pointerEvent.clientY / window.innerHeight ) * 2 + 1;

    rayCaster.setFromCamera(pointerLocation, camera);

    const intersects = rayCaster.intersectObjects(scene.children, true);

    if (intersects.length > 0) {
      // First object is the closest object
      return intersects[0].object;
    }
  }

  return null;
}

/** Update background and on-screen text to match new background color */
function updateBackground(rgbObject) {
  var newTextColor;
  var hexPrefix;

  // Display RGB332 hex code
  if (rgbObject.color8 < 0x10) {
    hexPrefix = "0x0";
  } else {
    hexPrefix = "0x";
  }
  rgb332Text.textContent = hexPrefix+rgbObject.color8.toString(16).toUpperCase();

  // Update on-screen text color
  if (rgbObject.val > 0.5) {
    // Black text for light background
    newTextColor = "black";
  } else {
    // White text for dark background
    newTextColor = "white";
  }
  for(var i = 0; i < onScreenText.length; i++) {
    onScreenText[i].style.color = newTextColor;
  }

  // Update background color
  scene.background = new THREE.Color(rgbObject.color24);
}

/** Calculate RGB888 and HSV equivalents of given RGB332 value */
class RGB332 {
  constructor(rgb332) {
    this.color8 = rgb332;

    // Convert to 24-bit RGB888 and also prepare fractional RGB for later
    this.redf = (this.color8 >> 5)/7;
    this.red8 = this.redf * 0xFF;

    this.greenf = ((this.color8 & 0x1C) >> 2)/7;
    this.green8 = this.greenf * 0xFF;

    this.bluef = (this.color8 & 0x03)/3;
    this.blue8 = this.bluef * 0xFF;

    this.color24 = this.red8 << 16 | this.green8 << 8 | this.blue8;

    // Use fractional RGB to calculate HSV using algorithm modified from
    // https://dystopiancode.blogspot.com/2012/06/hsv-rgb-conversion-algorithms-in-c.html

    var major = Math.max(this.redf, this.greenf, this.bluef);
    var minor = Math.min(this.redf, this.greenf, this.bluef);
    var chroma = major-minor;

    this.val = major;

    if (0.0 != chroma) {
      if (major == this.redf) {
        this.hue = (( this.greenf - this.bluef ) / chroma) % 6.0;
      } else if (major == this.greenf) {
        this.hue = (( this.bluef - this.redf ) / chroma) + 2.0;
      } else {
        this.hue = (( this.redf - this.greenf ) / chroma) + 4.0;
      }
      this.hue *= 60.0;

      if (0 < major) {
        this.sat = chroma / major;
      } else {
        this.sat = 0;
      }
    } else {
      this.hue = 0;
      this.sat = 0;
    }
  }
}

/** Put 256 boxes in Three.js scene, one for each RGB332 color and arranged
    in a HSV color cylinder. */
function addColors() {
  // Box geometry object shared by all colorful boxes
  const boxGeometry = new THREE.BoxGeometry();

  // Adjust these values to increase/decrease density of colors in HSV cylinder
  const satScale = 15;
  const valScale = 10;

  // Adjust these values to increase/decrease density of colors in RGB rectangle
  const xScale = -25;
  const yScale = -25;
  const zScale = 10;

  // Adjust these values to tune HSV <-> RGB animation
  const animDuration = 1.0;
  const animTimes = [0, animDuration];
  const scratchQuaternion = new THREE.Quaternion();
  scratchQuaternion.identity();
  const identityQuaternionArray = scratchQuaternion.toArray();
  const axisZ = new THREE.Vector3(0, 0, 1);

  toRGBanim = new Array();
  toHSVanim = new Array();
  animMixers = new Array();

  // Map used to find color from its representative Mesh
  colorMap = new Map();

  // Group for all the boxes, handy to manipulate entire cylinder at once.
  hsvCyl = new THREE.Group();

  // Add 256 boxes, one for each color.
  for(var i = 0; i <= 0xFF; i++) {
    var nowColor = new RGB332(i);

    // Create box to represent color
    var nowMat = new THREE.MeshBasicMaterial( { color: nowColor.color24 } );
    var nowCube = new THREE.Mesh( boxGeometry, nowMat );
    var xHSV = 0;
    var yHSV = nowColor.sat * satScale;
    var zHSV = (nowColor.val-0.5) * valScale;

    var xRGB = (nowColor.redf-1) * xScale;
    var yRGB = (nowColor.greenf-1) * yScale;
    var zRGB = (nowColor.bluef-0.5) * zScale;

    nowCube.position.y = yHSV;
    nowCube.position.z = zHSV;

    // Create RGB <-> HSV animation objects for box
    var cubeMixer = new THREE.AnimationMixer(nowCube);
    animMixers.push(cubeMixer);

    if(nowColor.hue >= 0 && nowColor.hue < 180) {
      // HACK: I clearly don't understand quaternions
      xRGB *= -1;
      yRGB *= -1;
    }

    var cubeToRGBTrack = new THREE.VectorKeyframeTrack(
      ".position",
      animTimes,
      [xHSV, yHSV, zHSV, xRGB, yRGB, zRGB]);
    var cubeToRGBClip = new THREE.AnimationClip(
      "CubeToRGB"+i,
      -1,
      [cubeToRGBTrack]);
    var cubeToRGB = cubeMixer.clipAction(cubeToRGBClip);
    cubeToRGB.setLoop(THREE.LoopOnce);
    cubeToRGB.clampWhenFinished = true;
    toRGBanim.push(cubeToRGB);

    var cubeToHSVTrack = new THREE.VectorKeyframeTrack(
      ".position",
      animTimes,
      [xRGB, yRGB, zRGB, xHSV, yHSV, zHSV]);
    var cubeToHSVClip = new THREE.AnimationClip(
      "CubeToHSV"+i,
      -1,
      [cubeToHSVTrack]);
    var cubeToHSV = cubeMixer.clipAction(cubeToHSVClip);
    cubeToHSV.setLoop(THREE.LoopOnce);
    cubeToHSV.clampWhenFinished = true;
    toHSVanim.push(cubeToHSV);

    // Create rotor for HSV cylinder
    var rotor = new THREE.Group();

    rotor.add(nowCube);
    // HACK: The +0.1 tilts the 180 degree items in the direction I want for animation
    rotor.rotateZ(2*Math.PI*(nowColor.hue+0.1)/360);
    var hsvRotorQuaternionArray = rotor.quaternion.toArray();

    var rgbRotorQuaterionArray = identityQuaternionArray;
    if(nowColor.hue >= 0 && nowColor.hue < 180) {
      // HACK: I clearly don't understand quaternions
      scratchQuaternion.identity();
      scratchQuaternion.setFromAxisAngle(axisZ, Math.PI);
      rgbRotorQuaterionArray = scratchQuaternion.toArray();;
    }

    // Create RGB <-> HSV animation objects
    var rotorMixer = new THREE.AnimationMixer(rotor);
    animMixers.push(rotorMixer);

    var rotorToRGBTrack = new THREE.QuaternionKeyframeTrack(
      ".quaternion",
      animTimes,
      hsvRotorQuaternionArray.concat(rgbRotorQuaterionArray));
    var rotorToRGBClip = new THREE.AnimationClip(
      "RotorToRGB"+i,
      -1,
      [rotorToRGBTrack]);
    var rotorToRGB = rotorMixer.clipAction(rotorToRGBClip);
    rotorToRGB.setLoop(THREE.LoopOnce);
    rotorToRGB.clampWhenFinished = true;
    toRGBanim.push(rotorToRGB);

    var rotorToHSVTrack = new THREE.QuaternionKeyframeTrack(
      ".quaternion",
      animTimes,
      rgbRotorQuaterionArray.concat(hsvRotorQuaternionArray));
    var rotorToHSVClip = new THREE.AnimationClip(
      "RotorToHSV"+i,
      -1,
      [rotorToHSVTrack]);
    var rotorToHSV = rotorMixer.clipAction(rotorToHSVClip);
    rotorToHSV.setLoop(THREE.LoopOnce);
    rotorToHSV.clampWhenFinished = true;
    toHSVanim.push(rotorToHSV);

    hsvCyl.add(rotor);

    colorMap.set(nowCube, nowColor);
  }

  hsvCyl.rotateZ(-Math.PI/2);
  var rgbQuatArray = hsvCyl.quaternion.toArray();
  hsvCyl.rotateZ(+Math.PI/2);

  hsvCyl.rotateZ(-Math.PI/3);
  // Create RGB <-> HSV animation objects
  var cylMixer = new THREE.AnimationMixer(hsvCyl);
  animMixers.push(cylMixer);

  var cylToRGBTrackQ = new THREE.QuaternionKeyframeTrack(
    ".quaternion",
    animTimes,
    hsvCyl.quaternion.toArray().concat(rgbQuatArray));
  var cylToRGBTrackV = new THREE.VectorKeyframeTrack(
    ".position",
    animTimes,
    [0, 0, 0, xScale/2, -yScale/2, 0]);
  var hsvCylToRGBClip = new THREE.AnimationClip(
    "CylinToRGB",
    -1,
    [cylToRGBTrackQ, cylToRGBTrackV]);
  var cylToRGB = cylMixer.clipAction(hsvCylToRGBClip);
  cylToRGB.setLoop(THREE.LoopOnce);
  cylToRGB.clampWhenFinished = true;
  toRGBanim.push(cylToRGB);

  var cylToHSVTrackV = new THREE.QuaternionKeyframeTrack(
    ".quaternion",
    animTimes,
    rgbQuatArray.concat(hsvCyl.quaternion.toArray()));
  var cylToRGBTrackV = new THREE.VectorKeyframeTrack(
    ".position",
    animTimes,
    [xScale/2, -yScale/2, 0, 0, 0, 0]);
  var hsvCylToHSVClip = new THREE.AnimationClip(
    "CylinToHSV",
    -1,
    [cylToHSVTrackV, cylToRGBTrackV]);
  var cylToHSV = cylMixer.clipAction(hsvCylToHSVClip);
  cylToHSV.setLoop(THREE.LoopOnce);
  cylToHSV.clampWhenFinished = true;
  toHSVanim.push(cylToHSV);

  scene.add(hsvCyl);
}

/** Callback to update rendering based on user navigation handled by orbitControl */
function animate() {
  orbitControl.update();
  var clockDelta = clock.getDelta();
  for(var i = 0; i < animMixers.length; i++) {
    animMixers[i].update(clockDelta);
  }
  renderer.render( scene, camera );
}

/** Launch upon HTML DOM content load */
function contentLoaded() {
  begin();
  addColors();
}

/** Switch color model */
function switchColorModel() {
  button.disabled = true;
  window.setTimeout(switchEnable, 1000);
  for(var i = 0; i < toRGBanim.length; i++) {
    if (isHSV) {
      toHSVanim[i].stop();
      toRGBanim[i].play();
    } else {
      toRGBanim[i].stop();
      toHSVanim[i].play();
    }
  }
}

function switchEnable() {
  if (isHSV) {
    isHSV = false;
    button.textContent = "Switch to HSV";
  } else {
    isHSV = true;
    button.textContent = "Switch to RGB";
  }
  button.disabled = false;
}

//////////////////////////////////////////////////////////////////////////
//
//  Page load setup

document.addEventListener('DOMContentLoaded', contentLoaded, false);
document.defaultView.addEventListener('resize', resizeView);
