import * as THREE from 'three';
import {OrbitControls} from './assets/three/OrbitControls.js';
import {PLYLoader} from './assets/three/PLYLoader.js';

export async function showPlaster(meta) {
  const status = document.querySelector('#plasterStatus');
  const fallback = document.querySelector('#plaster');
  const host = document.querySelector('#plaster3d');
  const toolbar = document.querySelector('#plasterTools');
  function useImage(message) {
    host.hidden = true;
    toolbar.hidden = true;
    if (meta.plaster_available) {
      fallback.src = '/plaster.png';
      fallback.hidden = false;
      fallback.style.display = 'block';
      status.textContent = message + '，显示静态图片备用预览。';
    } else {
      status.textContent = message + '。请先执行 02 export --plaster 生成石膏网格。';
    }
  }
  if (!meta.surface_available) {
    useImage('未找到 surface.ply 三维网格');
    return;
  }
  let renderer;
  try {
    status.textContent = '正在加载 surface.ply 三维网格…';
    renderer = new THREE.WebGLRenderer({antialias: true});
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    renderer.setClearColor(0x26292c);
    host.hidden = false;
    renderer.domElement.style.width = '100%';
    renderer.domElement.style.height = '100%';
    host.appendChild(renderer.domElement);
    renderer.domElement.setAttribute('aria-label', '石膏三维网格，左键旋转、右键平移、滚轮缩放');
    const scene = new THREE.Scene();
    const camera = new THREE.PerspectiveCamera(45, 1, .01, 1000);
    camera.up.set(0, 0, 1);
    const controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    const geometry = await new PLYLoader().loadAsync('/surface.ply');
    if (!geometry.getAttribute('position')?.count) throw Error('网格顶点为空');
    geometry.computeVertexNormals();
    geometry.computeBoundingSphere();
    const {center, radius} = geometry.boundingSphere;
    if (!(radius > 0) || !Number.isFinite(radius)) throw Error('网格范围无效');
    scene.add(new THREE.Mesh(geometry, new THREE.MeshStandardMaterial({
      color: 0xeee6d8, roughness: .85, metalness: 0, side: THREE.DoubleSide,
    })));
    scene.add(new THREE.HemisphereLight(0xffffff, 0x565045, 2.5));
    const light = new THREE.DirectionalLight(0xffffff, 3);
    light.position.set(1, -2, 4);
    scene.add(light);
    controls.target.copy(center);
    controls.minDistance = radius * .02;
    controls.maxDistance = radius * 12;
    camera.near = Math.max(.001, radius / 1000);
    camera.far = radius * 100;
    let direction = new THREE.Vector3(1, -1, 1.1).normalize();
    function reset(view) {
      if (view === 'top') direction.set(.0001, 0, 1).normalize();
      else direction.set(1, -1, 1.1).normalize();
      controls.target.copy(center);
      const aspect = host.clientWidth / Math.max(host.clientHeight, 1);
      const verticalFov = THREE.MathUtils.degToRad(camera.fov);
      const limitingFov = Math.min(verticalFov, 2 * Math.atan(Math.tan(verticalFov / 2) * aspect));
      camera.position.copy(center).addScaledVector(direction, radius / Math.sin(limitingFov / 2) * 1.1);
      controls.update();
    }
    function resize() {
      const width = host.clientWidth, height = host.clientHeight;
      renderer.setSize(width, height, false);
      camera.aspect = width / height;
      camera.updateProjectionMatrix();
    }
    const observer = new ResizeObserver(resize);
    observer.observe(host);
    resize();
    reset('perspective');
    document.querySelector('#plasterReset').onclick = () => reset('perspective');
    document.querySelector('#plasterTop').onclick = () => reset('top');
    toolbar.hidden = false;
    fallback.hidden = true;
    fallback.style.display = 'none';
    const triangles = (geometry.index ? geometry.index.count : geometry.attributes.position.count) / 3;
    status.textContent = `三维网格已加载 · ${triangles.toLocaleString()} 个三角面 · 左键旋转 / 右键平移 / 滚轮缩放`;
    renderer.domElement.addEventListener('webglcontextlost', event => {
      event.preventDefault();
      renderer.setAnimationLoop(null);
      useImage('三维显示上下文丢失，请刷新页面重试');
    });
    renderer.setAnimationLoop(() => {controls.update(); renderer.render(scene, camera);});
    window.addEventListener('pagehide', () => {
      renderer.setAnimationLoop(null);
      observer.disconnect(); controls.dispose(); geometry.dispose(); renderer.dispose();
    }, {once:true});
  } catch (error) {
    renderer?.dispose();
    useImage('三维预览加载失败：' + error.message);
  }
}
