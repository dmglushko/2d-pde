// Dependency-free checks. The DOM mock tests behavior, not browser rendering.
const fs=require('fs'),path=require('path'),vm=require('vm'),assert=require('assert/strict');
const here=__dirname, examples=path.resolve(here,'../../examples/mesh_viewer');
const c={window:{}};vm.createContext(c);
vm.runInContext(fs.readFileSync(path.join(here,'mesh.js'),'utf8'),c);
const {parseMesh,analyse}=c.window.MeshTools;
for(const [order,nodes] of [[1,289],[2,1089]]) {
  const source=fs.readFileSync(path.join(examples,`rectangle_p${order}.pde2d`),'utf8');
  const mesh=parseMesh(source),a=analyse(mesh);
  assert.equal(mesh.nodes.length,nodes);assert.equal(mesh.elements.length,512);assert.equal(a.issues.length,0);
  for(const bad of [source.slice(0,-25),source+' EXTRA',source.replace('PDE2D_MESH 1','PDE2D_MESH 2'),source.replace(`ORDER ${order}`,'ORDER 3'),source.replace(/(NODES \d+\s+)\S+/,'$1NaN'),source.replace(/(ELEMENTS \d+\s+)([36]) \d+/,'$1$2 999999')]) assert.throws(()=>parseMesh(bad));
}
assert.throws(()=>parseMesh('PDE2D_MESH 1 ORDER 1 NODES 50000000'));
const base=()=>({order:1,nodes:[[0,0],[1,0],[0,1]],elements:[[0,1,2]],boundaryEdges:[[0,1],[1,2],[2,0]]});
let m=base(),a=analyse(m);assert.equal(a.issues.length,0);assert.ok(Math.abs(a.triangles[0].quality-Math.sqrt(3)/2)<1e-14);
m=base();m.nodes[2]=[0.5,Math.sqrt(3)/2];assert.ok(Math.abs(analyse(m).triangles[0].quality-1)<1e-14);
m=base();m.elements[0]=[0,2,1];assert.equal(analyse(m).summary.inverted,1);
m=base();m.nodes[2]=[2,0];assert.equal(analyse(m).summary.degenerate,1);
m=base();m.elements.push([0,1,2]);a=analyse(m);assert.ok(a.issues.some(s=>s.includes('duplicates triangle')));assert.equal(a.problemTriangles.size,2);
m=base();m.boundaryEdges.push([0,1]);assert.ok(analyse(m).problemEdges.has('0,1'));
m=base();m.boundaryEdges.pop();assert.ok(analyse(m).issues.some(s=>s.includes('degree')));
m=base();m.nodes.push([1,-1]);m.elements.push([1,0,3],[0,1,3]);assert.ok(analyse(m).issues.some(s=>s.includes('non-manifold')));
m=base();m.order=2;m.nodes.push([.5,0],[.5,.5],[0,.5]);m.elements=[[0,1,2,3,4,5]];assert.equal(analyse(m).issues.length,0);
m.nodes[3]=[.5,.1];assert.ok(analyse(m).problemEdges.has('0,1'));
m=base();m.nodes[1]=[1e308,0];assert.throws(()=>analyse(m),/numeric scale/);
console.log('Real P1/P2 exports, malformed input, quality values, and defect diagnostics passed');

function boot(source) {
  const listeners={},elements={};
  for(const id of ['canvas','dropzone','error','filename','mesh-title','overview','inspector','diagnostics','scale','fit','open','file','wire','boundary','nodes','triangles','p2nodes'])
    elements[id]={checked:['wire','boundary','p2nodes'].includes(id),hidden:false,innerHTML:'',textContent:'',classList:{add(){},remove(){}},addEventListener(name,fn){listeners[id+':'+name]=fn;},getBoundingClientRect(){return {width:800,height:600}}};
  elements.canvas.setPointerCapture=()=>{};
  elements.canvas.getContext=()=>({setTransform(){},clearRect(){},beginPath(){},lineTo(){},moveTo(){},closePath(){},fill(){},stroke(){},arc(){},fillText(){}});
  const context={window:{devicePixelRatio:1,exampleMesh:source},document:{getElementById:id=>elements[id]},ResizeObserver:class{observe(){}}};
  vm.createContext(context);
  for(const file of ['mesh.js','app.js'])vm.runInContext(fs.readFileSync(path.join(here,file),'utf8'),context);
  const event=(type,x=400,y=300)=>listeners['canvas:'+type]({pointerId:1,offsetX:x,offsetY:y});
  return {listeners,elements,event};
}
let ui=boot(fs.readFileSync(path.join(examples,'rectangle_p2.pde2d'),'utf8'));
assert.equal(ui.elements.error.hidden,true);
ui.event('pointerdown');ui.event('pointerup');assert.match(ui.elements.inspector.innerHTML,/Triangle \d+/);
const selected=ui.elements.inspector.innerHTML;
ui.event('pointerdown');for(let x=401;x<=410;x++)ui.event('pointermove',x,300);ui.event('pointerup',410,300);assert.equal(ui.elements.inspector.innerHTML,selected);
ui.event('pointerdown');ui.event('pointercancel');const scale=ui.elements.scale.textContent;ui.event('pointermove',600,300);assert.equal(ui.elements.scale.textContent,scale);
ui.listeners['canvas:wheel']({offsetX:400,offsetY:300,deltaY:-100,preventDefault(){}});assert.notEqual(ui.elements.scale.textContent,scale);
ui.elements.nodes.checked=true;ui.listeners['nodes:change']();
ui=boot('PDE2D_MESH 1 ORDER 1 NODES 3 0 0 1 0 2 0 ELEMENTS 1 3 0 1 2 BOUNDARY_EDGES 3 0 1 1 2 2 0');
ui.event('pointerdown');ui.event('pointerup');assert.match(ui.elements.inspector.innerHTML,/degenerate/);
console.log('Startup, selection, slow pan, pointer cancellation, zoom, node redraw, and collapsed selection passed');
// Invalid uploads must leave the last displayed mesh intact.
(async()=>{
  const before=ui.elements.overview.innerHTML;
  await ui.listeners['dropzone:drop']({preventDefault(){},dataTransfer:{files:[{name:'broken.pde2d',text:async()=> 'PDE2D_MESH 1 ORDER 1 NODES 50000000'}]}});
  assert.equal(ui.elements.overview.innerHTML,before);
  assert.equal(ui.elements.error.hidden,false);
  assert.match(ui.elements.error.textContent,/Truncated NODES/);
  console.log('Malformed upload preserves the displayed mesh');
})().catch(error=>{console.error(error);process.exitCode=1;});
