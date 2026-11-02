// Dependency-free checks. The DOM mock tests behavior, not browser rendering.
const fs=require('fs'),path=require('path'),vm=require('vm'),assert=require('assert/strict');
const here=__dirname, examples=path.resolve(here,'../../examples/mesh_viewer');
const c={window:{}};vm.createContext(c);
vm.runInContext(fs.readFileSync(path.join(here,'mesh.js'),'utf8'),c);
const {parseMesh,analyse}=c.window.MeshTools;
const source=fs.readFileSync(path.join(examples,'rectangle_p1.pde2d'),'utf8');
const mesh=parseMesh(source),analysis=analyse(mesh);
assert.equal(mesh.order,undefined);assert.equal(mesh.nodes.length,289);assert.equal(mesh.elements.length,512);assert.equal(analysis.issues.length,0);
for(const bad of [source.slice(0,-25),source+' EXTRA',source.replace('PDE2D_MESH 1','PDE2D_MESH 3'),source.replace('ORDER 1','ORDER 2'),source.replace('ORDER 1','ORDER 3'),source.replace(/(NODES \d+\s+)\S+/,'$1NaN'),source.replace('ELEMENTS 512\n3 ','ELEMENTS 512\n6 '),source.replace(/(ELEMENTS \d+\s+3 )\d+/,'$1 999999')]) assert.throws(()=>parseMesh(bad));
assert.throws(()=>parseMesh('PDE2D_MESH 1 ORDER 1 NODES 50000000'));

const v2='PDE2D_MESH 2 ORDER 1 NODES 3 0 0 1 0 0 1 ELEMENTS 1 3 0 1 2 BOUNDARY_EDGES 3 0 1 inlet.1 1 2 - 2 0 wall_A-2';
const tagged=parseMesh(v2);
assert.deepEqual(JSON.parse(JSON.stringify(tagged.boundaryEdges)), [[0,1,'inlet.1'],[1,2,null],[2,0,'wall_A-2']]);
assert.equal(analyse(tagged).issues.length,0);
assert.throws(()=>parseMesh(v2.replace('inlet.1','bad/tag')),/invalid region/);
assert.throws(()=>parseMesh(v2.replace(' wall_A-2','')),/Truncated|Unexpected end/);
assert.throws(()=>parseMesh(v2+' extra'),/trailing/);
assert.equal(c.window.MeshTools.regionColor(null),'#e89b13');
assert.notEqual(c.window.MeshTools.regionColor('inlet.1'),c.window.MeshTools.regionColor('wall_A-2'));

// v3 fixtures serialize every persisted field and parse it back; no mesher assumptions.
const plain=value=>JSON.parse(JSON.stringify(value));
const result3={...plain(tagged),options:{algorithm:1,boundaryTreatment:2,preserveRegions:true,boundary:{maxEdgeLength:0.5,maxCurveDeviation:0.02},refinement:{maxEdgeLength:0.25,maxSteps:7}},
  report:{sizeSatisfied:false,delaunayVerified:true,refinementSteps:7,message:'Limit reached\nRésumé <&>'},
  boundaryContext:{protectedVertices:[0,1,2],polygonSnapshot:[[0,0],[1,0],[0,1]],curves:[
    {id:'18446744073709551615',region:'inlet.1',kind:0,values:[0,0,1,0]},
    {id:'2',region:null,kind:1,values:[1,0,0.75,0.75,0,1]},
    {id:'3',region:'wall_A-2',kind:2,values:[0,1,-0.1,0.8,-0.1,0.2,0,0]}],
    edgeSources:[[{curve:0,begin:0,end:0.5},{curve:0,begin:0.5,end:1}],[{curve:1,begin:0,end:1}],[{curve:2,begin:0,end:1}]]}};
function writeV3(r) {
  const o=r.options,c=r.boundaryContext,rep=r.report;
  return ['PDE2D_MESH 3 ORDER 1',`NODES ${r.nodes.length}`, ...r.nodes.map(p=>p.join(' ')),
    `ELEMENTS ${r.elements.length}`,...r.elements.map(e=>`3 ${e.join(' ')}`),`BOUNDARY_EDGES ${r.boundaryEdges.length}`,...r.boundaryEdges.map(([a,b,tag])=>`${a} ${b} ${tag??'-'}`),
    `OPTIONS ${o.algorithm} ${o.boundaryTreatment} ${+o.preserveRegions} ${o.boundary.maxEdgeLength} ${o.boundary.maxCurveDeviation} ${o.refinement.maxEdgeLength} ${o.refinement.maxSteps}`,
    `REPORT ${+rep.sizeSatisfied} ${+rep.delaunayVerified} ${rep.refinementSteps} ${Buffer.from(rep.message).toString('hex')||'-'}`,
    `CONTEXT ${+!!c}`,...(c?[`PROTECTED_VERTICES ${c.protectedVertices.length}`,c.protectedVertices.join(' '),`POLYGON_SNAPSHOT ${c.polygonSnapshot.length}`,...c.polygonSnapshot.map(p=>p.join(' ')),`SOURCE_CURVES ${c.curves.length}`,
      ...c.curves.map(s=>`${s.id} ${s.region??'-'} ${s.kind} ${s.values.join(' ')}`),`EDGE_SOURCES ${c.edgeSources.length}`,
      ...c.edgeSources.map(spans=>`${spans.length} ${spans.map(s=>`${s.curve} ${s.begin} ${s.end}`).join(' ')}`)]:[]),'END_RESULT'].join('\n');
}
const v3=writeV3(result3);
assert.deepEqual(plain(parseMesh(v3)),result3);
assert.equal(analyse(parseMesh(v3)).issues.length,0);
assert.deepEqual(plain(parseMesh(writeV3(parseMesh(v3)))),result3);
const noContext=plain(result3);noContext.boundaryContext=null;noContext.report.message='';
assert.deepEqual(plain(parseMesh(writeV3(noContext))),noContext);
// Ellipse source, reversed span, and multiple spans crossing source-curve joins.
const arc=plain(result3);arc.boundaryContext.curves[1]={id:'2',region:null,kind:3,values:[0,0,1,1,0,0,Math.PI/2]};
assert.deepEqual(plain(parseMesh(writeV3(arc))),arc);
const reversed=plain(result3);reversed.boundaryContext.curves[0].values=[1,0,0,0];reversed.boundaryContext.edgeSources[0]=[{curve:0,begin:1,end:0}];
assert.deepEqual(plain(parseMesh(writeV3(reversed))),reversed);
const joined=plain(result3);joined.boundaryContext.curves[0].values=[0,0,0.5,0];joined.boundaryContext.curves.push({id:'4',region:'inlet.1',kind:0,values:[0.5,0,1,0]});joined.boundaryContext.edgeSources[0]=[{curve:0,begin:0,end:1},{curve:3,begin:0,end:1}];
assert.deepEqual(plain(parseMesh(writeV3(joined))),joined);
for(const mutate of [
  r=>r.options.algorithm=2,r=>r.options.boundaryTreatment=3,r=>r.options.boundary.maxEdgeLength=-1,r=>r.options.boundary.maxCurveDeviation=-1,
  r=>r.options.refinement.maxEdgeLength=-1,r=>r.options.refinement.maxSteps=-1,
  r=>r.boundaryContext.protectedVertices=[0,0],r=>r.boundaryContext.protectedVertices=[3],
  r=>r.boundaryContext.curves[1].id=r.boundaryContext.curves[0].id,r=>r.boundaryContext.curves[0].id='18446744073709551616',
  r=>r.boundaryContext.curves[0].kind=4,r=>r.boundaryContext.curves[0].kind=99,
  r=>r.boundaryContext.curves[0].values[0]=NaN,r=>r.boundaryContext.curves[0].region='bad/tag',
  r=>r.boundaryContext.edgeSources.pop(),r=>r.boundaryContext.edgeSources[0]=[],r=>r.boundaryContext.edgeSources[0][0].curve=3,
  r=>r.boundaryContext.edgeSources[0][0].begin=-0.1,r=>r.boundaryContext.edgeSources[0][0].end=1.1,
  r=>r.boundaryContext.edgeSources[0][0].end=0,r=>r.boundaryContext.edgeSources[0][1].begin=0.4,
  r=>r.boundaryContext.curves[0].values[0]=0.1,r=>r.boundaryContext.curves[0].region='wrong',
]) {const r=plain(result3);mutate(r);assert.throws(()=>parseMesh(writeV3(r)));}
for(const bad of [v3.replace('REPORT 0 1','REPORT 2 1'),v3.replace(/REPORT[^\n]+/,'REPORT 0 1 7 abx0'),v3.replace('CONTEXT 1','CONTEXT 2'),v3.replace('END_RESULT',''),v3+' EXTRA',v3.slice(0,v3.indexOf('EDGE_SOURCES')+20)])assert.throws(()=>parseMesh(bad));
const badArc=plain(arc);badArc.boundaryContext.curves[1].values[2]=0;assert.throws(()=>parseMesh(writeV3(badArc)));
console.log('v3 metadata roundtrip, all curve types, ordered/reversed spans, and malformed provenance passed');

const base=()=>({nodes:[[0,0],[1,0],[0,1]],elements:[[0,1,2]],boundaryEdges:[[0,1],[1,2],[2,0]]});
let m=base(),a=analyse(m);assert.equal(a.issues.length,0);assert.ok(Math.abs(a.triangles[0].quality-Math.sqrt(3)/2)<1e-14);
m=base();m.nodes[2]=[0.5,Math.sqrt(3)/2];assert.ok(Math.abs(analyse(m).triangles[0].quality-1)<1e-14);
m=base();m.elements[0]=[0,2,1];assert.equal(analyse(m).summary.inverted,1);
m=base();m.nodes[2]=[2,0];assert.equal(analyse(m).summary.degenerate,1);
for(const height of [1e-15,Number.MIN_VALUE]) {
  m=base();m.nodes[2]=[1,height];assert.equal(analyse(m).triangles[0].status,'valid');
  m.elements[0]=[0,2,1];assert.equal(analyse(m).triangles[0].status,'inverted');
}
m=base();m.elements.push([0,1,2]);a=analyse(m);assert.ok(a.issues.some(s=>s.includes('duplicates triangle')));assert.equal(a.problemTriangles.size,2);
m=base();m.boundaryEdges.push([0,1]);assert.ok(analyse(m).problemEdges.has('0,1'));
m=base();m.boundaryEdges.pop();assert.ok(analyse(m).issues.some(s=>s.includes('degree')));
m=base();m.nodes.push([1,-1]);m.elements.push([1,0,3],[0,1,3]);assert.ok(analyse(m).issues.some(s=>s.includes('non-manifold')));
m=base();m.nodes[1]=[1e308,0];assert.throws(()=>analyse(m),/numeric scale/);
console.log('P1 export, malformed input, quality values, and defect diagnostics passed');

function boot(input) {
  const listeners={},elements={},strokes=[];
  for(const id of ['canvas','dropzone','error','filename','mesh-title','overview','inspector','diagnostics','settings','report','regions','scale','fit','open','file','wire','boundary','nodes','triangles'])
    elements[id]={checked:['wire','boundary'].includes(id),hidden:false,innerHTML:'',textContent:'',classList:{add(){},remove(){}},addEventListener(name,fn){listeners[id+':'+name]=fn;},getBoundingClientRect(){return {width:800,height:600}}};
  elements.canvas.setPointerCapture=()=>{};
  elements.canvas.getContext=()=>({setTransform(){},clearRect(){},beginPath(){},lineTo(){},moveTo(){},closePath(){},fill(){},stroke(){strokes.push(this.strokeStyle);},arc(){},fillText(){}});
  const context={window:{devicePixelRatio:1,exampleMesh:input},document:{getElementById:id=>elements[id]},ResizeObserver:class{observe(){}}};
  vm.createContext(context);
  for(const file of ['mesh.js','app.js'])vm.runInContext(fs.readFileSync(path.join(here,file),'utf8'),context);
  const event=(type,x=400,y=300)=>listeners['canvas:'+type]({pointerId:1,offsetX:x,offsetY:y});
  return {listeners,elements,event,strokes};
}
const embeddedContext={window:{}};vm.createContext(embeddedContext);
vm.runInContext(fs.readFileSync(path.join(examples,'example_mesh.js'),'utf8'),embeddedContext);
const nativeSource=fs.readFileSync(path.join(examples,'rectangle_native.pde2d'),'utf8');
assert.equal(embeddedContext.window.exampleMesh,nativeSource);
assert.equal(parseMesh(nativeSource).report.delaunayVerified,true);
let ui=boot(v2);
assert.match(ui.elements.regions.innerHTML,/inlet\.1/);
assert.match(ui.elements.regions.innerHTML,/untagged boundary/);
assert.match(ui.elements.regions.innerHTML,/wall_A-2/);
assert.ok(ui.strokes.includes(c.window.MeshTools.regionColor('inlet.1')));
assert.ok(ui.strokes.includes(c.window.MeshTools.regionColor('wall_A-2')));
assert.ok(ui.strokes.includes(c.window.MeshTools.regionColor(null)));
const v3ui=boot(v3);
assert.equal(v3ui.elements.error.hidden,true);
assert.match(v3ui.elements.settings.innerHTML,/Refine on curves/);
assert.match(v3ui.elements.report.innerHTML,/reported unsatisfied/);
assert.match(v3ui.elements.report.innerHTML,/reported verified/);
assert.match(v3ui.elements.report.innerHTML,/Résumé &lt;&amp;&gt;/);
assert.ok(v3ui.strokes.includes('#127b72'));
assert.match(v3ui.elements.diagnostics.innerHTML,/No structural issues/);
assert.match(ui.elements.report.innerHTML,/No mesher guarantees/);
const defective=boot(v2.replace('2 0 wall_A-2','0 1 wall_A-2'));
assert.ok(defective.strokes.includes('#a036ac'));
assert.match(defective.elements.diagnostics.innerHTML,/duplicates edge/);
assert.match(ui.elements.regions.innerHTML, new RegExp(c.window.MeshTools.regionColor('inlet.1').replace(/[()]/g,'\\$&')));
ui=boot(embeddedContext.window.exampleMesh);
assert.match(ui.elements.regions.innerHTML,/bottom/);
assert.equal(ui.elements.error.hidden,true);assert.equal(ui.elements['mesh-title'].textContent,'P1 triangular mesh');assert.match(ui.elements.filename.textContent,/P1/);
ui.event('pointerdown');ui.event('pointerup');assert.match(ui.elements.inspector.innerHTML,/Triangle \d+/);
const selected=ui.elements.inspector.innerHTML;
ui.event('pointerdown');for(let x=401;x<=410;x++)ui.event('pointermove',x,300);ui.event('pointerup',410,300);assert.equal(ui.elements.inspector.innerHTML,selected);
ui.event('pointerdown');ui.event('pointercancel');const scale=ui.elements.scale.textContent;ui.event('pointermove',600,300);assert.equal(ui.elements.scale.textContent,scale);
ui.listeners['canvas:wheel']({offsetX:400,offsetY:300,deltaY:-100,preventDefault(){}});assert.notEqual(ui.elements.scale.textContent,scale);
ui.elements.nodes.checked=true;ui.listeners['nodes:change']();
ui=boot('PDE2D_MESH 1 ORDER 1 NODES 3 0 0 1 0 2 0 ELEMENTS 1 3 0 1 2 BOUNDARY_EDGES 3 0 1 1 2 2 0');
ui.event('pointerdown');ui.event('pointerup');assert.match(ui.elements.inspector.innerHTML,/degenerate/);
console.log('P1 startup, selection, slow pan, pointer cancellation, zoom, node redraw, and collapsed selection passed');
// Invalid uploads must leave the last displayed mesh intact.
(async()=>{
  const before=ui.elements.overview.innerHTML;
  await ui.listeners['dropzone:drop']({preventDefault(){},dataTransfer:{files:[{name:'broken.pde2d',text:async()=> 'PDE2D_MESH 1 ORDER 1 NODES 50000000'}]}});
  assert.equal(ui.elements.overview.innerHTML,before);assert.equal(ui.elements.error.hidden,false);assert.match(ui.elements.error.textContent,/Truncated NODES/);
  console.log('Malformed upload preserves the displayed mesh');
})().catch(error=>{console.error(error);process.exitCode=1;});
