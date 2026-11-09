// Version 5 files (meshes with holes) exported by tests/discretization/holes.cpp
// load in the viewer parser; malformed loop records are rejected.
const fs=require('fs'),path=require('path'),vm=require('vm'),assert=require('assert/strict');
const context={window:{}};vm.createContext(context);
vm.runInContext(fs.readFileSync(path.join(__dirname,'../../tools/mesh_viewer/mesh.js'),'utf8'),context);
const {parseMesh,analyse}=context.window.MeshTools;
const dir=process.argv[2], read=name=>fs.readFileSync(path.join(dir,name),'utf8');
for(const name of ['annulus_result.pde2d','annulus_again.pde2d','annulus_refined.pde2d','shuffled.pde2d']) {
  const mesh=parseMesh(read(name)), analysis=analyse(mesh);
  assert.equal(analysis.issues.length,0,name);
  assert.equal(analysis.summary.boundaryLoops,2,name);
  assert.equal(mesh.boundaryLoops.length,2,name);
  assert.equal(mesh.boundaryLoops[0]+mesh.boundaryLoops[1],mesh.boundaryEdges.length);
  // The outer loop comes first and carries its own region.
  assert.equal(mesh.boundaryEdges[0][2],'outer');
  assert.equal(mesh.boundaryEdges.at(-1)[2],'inner');
  assert.ok(mesh.options && mesh.report.delaunayVerified);
  assert.equal(mesh.boundaryContext.polygonLoops.length,2);
  assert.equal(mesh.boundaryContext.polygonLoops.reduce((a,b)=>a+b),mesh.boundaryContext.polygonSnapshot.length);
}
const raw=read('framed_raw.pde2d'), framed=parseMesh(raw);
assert.deepEqual([...framed.boundaryLoops],[4,4]);
assert.equal(framed.options,undefined);
assert.equal(analyse(framed).summary.boundaryLoops,2);
const plain=parseMesh(read('plain.pde2d'));
assert.equal(plain.boundaryLoops,undefined);
assert.equal(analyse(plain).summary.boundaryLoops,1);
const annulus=read('annulus_result.pde2d');
for(const bad of [
  raw.replace('4\n4\nRESULT','5\n3\nRESULT'),   // groups that are not loops
  raw.replace('4\n4\nRESULT','4\n5\nRESULT'),   // sizes do not add up
  raw.replace('4\n4\nRESULT','6\n2\nRESULT'),   // a loop of two edges
  raw.replace('BOUNDARY_LOOPS 2','BOUNDARY_LOOPS 0'),
  raw.replace('RESULT 0','RESULT 2'),
  raw.replace('RESULT 0\n',''),
  raw.replace('PDE2D_MESH 5','PDE2D_MESH 6'),
  annulus.replace(/POLYGON_LOOPS 2\n(\d+)/,(m,n)=>`POLYGON_LOOPS 2\n${Number(n)+1}`),
  annulus.replace('POLYGON_LOOPS','POLYGON_LOOP'),
]) assert.throws(()=>parseMesh(bad));
console.log('C++ version 5 exports (holes) parse with two boundary loops; malformed loop records rejected');
