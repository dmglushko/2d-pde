const fs=require('fs'),path=require('path'),vm=require('vm'),assert=require('assert/strict');
const context={window:{}};vm.createContext(context);
vm.runInContext(fs.readFileSync(path.join(__dirname,'mesh_viewer/mesh.js'),'utf8'),context);
const files=fs.readdirSync(process.argv[2]).filter(name=>name.endsWith('.pde2d'));
assert.equal(files.length,19);
for(const name of files) {
  const mesh=context.window.MeshTools.parseMesh(fs.readFileSync(path.join(process.argv[2],name),'utf8'));
  assert.ok(mesh.options);assert.ok(mesh.boundaryContext);
  assert.equal(context.window.MeshTools.analyse(mesh).issues.length,0,name);
}
console.log('19 fresh C++ showcase exports load without structural issues');
