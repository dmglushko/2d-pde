// The viewer (tools/mesh_viewer/mesh.js, unchanged from the previous code)
// reads a .pde2d file written by the new exporter: a plate with a hole,
// version 5 with two boundary loops, every edge tagged with its region.
const fs = require('fs'), path = require('path'), vm = require('vm');
const assert = require('assert/strict');
const context = {window: {}};
vm.createContext(context);
vm.runInContext(fs.readFileSync(path.join(__dirname,
    '../../tools/mesh_viewer/mesh.js'), 'utf8'), context);
const {parseMesh, analyse} = context.window.MeshTools;
const source = fs.readFileSync(process.argv[2], 'utf8');
assert.ok(source.startsWith('PDE2D_MESH 5\n'), 'version 5');
const mesh = parseMesh(source), analysis = analyse(mesh);
assert.deepEqual([...analysis.issues], [], 'no structural issues');
assert.equal(analysis.summary.boundaryLoops, 2, 'outer loop and hole');
assert.equal(mesh.boundaryLoops.length, 2);
assert.equal(analysis.summary.inverted, 0);
assert.equal(analysis.summary.degenerate, 0);
const regions = new Set(mesh.boundaryEdges.map(e => e[2]));
assert.deepEqual([...regions].sort(), ['hole', 'inlet', 'outlet', 'wall']);
// The outer loop comes first: its edges are not on the hole.
const outer = mesh.boundaryEdges.slice(0, mesh.boundaryLoops[0]);
assert.ok(outer.every(e => e[2] !== 'hole'), 'outer loop first');
assert.equal(mesh.options, undefined, 'no result trailer (RESULT 0)');
console.log('PASS viewer reads the exported plate with a hole');
