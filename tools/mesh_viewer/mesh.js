// PDE2D_MESH 1 reader and corner-triangle diagnostics. No external packages.
function parseMesh(source) {
  const tokens = source.trim().split(/\s+/);
  let at = 0;
  const take = () => {
    if (at >= tokens.length) throw new Error('Unexpected end of file');
    return tokens[at++];
  };
  const expect = word => { if (take() !== word) throw new Error(`Expected ${word}`); };
  const remaining = (count, name) => {
    if (count > tokens.length-at) throw new Error(`Truncated ${name} section`);
  };
  const integer = (name, max = 50_000_000) => {
    const value = take();
    if (!/^(0|[1-9]\d*)$/.test(value) || !Number.isSafeInteger(Number(value)) || Number(value) > max)
      throw new Error(`Invalid ${name}: ${value}`);
    return Number(value);
  };
  const finite = name => {
    const value = take();
    // Number('') and Number('Infinity') must not silently become coordinates.
    if (!/^[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?$/.test(value) || !Number.isFinite(Number(value)))
      throw new Error(`Invalid ${name}: ${value}`);
    return Number(value);
  };
  expect('PDE2D_MESH');
  if (integer('version') !== 1) throw new Error('Only PDE2D_MESH version 1 is supported');
  expect('ORDER');
  const order = integer('order');
  if (order !== 1 && order !== 2) throw new Error('Order must be 1 or 2');
  expect('NODES');
  const nodeCount = integer('node count');
  if (!nodeCount) throw new Error('Mesh has no nodes');
  remaining(2*nodeCount, 'NODES');
  const nodes = Array.from({length: nodeCount}, () => [finite('x coordinate'), finite('y coordinate')]);
  expect('ELEMENTS');
  const elementCount = integer('element count');
  if (!elementCount) throw new Error('Mesh has no elements');
  remaining((order===1?4:7)*elementCount, 'ELEMENTS');
  const elements = Array.from({length: elementCount}, (_, id) => {
    const length = integer('element node count', 6);
    if (length !== (order === 1 ? 3 : 6)) throw new Error(`Element ${id}: expected ${order === 1 ? 3 : 6} nodes`);
    return Array.from({length}, () => {
      const index = integer('node index');
      if (index >= nodeCount) throw new Error(`Element ${id}: node ${index} is outside NODES`);
      return index;
    });
  });
  expect('BOUNDARY_EDGES');
  const boundaryCount = integer('boundary edge count');
  remaining(2*boundaryCount, 'BOUNDARY_EDGES');
  const boundaryEdges = Array.from({length: boundaryCount}, (_, id) => {
    const a = integer('boundary endpoint'), b = integer('boundary endpoint');
    if (a >= nodeCount || b >= nodeCount) throw new Error(`Boundary edge ${id}: endpoint outside NODES`);
    return [a, b];
  });
  if (at !== tokens.length) throw new Error(`Unexpected trailing token: ${tokens[at]}`);
  return {order, nodes, elements, boundaryEdges};
}

const key = (a, b) => a < b ? `${a},${b}` : `${b},${a}`;
const distance = (a, b) => Math.hypot(a[0]-b[0], a[1]-b[1]);
const format = n => Number.isFinite(n) ? Number(n.toPrecision(6)).toString() : 'undefined';

function analyse(mesh) {
  const {nodes, elements, boundaryEdges, order} = mesh;
  const bounds = [Infinity, Infinity, -Infinity, -Infinity];
  for (const [x,y] of nodes) {
    bounds[0]=Math.min(bounds[0],x);bounds[1]=Math.min(bounds[1],y);
    bounds[2]=Math.max(bounds[2],x);bounds[3]=Math.max(bounds[3],y);
  }
  const span = Math.max(bounds[2]-bounds[0], bounds[3]-bounds[1]);
  if (!Number.isFinite(span) || !Number.isFinite(span*span))
    throw new Error('Coordinate range exceeds supported numeric scale');
  const areaTolerance = 1e-14 * span * span; // matches mesh_io.hpp corner-area rule
  const edges = new Map(), recorded = new Map(), issues = [];
  const problemEdges = new Set(), problemTriangles = new Set(), triangleKeys = new Map();
  const triangles = elements.map((e, id) => {
    const [a,b,c] = e.map(index => nodes[index]);
    const cross = (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0]);
    const area = cross / 2;
    const lengths = [distance(a,b), distance(b,c), distance(c,a)];
    const sumSquares = lengths.reduce((sum, v) => sum+v*v, 0);
    if (!Number.isFinite(cross) || !Number.isFinite(sumSquares))
      throw new Error(`Triangle ${id}: geometry exceeds supported numeric scale`);
    const quality = sumSquares ? 2*Math.sqrt(3)*Math.abs(cross)/sumSquares : 0;
    const longest = Math.max(...lengths), shortest = Math.min(...lengths);
    const aspect = shortest > 0 ? longest / shortest : Infinity;
    const angle = (u,v,w) => {
      const x = nodes[u], y = nodes[v], z = nodes[w];
      const ux=x[0]-y[0], uy=x[1]-y[1], vx=z[0]-y[0], vy=z[1]-y[1];
      return Math.atan2(Math.abs(ux*vy-uy*vx), ux*vx+uy*vy)*180/Math.PI;
    };
    const angles = [angle(e[2],e[0],e[1]),angle(e[0],e[1],e[2]),angle(e[1],e[2],e[0])];
    const status = cross <= areaTolerance ? (cross < -areaTolerance ? 'inverted' : 'degenerate') : 'valid';
    if (status !== 'valid') issues.push(`Triangle ${id} is ${status}`);
    if (new Set(e).size !== e.length) {issues.push(`Triangle ${id} repeats a node index`);problemTriangles.add(id);}
    const triangleKey=e.slice(0,3).sort((a,b)=>a-b).join(',');
    if (triangleKeys.has(triangleKey)) {
      const previous=triangleKeys.get(triangleKey);
      issues.push(`Triangle ${id} duplicates triangle ${previous}`);
      problemTriangles.add(id);problemTriangles.add(previous);
    } else triangleKeys.set(triangleKey,id);
    const edgeKeys = [key(e[0],e[1]),key(e[1],e[2]),key(e[2],e[0])];
    edgeKeys.forEach((k, local) => {
      if (!edges.has(k)) edges.set(k,{nodes: [e[local],e[(local+1)%3]], owners: [], mids: [], directions: []});
      const record = edges.get(k); record.owners.push(id);
      record.directions.push(e[local] < e[(local+1)%3] ? 1 : -1);
      if (order === 2) record.mids.push(e[3+local]);
    });
    return {id, nodes:e, area, lengths, quality, aspect, angles, status, edgeKeys};
  });
  boundaryEdges.forEach(([a,b], id) => {
    const k=key(a,b);
    if (a===b) {issues.push(`Boundary edge ${id} has the same endpoint twice`);problemEdges.add(k);}
    if (recorded.has(k)) {issues.push(`Boundary edge ${id} duplicates edge ${recorded.get(k)}`);problemEdges.add(k);}
    recorded.set(k,id);
  });
  for (const [k, edge] of edges) {
    if (edge.owners.length>2) {issues.push(`Edge ${k} is non-manifold (${edge.owners.length} triangles)`);problemEdges.add(k);}
    if ((edge.owners.length===1)!==recorded.has(k)) {issues.push(`Edge ${k}: recorded boundary disagrees with triangle ownership`);problemEdges.add(k);}
    if (edge.owners.length===2 && edge.directions[0]===edge.directions[1]) {
      issues.push(`Edge ${k}: adjacent triangles traverse the shared edge in the same direction`);problemEdges.add(k);
    }
    if (order===2 && new Set(edge.mids).size>1) {issues.push(`Edge ${k}: P2 midpoint indices disagree`);problemEdges.add(k);}
    if (order===2 && edge.mids.length) {
      const [u,v]=edge.nodes;
      for (const m of new Set(edge.mids)) {
        const p=nodes[m];
        const expected=[(nodes[u][0]+nodes[v][0])/2,(nodes[u][1]+nodes[v][1])/2];
        if (distance(p,expected)>1e-10*Math.max(1,span)) {issues.push(`Edge ${k}: P2 node ${m} is away from straight midpoint`);problemEdges.add(k);}
      }
    }
  }
  for (const [k] of recorded) if (!edges.has(k)) {issues.push(`Recorded boundary edge ${k} has no triangle`);problemEdges.add(k);}
  const degree = new Map();
  boundaryEdges.forEach(([a,b]) => {degree.set(a,(degree.get(a)||0)+1);degree.set(b,(degree.get(b)||0)+1);});
  for (const [id,n] of degree) if (n!==2) {
    issues.push(`Boundary node ${id} has degree ${n}, expected 2`);
    for (const [a,b] of boundaryEdges) if (a===id || b===id) problemEdges.add(key(a,b));
  }
  const used = new Set(elements.flat());
  const unused = nodes.length-used.size;
  if (unused) issues.push(`${unused} node(s) are unused by elements`);
  const valid = triangles.filter(t=>t.status==='valid');
  const stats = values => {
    if (!values.length) return null;
    let min=Infinity,max=-Infinity,sum=0;
    for (const value of values) {min=Math.min(min,value);max=Math.max(max,value);sum+=value;}
    return {min,mean:sum/values.length,max};
  };
  return {bounds, triangles, edges, recorded, problemEdges, problemTriangles, issues, areaTolerance,
    summary: {nodes:nodes.length, elements:elements.length, boundaryEdges:boundaryEdges.length,
      inverted:triangles.filter(t=>t.status==='inverted').length,
      degenerate:triangles.filter(t=>t.status==='degenerate').length,
      quality:stats(valid.map(t=>t.quality)), minAngle:stats(valid.map(t=>Math.min(...t.angles)))}};
}
window.MeshTools = {parseMesh, analyse, format};
