// Explicit PDE2D_MESH wire IDs; independent of C++ variant ordering.
const sourceCurveFormats = new Map([
  [0, {count:4, point:bezierSourcePoint}],
  [1, {count:6, point:bezierSourcePoint}],
  [2, {count:8, point:bezierSourcePoint}],
  [3, {count:7, validate:v=>v[2]>0 && v[3]>0 && v[6]!==0 && Math.abs(v[6])<=2*Math.PI,
    point:(v,t)=>{
      const angle=v[5]+t*v[6], x=v[2]*Math.cos(angle), y=v[3]*Math.sin(angle);
      return [v[0]+Math.cos(v[4])*x-Math.sin(v[4])*y,v[1]+Math.sin(v[4])*x+Math.cos(v[4])*y];
    }}],
]);
const interiorAlgorithmIds = new Set([0,1]); // EarClipping, ConstrainedDelaunay
const boundaryTreatmentIds = new Set([0,1,2]); // Fixed, SplitPolygonEdges, RefineOnCurves
// PDE2D_MESH 1 through 4 reader and corner-triangle diagnostics. No external packages.
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
  const registered = (name,registry) => {
    const id=integer(name);
    if(!registry.has(id)) throw new Error(`Unknown ${name}: ${id}`);
    return id;
  };
  const finite = name => {
    const value = take();
    // Number('') and Number('Infinity') must not silently become coordinates.
    if (!/^[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?$/.test(value) || !Number.isFinite(Number(value)))
      throw new Error(`Invalid ${name}: ${value}`);
    return Number(value);
  };
  expect('PDE2D_MESH');
  const version = integer('version');
  if (version !== 1 && version !== 2 && version !== 3 && version !== 4) throw new Error('Only PDE2D_MESH versions 1 through 4 are supported');
  expect('ORDER');
  const order = integer('order');
  if (order !== 1) throw new Error('Only order 1 is supported');
  expect('NODES');
  const nodeCount = integer('node count');
  if (!nodeCount) throw new Error('Mesh has no nodes');
  remaining(2*nodeCount, 'NODES');
  const nodes = Array.from({length: nodeCount}, () => [finite('x coordinate'), finite('y coordinate')]);
  expect('ELEMENTS');
  const elementCount = integer('element count');
  if (!elementCount) throw new Error('Mesh has no elements');
  remaining(4*elementCount, 'ELEMENTS');
  const elements = Array.from({length: elementCount}, (_, id) => {
    const length = integer('element node count', 3);
    if (length !== 3) throw new Error(`Element ${id}: expected 3 nodes`);
    return Array.from({length}, () => {
      const index = integer('node index');
      if (index >= nodeCount) throw new Error(`Element ${id}: node ${index} is outside NODES`);
      return index;
    });
  });
  expect('BOUNDARY_EDGES');
  const boundaryCount = integer('boundary edge count');
  remaining((version >= 2 ? 3 : 2)*boundaryCount, 'BOUNDARY_EDGES');
  const boundaryEdges = Array.from({length: boundaryCount}, (_, id) => {
    const a = integer('boundary endpoint'), b = integer('boundary endpoint');
    if (a >= nodeCount || b >= nodeCount) throw new Error(`Boundary edge ${id}: endpoint outside NODES`);
    if (version === 1) return [a, b];
    const region = take();
    if (!/^[A-Za-z0-9_.-]+$/.test(region)) throw new Error(`Boundary edge ${id}: invalid region ${region}`);
    return [a, b, region === '-' ? null : region];
  });
  const result = {nodes, elements, boundaryEdges};
  if (version >= 3) {
    expect('OPTIONS');
    const options = {algorithm: registered('algorithm',interiorAlgorithmIds), boundaryTreatment: registered('boundary treatment',boundaryTreatmentIds), preserveRegions: !!integer('preserve regions',1),
      boundary: {maxEdgeLength: finite('boundary length'), maxCurveDeviation: finite('curve deviation')},
      refinement: {maxEdgeLength: finite('refinement length'), maxSteps: integer('max steps',2147483647)}};
    options.boundary.preserveCorners = version >= 4 ? !!integer('preserve corners',1) : true;
    if (options.boundary.maxEdgeLength<0 || options.boundary.maxCurveDeviation<0 || options.refinement.maxEdgeLength<0) throw new Error('Invalid meshing tolerances');
    expect('REPORT');
    const report = {sizeSatisfied: !!integer('size satisfied',1), delaunayVerified: !!integer('Delaunay verified',1), refinementSteps: integer('refinement steps',Number.MAX_SAFE_INTEGER)};
    const hex = take();
    if (hex !== '-' && !/^(?:[0-9a-f]{2})+$/.test(hex)) throw new Error('Invalid report message encoding');
    // Decode UTF-8 without a dependency or relying on TextDecoder in older browsers.
    report.message = hex === '-' ? '' : decodeURIComponent(hex.replace(/../g,byte=>`%${byte}`));
    result.options=options; result.report=report;
    expect('CONTEXT');
    if (integer('context',1)) {
      expect('PROTECTED_VERTICES'); const count=integer('protected vertex count',nodeCount);
      remaining(count,'PROTECTED_VERTICES');
      const protectedVertices=Array.from({length:count},()=>integer('protected vertex',nodeCount-1));
      const boundaryNodes=new Set(boundaryEdges.flatMap(e=>e.slice(0,2)));
      if (new Set(protectedVertices).size!==count || protectedVertices.some(v=>!boundaryNodes.has(v))) throw new Error('Invalid protected boundary vertices');
      expect('POLYGON_SNAPSHOT');const polygonCount=integer('polygon snapshot count');
      if(polygonCount<3)throw new Error('Polygon snapshot needs three vertices');
      remaining(2*polygonCount,'POLYGON_SNAPSHOT');
      const polygonSnapshot=Array.from({length:polygonCount},()=>[finite('polygon x'),finite('polygon y')]);
      expect('SOURCE_CURVES'); const curveCount=integer('source curve count');
      if(!curveCount) throw new Error('Missing source curves');
      remaining(7*curveCount,'SOURCE_CURVES');
      const ids=new Set();
      const curves=Array.from({length:curveCount},()=>{
        // Keep IDs as decimal strings so the full uint64 range survives parsing.
        const id=take();
        if(!/^(0|[1-9]\d*)$/.test(id)||BigInt(id)>18446744073709551615n||ids.has(id))throw new Error('Invalid or duplicate source curve ID');
        ids.add(id); const tag=take();
        if(!/^[A-Za-z0-9_.-]+$/.test(tag))throw new Error('Invalid source curve region');
        const kind=registered('curve kind',sourceCurveFormats), format=sourceCurveFormats.get(kind);
        const values=Array.from({length:format.count},()=>finite('source curve coordinate'));
        if(format.validate && !format.validate(values)) throw new Error('Invalid source curve parameters');
        return {id,region:tag==='-'?null:tag,kind,values};
      });
      expect('EDGE_SOURCES'); const edgeCount=integer('edge source count');
      if(edgeCount!==boundaryCount)throw new Error('Invalid edge source count');
      let total=0;
      const close=(a,b)=>a.every(Number.isFinite)&&b.every(Number.isFinite)&&distance(a,b)<=1e-9*Math.max(1,Math.hypot(...a),Math.hypot(...b));
      const edgeSources=boundaryEdges.map(([a,b,tag])=>{
        const count=integer('span count');total+=count;
        if(!count||total>50_000_000)throw new Error('Invalid source span count');
        remaining(3*count,'EDGE_SOURCES'); let previous=null;
        const spans=Array.from({length:count},()=>{
          const curve=integer('source curve index',curveCount-1), begin=finite('span begin'),end=finite('span end');
          if(begin<0||begin>1||end<0||end>1||begin===end)throw new Error('Invalid source interval');
          const source=curves[curve];
          if(previous && !close(previous,sourcePoint(source,begin)))throw new Error('Disconnected source spans');
          previous=sourcePoint(source,end);
          if(options.preserveRegions && tag!==source.region)throw new Error('Source region mismatch');
          return {curve,begin,end};
        });
        return spans;
      });
      const outgoing=new Map(boundaryEdges.map(([a],i)=>[a,i]));
      for(let i=0;i<boundaryEdges.length;i++) {
        const next=outgoing.get(boundaryEdges[i][1]);if(next===undefined)throw new Error('Disconnected source boundary');
        const left=edgeSources[i].at(-1),right=edgeSources[next][0];
        if(!close(sourcePoint(curves[left.curve],left.end),sourcePoint(curves[right.curve],right.begin)))throw new Error('Source boundary intervals do not join');
      }
      result.boundaryContext={protectedVertices,polygonSnapshot,curves,edgeSources};
    } else result.boundaryContext=null;
    expect('END_RESULT');
  }
  if (at !== tokens.length) throw new Error(`Unexpected trailing token: ${tokens[at]}`);
  return result;
}

function sourcePoint({kind,values},t) {
  const format=sourceCurveFormats.get(kind);
  if(!format) throw new Error('Unknown source curve kind');
  return format.point(values,t);
}
function bezierSourcePoint(v,t) {
  const points=[];for(let i=0;i<v.length;i+=2)points.push(v.slice(i,i+2));
  while(points.length>1) {for(let i=0;i<points.length-1;i++)points[i]=points[i].map((x,j)=>(1-t)*x+t*points[i+1][j]);points.pop();}
  return points[0];
}

const key = (a, b) => a < b ? `${a},${b}` : `${b},${a}`;
const distance = (a, b) => Math.hypot(a[0]-b[0], a[1]-b[1]);
const format = n => Number.isFinite(n) ? Number(n.toPrecision(6)).toString() : 'undefined';

// Match the native orientation predicate: use an exact dyadic determinant
// when floating-point cancellation or underflow makes the sign uncertain.
function orientation(a,b,c) {
  const ax=a[0]-c[0],ay=a[1]-c[1],bx=b[0]-c[0],by=b[1]-c[1];
  const p=ax*by,q=ay*bx,det=p-q,bound=16*Number.EPSILON*(Math.abs(p)+Math.abs(q));
  if(Number.isFinite(det)&&Number.isFinite(bound)&&bound>=2**-1022&&Math.abs(det)>bound)
    return Math.sign(det);
  const bits=new DataView(new ArrayBuffer(8));
  const parts=value=>{
    bits.setFloat64(0,value);
    const raw=bits.getBigUint64(0),exponent=Number((raw>>52n)&2047n);
    let mantissa=raw&((1n<<52n)-1n);
    if(exponent)mantissa|=1n<<52n;
    if(raw>>63n)mantissa=-mantissa;
    return {mantissa,exponent:exponent?exponent-1075:-1074};
  };
  const values=[...a,...b,...c].map(parts);
  const scale=Math.min(...values.filter(v=>v.mantissa!==0n).map(v=>v.exponent),0);
  const [x,y,u,v,s,t]=values.map(v=>v.mantissa<<BigInt(v.exponent-scale));
  const exact=(x-s)*(v-t)-(y-t)*(u-s);
  return exact>0n?1:exact<0n?-1:0;
}

function analyse(mesh) {
  const {nodes, elements, boundaryEdges} = mesh;
  const bounds = [Infinity, Infinity, -Infinity, -Infinity];
  for (const [x,y] of nodes) {
    bounds[0]=Math.min(bounds[0],x);bounds[1]=Math.min(bounds[1],y);
    bounds[2]=Math.max(bounds[2],x);bounds[3]=Math.max(bounds[3],y);
  }
  const span = Math.max(bounds[2]-bounds[0], bounds[3]-bounds[1]);
  if (!Number.isFinite(span) || !Number.isFinite(span*span))
    throw new Error('Coordinate range exceeds supported numeric scale');
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
    const sign=orientation(a,b,c);
    const status = sign>0?'valid':sign<0?'inverted':'degenerate';
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
      if (!edges.has(k)) edges.set(k,{nodes: [e[local],e[(local+1)%3]], owners: [], directions: []});
      const record = edges.get(k); record.owners.push(id);
      record.directions.push(e[local] < e[(local+1)%3] ? 1 : -1);
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
  return {bounds, triangles, edges, recorded, problemEdges, problemTriangles, issues,
    summary: {nodes:nodes.length, elements:elements.length, boundaryEdges:boundaryEdges.length,
      inverted:triangles.filter(t=>t.status==='inverted').length,
      degenerate:triangles.filter(t=>t.status==='degenerate').length,
      quality:stats(valid.map(t=>t.quality)), minAngle:stats(valid.map(t=>Math.min(...t.angles)))}};
}
// Stable across file order and reloads; untagged edges retain the v1 color.
function regionColor(region) {
  if (region == null) return '#e89b13';
  let hash = 2166136261;
  for (const ch of region) hash = Math.imul(hash ^ ch.charCodeAt(0), 16777619);
  return `hsl(${(hash >>> 0) % 360}, 72%, 40%)`;
}
window.MeshTools = {parseMesh, analyse, format, regionColor};
