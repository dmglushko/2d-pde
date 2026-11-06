// Node VM measures JS work with no-op canvas, not browser rasterization or paint.
const fs=require('node:fs'),vm=require('node:vm'),path=require('node:path');
const n=100,nodes=[],elements=[],edges=[];
for(let y=0;y<=n;y++)for(let x=0;x<=n;x++)nodes.push(`${x} ${y}`);
for(let y=0;y<n;y++)for(let x=0;x<n;x++){const a=y*(n+1)+x,b=a+1,c=a+n+1,d=c+1;elements.push(`3 ${a} ${b} ${d}`,`3 ${a} ${d} ${c}`);}
for(let i=0;i<n;i++){edges.push(`${i} ${i+1}`,`${i*(n+1)+n} ${(i+1)*(n+1)+n}`,`${n*(n+1)+i+1} ${n*(n+1)+i}`,`${(i+1)*(n+1)} ${i*(n+1)}`);}
const source=`PDE2D_MESH 1 ORDER 1 NODES ${nodes.length} ${nodes.join(' ')} ELEMENTS ${elements.length} ${elements.join(' ')} BOUNDARY_EDGES ${edges.length} ${edges.join(' ')}`;
for(const dir of process.argv.slice(2)){
  const listeners={},el={},queue=[];let redraws=0,projections=0;
  const ctx=new Proxy({clearRect(){redraws++}}, {get:(o,k)=>o[k]||(()=>{}),set:(o,k,v)=>(o[k]=v,true)});
  const get=id=>el[id]||(el[id]={checked:['wire','boundary'].includes(id),classList:{add(){},remove(){}},addEventListener:(k,f)=>listeners[id+':'+k]=f,getBoundingClientRect:()=>({width:800,height:600}),getContext:()=>ctx,setPointerCapture(){}});
  const context={document:{getElementById:get},window:{devicePixelRatio:1,exampleMesh:source},ResizeObserver:class{observe(){}},requestAnimationFrame:f=>queue.push(f),countProjection:()=>projections++};
  vm.createContext(context);vm.runInContext(fs.readFileSync(path.join(dir,'mesh.js'),'utf8'),context);
  const start=performance.now();for(let i=0;i<5;i++)context.window.MeshTools.parseMesh(source);const imports=(performance.now()-start)/1000;
  let app=fs.readFileSync(path.join(dir,'app.js'),'utf8').replace('const screen = p => [','const screen = p => (countProjection(),[').replace('(view.y-p[1])/view.s];','(view.y-p[1])/view.s]);');
  vm.runInContext(app,context);while(queue.length)queue.shift()();redraws=0;projections=0;
  listeners['canvas:pointerdown']({pointerId:1,offsetX:400,offsetY:300});
  const begin=performance.now();for(let i=1;i<=100;i++)listeners['canvas:pointermove']({offsetX:400+i,offsetY:300});while(queue.length)queue.shift()();
  console.log(JSON.stringify({dir,bytes:Buffer.byteLength(source),nodes:nodes.length,triangles:elements.length,imports:5,import_seconds:imports,pointer_events:100,interaction_seconds:(performance.now()-begin)/1000,redraws,projections}));
}
