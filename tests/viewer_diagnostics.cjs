// Execute the standalone gallery's actual inline script with a deterministic DOM/canvas.
const fs=require('node:fs'),vm=require('node:vm'),assert=require('node:assert/strict');
const cp=require('node:child_process'),os=require('node:os'),path=require('node:path');
const temp=fs.mkdtempSync(path.join(os.tmpdir(),'pde2d-viewer7-'));
function boot(html){
  const elements={},listeners={},raf=[];let now=0,clears=0,resets=0;
  const ctx=new Proxy({clearRect(){clears++}}, {get:(o,k)=>o[k]||(()=>{}),set:(o,k,v)=>(o[k]=v,true)});
  function element(id){if(elements[id])return elements[id];let width=600,height=340;
    const e={value:'',textContent:'',innerHTML:'',parentElement:{},options:[],append(v){this.options.push(v)},setAttribute(){},getContext:()=>ctx,getBoundingClientRect:()=>({width,height}),resize(w,h){width=w;height=h}};
    for(const k of ['width','height'])Object.defineProperty(e,k,{get(){return this['_'+k]},set(v){resets++;this['_'+k]=v}});
    return elements[id]=e;
  }
  element('field').value='values';element('field').options=['values','exact','error'].map(value=>({value}));
  const context={document:{getElementById:element,createElement:()=>({}),documentElement:{}},window:{devicePixelRatio:1,addEventListener:(k,f)=>listeners[k]=f},getComputedStyle:()=>({getPropertyValue:()=>''}),performance:{now:()=>now},requestAnimationFrame:f=>raf.push(f),atob:s=>Buffer.from(s,'base64').toString('binary')};
  vm.createContext(context);vm.runInContext(html.match(/<script>([\s\S]*)<\/script>/)[1],context);
  return {elements,listeners,eval:s=>vm.runInContext(s,context),tick(t){now=t;const f=raf.shift();f(t)},get resets(){return resets},get clears(){return clears}};
}
const fixture={nodes:[[0,0],[1,0],[0,1]],triangles:[[0,1,2]],convergence:[{n:4,l2:.1,h1:.2}],examples:['a','b'].map((id,j)=>({id,name:id,residual:0,frames:[0,1,2].map(time=>({time,values:[time+j,1,4],...(j?{}:{exact:[0,1,3]}),l2:time,h1:time}))}))};
fs.writeFileSync(path.join(temp,'fixture.json'),JSON.stringify(fixture));
cp.execFileSync('node',['tools/render_solutions.mjs',path.join(temp,'fixture.json'),path.join(temp,'gallery.html')]);
let ui=boot(fs.readFileSync(path.join(temp,'gallery.html'),'utf8'));let e=ui.elements;
assert.equal(e.timestamp.textContent,'t = 0.0000');const fixed=e.high.textContent,reset=ui.resets;
e.time.value='2';e.time.oninput();assert.equal(e.timestamp.textContent,'t = 2.0000');assert.equal(e.high.textContent,fixed);assert.equal(ui.resets,reset);
e.field.value='error';e.field.onchange();assert.notEqual(e.high.textContent,fixed);assert.equal(ui.eval('bound()'),2);
e.view.onclick();assert.equal(ui.eval('map'),true);assert.equal(ui.eval('baseProjection[1][0]'),575);
e.surface.resize(800,400);ui.listeners.resize();assert.equal(ui.eval('baseProjection[1][0]'),775);assert.ok(ui.resets>reset);
e.example.value='b';e.example.onchange();assert.equal(e.field.value,'values');assert.equal(e.field.options[1].disabled,true);assert.equal(e.timestamp.textContent,'t = 0.0000');
e.speed.value='1';e.play.onclick();ui.tick(500);assert.equal(ui.eval('frame'),0);ui.tick(1100);assert.equal(ui.eval('frame'),1);
e.speed.value='4';ui.tick(1350);assert.equal(ui.eval('frame'),0); // wraps 2.1 -> .1
e.time.value='1';e.time.oninput();ui.tick(1450);assert.equal(ui.eval('frame'),1);e.play.onclick();ui.tick(2000);assert.equal(ui.eval('frame'),1);
e.example.value='a';e.example.onchange();assert.equal(e.field.options[1].disabled,false);
console.log('Gallery frame/case/field/view switches, fixed scale, canvas reuse, resize, pause, scrub and speed/time playback passed');
// Optional same-data redraw benchmark of saved baseline and current packaging.
if(process.argv[2])for(const filename of process.argv.slice(2)){
  const html=fs.readFileSync(filename,'utf8');const start=performance.now();ui=boot(html);const setup=(performance.now()-start)/1000;
  const before=ui.resets,t=performance.now();for(let i=0;i<20;i++)ui.eval('frame=(frame+1)%selected.frames.length;drawSurface()');
  console.log(JSON.stringify({file:filename,setup_seconds:setup,redraw_seconds:(performance.now()-t)/1000,redraws:20,canvas_resets:ui.resets-before,nodes:ui.eval('data.nodes.length'),triangles:ui.eval('data.triangles.length'),frames:ui.eval('selected.frames.length')}));
}
