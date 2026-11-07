const assert=require('node:assert/strict');
const fs=require('node:fs'),os=require('node:os'),path=require('node:path');
const {spawnSync}=require('node:child_process');
const {check}=require('./long_waves_diagnostics.cjs');
function fixture(){
  return {nodes:[[0,0],[1,0],[0,1]],examples:['long-dirichlet','long-neumann'].map(id=>({
    id,timings:{time_steps:2000,retained_reports:2000,observed_frames:2001,retained_frames:401,
      observer_included_in_integration:true,integration_seconds:3,integration_excluding_observer_seconds:2,diagnostics_seconds:1},
    frames:Array.from({length:401},(_,i)=>({time:i*.25,values:[0,1,2],energy:1,mean:1,work:0,
      ...(id==='long-neumann'?{l2:0,h1:0,exact:[0,1,2]}:{})}))
  }))};
}
const valid=fixture();
assert.doesNotThrow(()=>check(valid));
assert.doesNotThrow(()=>check(valid,fixture()));
for(const invalid of [null,false,0,'',{},[]])assert.throws(()=>check(valid,invalid));
const legacy=fixture();
for(const ex of legacy.examples)for(const key of ['time_steps','retained_reports','observed_frames','retained_frames','observer_included_in_integration','integration_excluding_observer_seconds'])delete ex.timings[key];
legacy.examples.reverse();
assert.doesNotThrow(()=>check(valid,legacy),'Legacy baseline, matched by case ID');
const mutations=[
  d=>{d.examples=[];},d=>{delete d.examples;},d=>{d.examples.pop();},
  d=>{d.examples.push(structuredClone(d.examples[0]));},
  d=>{d.examples[1]=structuredClone(d.examples[0]);},d=>{d.examples[0].id='unknown';},
  d=>{d.nodes=[];},d=>{d.nodes[0][0]=Infinity;},
];
for(let j=0;j<2;j++){
  const ex=d=>d.examples[j],frame=d=>ex(d).frames[5];
  mutations.push(
    d=>{ex(d).frames=[];},d=>{delete ex(d).frames;},d=>{ex(d).frames.pop();},
    d=>{ex(d).frames[5]=structuredClone(ex(d).frames[4]);},
    d=>{frame(d).time=Infinity;},d=>{frame(d).values.pop();},
    d=>{delete frame(d).values;},d=>{frame(d).values[0]=NaN;},
    d=>{delete ex(d).timings;},d=>{ex(d).timings.integration_seconds=Infinity;},
    d=>{ex(d).timings.observed_frames=0;}
  );
  for(const key of ['energy','mean','work'])for(const value of [undefined,null,Infinity,'0'])mutations.push(d=>{frame(d)[key]=value;});
}
for(const key of ['l2','h1'])for(const value of [undefined,null,Infinity,-1,'0'])mutations.push(d=>{d.examples[1].frames[5][key]=value;});
mutations.push(d=>{delete d.examples[1].frames[5].exact;},d=>{d.examples[1].frames[5].exact=[];},
  d=>{d.examples[1].frames[5].exact[0]=Infinity;},d=>{d.examples[0].frames[5].l2=0;});
for(const [i,mutate] of mutations.entries()){
  const broken=fixture();mutate(broken);
  assert.throws(()=>check(broken),`Current mutation ${i}`);
  assert.throws(()=>check(valid,broken),`Baseline mutation ${i}`);
}
for(const key of ['time_steps','retained_reports','observed_frames','retained_frames','observer_included_in_integration','integration_excluding_observer_seconds']){
  const broken=fixture();delete broken.examples[0].timings[key];
  assert.throws(()=>check(broken),`Missing current ${key}`);
}
const changed=fixture();changed.examples[1].frames[5].values[0]=.01;
assert.throws(()=>check(valid,changed),'Baseline numerical mismatch');
// Exercise the actual CLI for the reported two-empty-input false positive.
const directory=fs.mkdtempSync(path.join(os.tmpdir(),'wave-diagnostics-'));
try{
  const file=path.join(directory,'waves.json'),checker=path.join(__dirname,'long_waves_diagnostics.cjs');
  for(const data of [{examples:[]},valid]){
    fs.writeFileSync(file,JSON.stringify(data));
    const result=spawnSync(process.execPath,[checker,file,file],{encoding:'utf8'});
    assert.equal(result.status,data===valid?0:1,result.stderr);
    assert.equal(result.stdout.includes('observations'),data===valid);
  }
}finally{fs.rmSync(directory,{recursive:true,force:true});}
console.log(`${mutations.length} malformed datasets rejected in both input positions; valid and legacy baselines passed`);
