const fs=require('node:fs'),assert=require('node:assert/strict');
const ids=['long-dirichlet','long-neumann'];
function finite(value,label){assert.ok(Number.isFinite(value),`${label} must be finite`);}
function vector(values,size,label){
  assert.ok(Array.isArray(values),`${label} must be an array`);
  assert.equal(values.length,size,`${label} size`);
  for(const value of values)finite(value,label);
}
function validate(data,{baseline=false}={}){
  assert.ok(Array.isArray(data.nodes)&&data.nodes.length>0,'Expected mesh nodes');
  for(const node of data.nodes)vector(node,2,'node');
  assert.ok(Array.isArray(data.examples),'Expected examples');
  assert.deepEqual(data.examples.map(ex=>ex.id).sort(),[...ids].sort(),'Expected each wave case exactly once');
  const cases=new Map(data.examples.map(ex=>[ex.id,ex]));
  for(const ex of cases.values()){
    assert.ok(Array.isArray(ex.frames),'Expected frames');
    assert.equal(ex.frames.length,401);
    const timings=ex.timings;
    // Historical benchmark baselines predate the observer counters.
    assert.ok(timings&&typeof timings==='object','Expected timings');
    for(const [key,value] of Object.entries(timings)){
      if(key==='observer_included_in_integration')assert.equal(value,true);
      else {finite(value,key);assert.ok(value>=0,key);}
    }
    for(const key of ['integration_seconds','diagnostics_seconds'])finite(timings[key],key);
    for(const [key,value] of Object.entries({time_steps:2000,retained_reports:2000,observed_frames:2001,retained_frames:401,observer_included_in_integration:true})){
      if(!baseline||key in timings)assert.equal(timings[key],value,key);
    }
    if(!baseline||'integration_excluding_observer_seconds' in timings){
      finite(timings.integration_excluding_observer_seconds,'integration_excluding_observer_seconds');
      assert.ok(Math.abs(timings.integration_seconds-timings.integration_excluding_observer_seconds-timings.diagnostics_seconds)<1e-12);
    }
    for(let i=0;i<401;i++){
      const f=ex.frames[i];assert.equal(f.time,i*.25,'Frame time/order');
      vector(f.values,data.nodes.length,'values');
      for(const key of ['energy','mean','work'])finite(f[key],key);
      if(ex.id==='long-dirichlet')for(const key of ['l2','h1','exact'])assert.equal(key in f,false);
      else {
        for(const key of ['l2','h1']){finite(f[key],key);assert.ok(f[key]>=0,key);}
        vector(f.exact,data.nodes.length,'exact');
      }
    }
  }
  return cases;
}
function check(data,baseline){
  const cases=validate(data);
  if(baseline!==undefined){
    const previous=validate(baseline,{baseline:true});
    assert.deepEqual(data.nodes,baseline.nodes,'Baseline mesh nodes');
    const maxima={values:0,energy:0,mean:0,work:0,l2:0,h1:0,exact:0};
    for(const id of ids)for(let k=0;k<401;k++){
      const a=cases.get(id).frames[k],b=previous.get(id).frames[k];
      for(const key of Object.keys(maxima))if(key==='values'||key==='exact'){
        if(key in a)for(let i=0;i<a[key].length;i++)maxima[key]=Math.max(maxima[key],Math.abs(a[key][i]-b[key][i]));
      }else if(key in a)maxima[key]=Math.max(maxima[key],Math.abs(a[key]-b[key]));
    }
    for(const v of Object.values(maxima))assert.ok(v<1e-8);
    return maxima;
  }
}
if(require.main===module){
  assert.ok(process.argv[2]&&process.argv.length<=4,'Usage: long_waves_diagnostics.cjs current.json [baseline.json]');
  const read=file=>JSON.parse(fs.readFileSync(file,'utf8'));
  const maxima=check(read(process.argv[2]),process.argv[3]?read(process.argv[3]):undefined);
  if(maxima)console.log('Baseline absolute differences:',maxima);
  console.log('401 aligned frames per case, 2001 observations, full precision endpoints, observer timing overlap and driven metrics passed');
}
module.exports={check};
