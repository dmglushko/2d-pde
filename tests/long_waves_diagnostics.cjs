const fs=require('node:fs'),assert=require('node:assert/strict');
const data=JSON.parse(fs.readFileSync(process.argv[2],'utf8'));
for(const ex of data.examples){
  assert.equal(ex.frames.length,401);assert.equal(ex.frames[0].time,0);assert.equal(ex.frames.at(-1).time,100);
  assert.equal(ex.timings.time_steps,2000);assert.equal(ex.timings.retained_reports,2000);assert.equal(ex.timings.observed_frames,2001);assert.equal(ex.timings.retained_frames,401);
  assert.equal(ex.timings.observer_included_in_integration,true);
  assert.ok(Math.abs(ex.timings.integration_seconds-ex.timings.integration_excluding_observer_seconds-ex.timings.diagnostics_seconds)<1e-12);
  for(let i=0;i<ex.frames.length;i++){
    const f=ex.frames[i];assert.equal(f.time,i*.25);assert.equal(f.values.length,data.nodes.length);
    for(const key of ['energy','mean','work'])assert.ok(Number.isFinite(f[key]));
    if(ex.id==='long-dirichlet')for(const key of ['l2','h1','exact'])assert.equal(key in f,false);
    else {assert.ok(f.l2>=0&&f.h1>=0);assert.equal(f.exact.length,data.nodes.length);}
  }
}
if(process.argv[3]){
  const baseline=JSON.parse(fs.readFileSync(process.argv[3],'utf8'));const maxima={values:0,energy:0,mean:0,work:0,l2:0,h1:0};
  for(let j=0;j<data.examples.length;j++)for(let k=0;k<401;k++){
    const a=data.examples[j].frames[k],b=baseline.examples[j].frames[k];assert.equal(a.time,b.time);
    for(const key of Object.keys(maxima))if(key==='values'){for(let i=0;i<a.values.length;i++)maxima.values=Math.max(maxima.values,Math.abs(a.values[i]-b.values[i]));}
    else if(key in a)maxima[key]=Math.max(maxima[key],Math.abs(a[key]-b[key]));
  }
  for(const v of Object.values(maxima))assert.ok(v<1e-8);console.log('Baseline absolute differences:',maxima);
}
console.log('401 aligned frames, 2001 observations, full precision endpoints, observer timing overlap and driven metrics passed');
