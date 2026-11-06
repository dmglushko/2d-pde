#!/usr/bin/env node
// Run only after other builds/tests finish. Keep raw datasets and timings.
import fs from 'node:fs';
import path from 'node:path';
import os from 'node:os';
import assert from 'node:assert/strict';
import {createHash} from 'node:crypto';
import {spawnSync,execFileSync} from 'node:child_process';

const [baseline,serial,openmp,destination='build/wave-benchmark',count='3']=process.argv.slice(2);
const repetitions=Number(count);
if(!baseline||!serial||!openmp||!Number.isInteger(repetitions)||repetitions<1)
  throw Error('Usage: benchmark_long_waves.mjs baseline_binary serial_binary openmp_binary [directory [repetitions]]');
fs.mkdirSync(destination,{recursive:true});
const policies=[['serial',serial,1],['baseline',baseline,1],['omp2',openmp,2],['omp4',openmp,4]];
const env={...process.env,OMP_DYNAMIC:'FALSE',OPENBLAS_NUM_THREADS:'1',MKL_NUM_THREADS:'1'};
const report={
  recorded_at:new Date().toISOString(),commit:execFileSync('git',['rev-parse','HEAD'],{encoding:'utf8'}).trim(),
  cpu:os.cpus()[0].model,logical_cpus:os.cpus().length,platform:os.platform(),release:os.release(),
  compiler:execFileSync('c++',['--version'],{encoding:'utf8'}).split('\n')[0],
  settings:{subdivisions:20,nodes:441,triangles:800,side_length:10,end:100,step:.05,quadrature_degree:6,repetitions},
  note:'Sequential processes, fixed workload; machine frequency/load not controlled. Integration excludes timed observer diagnostics for old/new comparison; diagnostics are reported separately. No wall-time assertions.',
  binaries:policies.map(([policy,binary,threads])=>({policy,binary:path.resolve(binary),threads,sha256:createHash('sha256').update(fs.readFileSync(binary)).digest('hex')})),
  runs:[]
};
let reference;
for(let run=1;run<=repetitions;run++)for(const [policy,binary,threads] of policies){
  const output=path.join(destination,`${policy}-${run}.json`);
  const args=[output,'20','.05'];if(policy!=='baseline')args.push(String(threads));
  const result=spawnSync(path.resolve(binary),args,{encoding:'utf8',env:{...env,OMP_NUM_THREADS:String(threads)},maxBuffer:4*1024*1024});
  fs.writeFileSync(output+'.log',result.stdout+(result.stderr||''));
  assert.equal(result.status,0,result.stdout+result.stderr);
  const data=JSON.parse(fs.readFileSync(output,'utf8'));
  const profile=JSON.parse(fs.readFileSync(output.replace(/\.json$/,'.timings.json'),'utf8'));
  assert.equal(data.nodes.length,441);assert.equal(data.triangles.length,800);
  if(!reference&&policy==='serial')reference=data;
  const differences={values:0,energy:0,mean:0,work:0,l2:0,h1:0};
  for(let j=0;j<data.examples.length;j++){
    const example=data.examples[j],timings=example.timings;
    assert.equal(example.frames.length,401);assert.equal(example.frames[0].time,0);assert.equal(example.frames.at(-1).time,100);
    assert.equal(timings.assembly_calls,2003);assert.equal(timings.linear_solves,2001);
    if(policy!=='baseline'){
      assert.equal(timings.time_steps,2000);assert.equal(timings.observed_frames,2001);
      assert.equal(timings.solver_preparations,2);assert.equal(timings.rank_checks,2);
      assert.equal(timings.factorizations,2);assert.equal(timings.solver_reuses,1999);
    }
    if(reference)for(let k=0;k<401;k++){
      const frame=example.frames[k],other=reference.examples[j].frames[k];assert.equal(frame.time,other.time);
      for(const key of Object.keys(differences))if(key==='values'){
        for(let i=0;i<frame.values.length;i++)differences.values=Math.max(differences.values,Math.abs(frame.values[i]-other.values[i]));
      }else if(key in frame)differences[key]=Math.max(differences[key],Math.abs(frame[key]-other[key]));
    }
  }
  if(reference){
    for(const error of Object.values(differences))assert.ok(error<1e-8);
    if(policy!=='baseline')assert.ok(Object.values(differences).every(error=>error===0),'Serial/parallel wave frames differ');
  }
  for(const c of profile.cases)c.timings.comparable_integration_seconds=c.timings.integration_excluding_observer_seconds??c.timings.integration_seconds;
  report.runs.push({run,policy,differences:reference?differences:null,profile});
  fs.writeFileSync(path.join(destination,'results.json'),JSON.stringify(report,null,2)+'\n');
  console.log(JSON.stringify({run,policy,integration_seconds:profile.cases.map(c=>c.timings.comparable_integration_seconds),diagnostics_seconds:profile.cases.map(c=>c.timings.diagnostics_seconds)}));
}
const median=values=>[...values].sort((a,b)=>a-b)[Math.floor(values.length/2)];
report.summary=policies.map(([policy])=>({policy,cases:[0,1].map(i=>{
  const cases=report.runs.filter(r=>r.policy===policy).map(r=>r.profile.cases[i]);
  const keys=['comparable_integration_seconds','assembly_seconds','constraints_seconds','rank_check_seconds','factorization_seconds','linear_solve_seconds','diagnostics_seconds'];
  return {name:cases[0].name,...Object.fromEntries(keys.map(key=>[key,{median:median(cases.map(c=>c.timings[key])),min:Math.min(...cases.map(c=>c.timings[key])),max:Math.max(...cases.map(c=>c.timings[key]))}]))};
})}));
fs.writeFileSync(path.join(destination,'results.json'),JSON.stringify(report,null,2)+'\n');
