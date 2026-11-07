// Tiny workloads test the public CLI without asserting machine-dependent speed.
const assert=require('node:assert/strict'),fs=require('node:fs'),path=require('node:path');
const {spawnSync}=require('node:child_process');
const [binary,directory,openmp]=process.argv.slice(2);
const run=(...args)=>spawnSync(binary,args,{encoding:'utf8',env:{...process.env,OMP_DYNAMIC:'FALSE'}});
const help=run('--help');assert.equal(help.status,0,help.stderr);assert.ok(help.stdout.includes('--benchmark'));
const result=run('--benchmark',directory,'2','2','50');
if(openmp==='ON'){
  assert.equal(result.status,0,result.stderr+result.stdout);
  assert.ok(result.stdout.includes('FEM evolution:')&&result.stdout.includes('observer='));
  const report=JSON.parse(fs.readFileSync(path.join(directory,'benchmark.json'),'utf8'));
  assert.equal(report.repetitions,2);assert.equal(report.subdivisions,2);assert.equal(report.time_step,50);
  assert.equal(report.runs.length,6);assert.deepEqual(report.summary.map(s=>s.threads),[1,2,4]);
  for(const summary of report.summary){
    assert.ok(Number.isFinite(summary.median_seconds)&&summary.median_seconds>=0);
    assert.ok(summary.min_seconds<=summary.median_seconds&&summary.median_seconds<=summary.max_seconds);
  }
  for(const entry of report.runs){
    assert.ok(entry.max_difference<=1e-8);assert.equal(entry.cases.length,2);
    const policy=entry.threads===1?'serial':`omp${entry.threads}`;
    const data=JSON.parse(fs.readFileSync(path.join(directory,`${policy}-${entry.repetition}.json`),'utf8'));
    assert.equal(data.execution_threads,entry.threads);
    assert.deepEqual(data.examples.map(e=>e.id),['long-dirichlet','long-neumann']);
    for(const ex of data.examples){
      assert.equal(ex.timings.time_steps,2);assert.equal(ex.timings.observed_frames,3);
      assert.equal(ex.frames[0].time,0);assert.equal(ex.frames.at(-1).time,100);
    }
  }
}else{
  assert.notEqual(result.status,0);assert.ok(result.stderr.includes('requires an OpenMP-enabled build'));
  const single=run(path.join(directory,'serial.json'),'2','50','1');
  assert.equal(single.status,0,single.stderr+single.stdout);
  assert.ok(single.stdout.includes('FEM evolution:')&&single.stdout.includes('selected_backend=Serial'));
}
const invalid=run('--benchmark',directory,'0');assert.notEqual(invalid.status,0);
console.log('Native benchmark CLI, reports and backend availability checks passed');
