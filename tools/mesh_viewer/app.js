(() => {
  const {parseMesh, analyse, format, regionColor} = window.MeshTools;
  const $ = id => document.getElementById(id);
  const canvas = $('canvas'), wrap=$('dropzone'), ctx=canvas.getContext('2d');
  const checks=['wire','boundary','nodes','triangles'];
  let mesh=null, analysis=null, selected=-1, view={x:0,y:0,s:1};
  let pointer=null, moved=false;
  const safe = value => String(value).replace(/[&<>"']/g, c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
  const number = n => format(n);
  const showError = message => {$('error').textContent=message;$('error').hidden=false;};
  const clearError = () => {$('error').hidden=true;};
  const world = (x,y) => [view.x+x*view.s,view.y-y*view.s];
  const screen = p => [(p[0]-view.x)/view.s,(view.y-p[1])/view.s];
  const px = () => window.devicePixelRatio || 1;
  const resize = () => {
    const r=wrap.getBoundingClientRect(), d=px();
    canvas.width=Math.max(1,Math.round(r.width*d));canvas.height=Math.max(1,Math.round(r.height*d));
    ctx.setTransform(d,0,0,d,0,0);draw();
  };
  const size = () => [canvas.width/px(),canvas.height/px()];
  const fit = () => {
    if(!analysis)return;
    const [w,h]=size(),[x0,y0,x1,y1]=analysis.bounds;
    const spanX=Math.max(x1-x0,1e-12),spanY=Math.max(y1-y0,1e-12);
    view.s=Math.max(spanX/Math.max(w-90,1),spanY/Math.max(h-90,1));
    view.x=(x0+x1)/2-w*view.s/2;view.y=(y0+y1)/2+h*view.s/2;
    draw();
  };
  function polygon(e) {
    ctx.beginPath();e.slice(0,3).forEach((id,i)=>{const [x,y]=screen(mesh.nodes[id]);i?ctx.lineTo(x,y):ctx.moveTo(x,y);});ctx.closePath();
  }
  function draw() {
    const [w,h]=size();ctx.clearRect(0,0,w,h);if(!mesh)return;
    const flags=Object.fromEntries(checks.map(id=>[id,$(id).checked]));
    // A quiet blue fill makes individual triangles selectable without hiding the grid.
    for(const t of analysis.triangles){
      polygon(t.nodes);
      const hasIssue=t.status!=='valid'||analysis.problemTriangles.has(t.id);
      ctx.fillStyle=t.id===selected?'#bcd4ff':hasIssue?'#f7a3ab88':'#dbe8fa82';ctx.fill();
      if(flags.wire){ctx.strokeStyle=t.id===selected?'#2863cc':'#8fa6c4';ctx.lineWidth=t.id===selected?1.8:.8;ctx.stroke();}
      if(hasIssue){
        ctx.strokeStyle='#d74856';ctx.lineWidth=3;ctx.stroke();
        const p=t.nodes.slice(0,3).map(id=>mesh.nodes[id]);
        const [x,y]=screen([(p[0][0]+p[1][0]+p[2][0])/3,(p[0][1]+p[1][1]+p[2][1])/3]);
        ctx.fillStyle='#bb3548';ctx.font='bold 15px system-ui';ctx.fillText('!',x-3,y+5);
      }
      if(flags.triangles){
        const p=t.nodes.slice(0,3).map(id=>mesh.nodes[id]);
        const [x,y]=screen([(p[0][0]+p[1][0]+p[2][0])/3,(p[0][1]+p[1][1]+p[2][1])/3]);
        ctx.fillStyle='#32537f';ctx.font='10px ui-monospace, monospace';ctx.textAlign='center';ctx.fillText(t.id,x,y);ctx.textAlign='left';
      }
    }
    if(flags.boundary){ctx.lineWidth=3;ctx.lineCap='round';for(const [a,b,region] of mesh.boundaryEdges){ctx.strokeStyle=regionColor(region);const p=screen(mesh.nodes[a]),q=screen(mesh.nodes[b]);ctx.beginPath();ctx.moveTo(...p);ctx.lineTo(...q);ctx.stroke();}}
    ctx.strokeStyle='#a036ac';ctx.lineWidth=3;
    for(const k of analysis.problemEdges){const edge=analysis.edges.get(k);const ends=edge?edge.nodes:k.split(',').map(Number);const p=screen(mesh.nodes[ends[0]]),q=screen(mesh.nodes[ends[1]]);ctx.beginPath();ctx.moveTo(...p);ctx.lineTo(...q);ctx.stroke();}
    if(flags.nodes){
      ctx.font='10px ui-monospace, monospace';
      for(let id=0;id<mesh.nodes.length;id++){const [x,y]=screen(mesh.nodes[id]);
        ctx.fillStyle='#203e68';ctx.beginPath();ctx.arc(x,y,2.2,0,2*Math.PI);ctx.fill();
        ctx.fillStyle='#193961';ctx.fillText(String(id),x+4,y-4);
      }
    }
    // Protected junctions stay visible independently of node ID labels.
    ctx.strokeStyle='#127b72';ctx.lineWidth=2;
    for(const id of mesh.boundaryContext?.protectedVertices || []) {
      const [x,y]=screen(mesh.nodes[id]);ctx.beginPath();ctx.arc(x,y,5,0,2*Math.PI);ctx.stroke();
    }
    $('scale').textContent=`1 screen px ≈ ${number(view.s)} mesh units`;
  }
  function displaySummary() {
    const regions=[...new Set(mesh.boundaryEdges.map(edge=>edge[2] ?? null))].sort((a,b)=>a === null ? -1 : b === null ? 1 : a.localeCompare(b));
    $('regions').innerHTML=regions.map(region=>`<span><i class="swatch" style="border-color:${regionColor(region)};border-top-width:3px"></i>${region === null ? 'untagged boundary' : safe(region)}</span>`).join('');
    const s=analysis.summary;
    const stat=(label,value,wide=false)=>`<div class="stat${wide?' wide':''}"><strong>${safe(value)}</strong><span>${safe(label)}</span></div>`;
    $('mesh-title').textContent='P1 triangular mesh';
    $('overview').innerHTML=stat('Nodes',s.nodes)+stat('Triangles',s.elements)+stat('Recorded boundary edges',s.boundaryEdges)+stat('Degenerate / inverted triangles',s.degenerate+s.inverted)+stat('Mean-ratio quality: min / mean / max',s.quality?`${number(s.quality.min)} / ${number(s.quality.mean)} / ${number(s.quality.max)}`:'—',true)+stat('Minimum angle: min / mean / max',s.minAngle?`${number(s.minAngle.min)}° / ${number(s.minAngle.mean)}° / ${number(s.minAngle.max)}°`:'—',true)+'<p class="muted" style="grid-column:span 2">Quality summaries exclude degenerate and inverted triangles. Connectivity warnings are listed below.</p>';
    const row=(label,value)=>`<div class="row"><span>${safe(label)}</span><span>${safe(value)}</span></div>`;
    $('settings').innerHTML=mesh.options ?
      row('Interior algorithm',['Ear clipping','Constrained Delaunay'][mesh.options.algorithm])+
      row('Boundary treatment',['Fixed','Split polygon edges','Refine on curves'][mesh.options.boundaryTreatment])+
      row('Preserve regions',mesh.options.preserveRegions?'yes':'no')+
      row('Preserve corners',mesh.options.boundary.preserveCorners?'yes':'no')+
      row('Boundary maximum edge',mesh.options.boundary.maxEdgeLength ? number(mesh.options.boundary.maxEdgeLength) : 'disabled')+
      row('Maximum curve deviation',mesh.options.boundary.maxCurveDeviation ? number(mesh.options.boundary.maxCurveDeviation) : 'disabled')+
      row('Refinement maximum edge',mesh.options.refinement.maxEdgeLength ? number(mesh.options.refinement.maxEdgeLength) : 'disabled')+
      row('Refinement step limit',mesh.options.refinement.maxSteps)+
      row('Protected junctions',mesh.boundaryContext?.protectedVertices.length ?? 0)+
      row('Stored source curves',mesh.boundaryContext?.curves.length ?? 0) :
      '<p class="muted">Settings unavailable in v1/v2 files.</p>';
    $('report').innerHTML=mesh.report ?
      row('Size target satisfied',mesh.report.sizeSatisfied?'reported satisfied':'reported unsatisfied')+
      row('Constrained Delaunay',''+(mesh.report.delaunayVerified?'reported verified':'not verified'))+
      row('Refinement steps',mesh.report.refinementSteps)+
      `<p class="report-message">${safe(mesh.report.message)}</p><p class="muted">Stored mesher report; the viewer does not independently verify size or Delaunay guarantees.</p>` :
      '<p class="muted">No mesher guarantees stored in this file.</p>';
    $('diagnostics').innerHTML=analysis.issues.length?`<div class="row"><span>Issues found</span><b>${analysis.issues.length}</b></div><div class="issue-list">${analysis.issues.slice(0,100).map(x=>`<div class="issue-item">${safe(x)}</div>`).join('')}${analysis.issues.length>100?`<p class="muted">First 100 shown.</p>`:''}</div>`:'<p class="ok">No structural issues detected.</p>';
  }
  function inspect() {
    if(selected<0){$('inspector').innerHTML='<p class="muted">Select a triangle in the canvas.</p>';return;}
    const t=analysis.triangles[selected];
    const row=(label,value)=>`<div class="row"><span>${label}</span><span>${safe(value)}</span></div>`;
    const label=analysis.problemTriangles.has(t.id)?`${t.status} geometry; connectivity issue`:t.status;
    let html=`<h2>Triangle ${t.id} <span class="chip ${t.status==='valid'&&!analysis.problemTriangles.has(t.id)?'':'bad'}">${label}</span></h2>`;
    html+=row('Corner indices',t.nodes.slice(0,3).join(', '));
    html+='<div class="subtitle">Coordinates</div><div class="code-list">'+t.nodes.map((id,i)=>`corner ${i}: node ${id} = (${number(mesh.nodes[id][0])}, ${number(mesh.nodes[id][1])})`).join('<br>')+'</div>';
    html+='<div class="subtitle">Edges and neighbours</div>';
    t.edgeKeys.forEach((k,i)=>{
      const edge=analysis.edges.get(k), other=edge.owners.filter(id=>id!==t.id);
      html+=row(`${['01','12','20'][i]}: ${t.nodes[i]}–${t.nodes[(i+1)%3]}`,
        `length ${number(t.lengths[i])}; ${other.length?`triangle ${other.join(', ')}`:'boundary'}`);
    });
    html+='<div class="subtitle">Shape</div>';
    html+=row('Signed area',number(t.area));
    html+=row('Orientation',t.status==='valid'?'counterclockwise':t.status==='inverted'?'clockwise':'zero area');
    html+=row('Mean-ratio quality',number(t.quality));
    html+=row('Minimum angle',`${number(Math.min(...t.angles))}°`);
    html+=row('Edge ratio',number(t.aspect));
    $('inspector').innerHTML=html;
  }
  function load(source,name){
    try{const candidate=parseMesh(source), diagnostic=analyse(candidate);
      mesh=candidate;analysis=diagnostic;selected=-1;
      $('filename').textContent=name;clearError();displaySummary();inspect();fit();
    }catch(error){showError(`Could not load ${name}: ${error.message}`);}
  }
  const hit=(x,y)=>{
    const p=world(x,y), tol=1e-11;
    for(let id=analysis.triangles.length-1;id>=0;id--){
      const e=mesh.elements[id],a=mesh.nodes[e[0]],b=mesh.nodes[e[1]],c=mesh.nodes[e[2]];
      const cross=(u,v)=>u[0]*v[1]-u[1]*v[0];
      const sub=(u,v)=>[u[0]-v[0],u[1]-v[1]];
      const d=cross(sub(b,a),sub(c,a));if(d===0)continue;
      const u=cross(sub(b,p),sub(c,p))/d,v=cross(sub(c,p),sub(a,p))/d,w=1-u-v;
      if(u>=-tol&&v>=-tol&&w>=-tol)return id;
    }
    // A collapsed triangle has no interior. Pick its edge within six screen pixels.
    let best=-1, closest=6;
    for(const t of analysis.triangles) if(t.status==='degenerate') {
      for(let i=0;i<3;i++) {
        const a=screen(mesh.nodes[t.nodes[i]]),b=screen(mesh.nodes[t.nodes[(i+1)%3]]);
        const dx=b[0]-a[0],dy=b[1]-a[1],length2=dx*dx+dy*dy;
        const u=length2?Math.max(0,Math.min(1,((x-a[0])*dx+(y-a[1])*dy)/length2)):0;
        const distance=Math.hypot(x-a[0]-u*dx,y-a[1]-u*dy);
        if(distance<=closest){closest=distance;best=t.id;}
      }
    }
    return best;
  };
  canvas.addEventListener('pointerdown',e=>{canvas.setPointerCapture(e.pointerId);pointer={x:e.offsetX,y:e.offsetY,startX:e.offsetX,startY:e.offsetY};moved=false;wrap.classList.add('dragging');});
  canvas.addEventListener('pointermove',e=>{if(!pointer)return;const dx=e.offsetX-pointer.x,dy=e.offsetY-pointer.y;if(Math.hypot(e.offsetX-pointer.startX,e.offsetY-pointer.startY)>3)moved=true;view.x-=dx*view.s;view.y+=dy*view.s;pointer.x=e.offsetX;pointer.y=e.offsetY;draw();});
  canvas.addEventListener('pointerup',e=>{if(!moved&&analysis){selected=hit(e.offsetX,e.offsetY);inspect();draw();}pointer=null;wrap.classList.remove('dragging');});
  canvas.addEventListener('pointercancel',()=>{pointer=null;wrap.classList.remove('dragging');});
  canvas.addEventListener('wheel',e=>{e.preventDefault();if(!mesh)return;const before=world(e.offsetX,e.offsetY);view.s*=Math.exp(e.deltaY*.001);view.s=Math.min(Math.max(view.s,1e-12),1e12);view.x=before[0]-e.offsetX*view.s;view.y=before[1]+e.offsetY*view.s;draw();},{passive:false});
  $('fit').onclick=fit;
  checks.forEach(id=>$(id).addEventListener('change',draw));
  $('open').onclick=()=>$('file').click();
  $('file').addEventListener('change',async e=>{const file=e.target.files[0];if(file)load(await file.text(),file.name);e.target.value='';});
  wrap.addEventListener('dragover',e=>e.preventDefault());
  wrap.addEventListener('drop',async e=>{e.preventDefault();const file=e.dataTransfer.files[0];if(file)load(await file.text(),file.name);});
  new ResizeObserver(resize).observe(wrap);
  resize();
  if(window.exampleMesh)load(window.exampleMesh,'Example: rectangle P1');
  else showError('Example mesh is missing. Open a .pde2d file to start.');
})();
