// A refinement study: the case solved on k meshes, the mesh size bound
// halved level by level, each solution compared with the exact solution
// in the case (view.exact_solution) on the server, in double. The page's
// existing actions do the work one level after another (runWithPrerequisites,
// compare), each with its tickets; the study only sets the mesh bound
// between them, as edits the page makes by itself (no undo step, the case
// stays as saved or not), and puts the case's own value back at the end,
// when stopped or when a level fails; after a whole study the case's own
// level is solved again, so the result shown is current. A change the
// user makes to the case meanwhile stops the study (and the user's mesh
// value is kept).

import { runWithPrerequisites, stopRun, followedRun } from "./actions.js";
import { compare, comparisons } from "./compare.js";
import { observedOrders, studyLevels } from "./pure/study.js";
import { editCase, freshResult, notify, sectionRevision, store } from "./state.js";

const emptyStudy = () => ({ running: false, stopping: false, levels: 0, rows: [], message: null, failed: false, expression: null, caseId: null });

// rows: {level, bound ({key, value}), nodes, h (the longest edge), max
// (the largest nodal error over the snapshots), l2 (the largest lumped
// L2 error)}.
export const study = emptyStudy();

const setBound = (bound) => editCase((c) => (c.mesh.spec.max_edge_length = structuredClone(bound)), "edit", { sections: ["mesh"], automatic: true });

// The observed orders of the rows: {max, l2} per row (null for the
// first).
export function studyOrders(rows = study.rows) {
  const max = observedOrders(rows, (r) => r.max);
  const l2 = observedOrders(rows, (r) => r.l2);
  return rows.map((_, i) => ({ max: max[i], l2: l2[i] }));
}

// Runs the study with `levels` meshes. Resolves when it ends (done,
// stopped or failed); study.message says how.
export async function runStudy(levels) {
  if (study.running) return;
  const expression = (store.caseData.view?.exact_solution ?? "").trim();
  const original = structuredClone(store.caseData.mesh?.spec?.max_edge_length ?? null);
  const plan = studyLevels(original, levels);
  Object.assign(study, emptyStudy(), { levels, expression, caseId: store.caseId });
  if (!expression || !plan) {
    study.failed = true;
    study.message = !expression ? "Write an exact solution first: the study compares each level with it." : "The study halves the mesh's max edge length: set it (absolute or relative) in the Mesh step.";
    notify("study");
    return;
  }
  study.running = true;
  notify("study");
  // What the user may change meanwhile: any of these stops the study.
  const others = () => `${store.caseId}|${sectionRevision("geometry", "problem", "solve", "assembly")}|${(store.caseData.view?.exact_solution ?? "").trim()}`;
  const before = others();
  let ours = null;
  const userChanged = () => others() !== before || (ours !== null && sectionRevision("mesh") !== ours);
  let restore = true;
  try {
    for (const [level, bound] of plan.entries()) {
      if (study.stopping) break;
      setBound({ ...original, [bound.key]: bound.value });
      ours = sectionRevision("mesh");
      const ok = await runWithPrerequisites("solve");
      if (userChanged()) {
        restore = others().startsWith(`${study.caseId}|`) && sectionRevision("mesh") === ours;
        study.message = "Stopped: the case changed during the study.";
        break;
      }
      if (study.stopping) break;
      if (ok !== true) {
        study.failed = true;
        study.message = `Level ${level + 1} could not be solved; the study stopped (see the step that opened).`;
        break;
      }
      await compare("exact", { exact: expression });
      const c = comparisons.exact;
      if (userChanged()) {
        restore = others().startsWith(`${study.caseId}|`) && sectionRevision("mesh") === ours;
        study.message = "Stopped: the case changed during the study.";
        break;
      }
      if (study.stopping) break;
      if (c.error || !c.summary) {
        study.failed = true;
        study.message = `Level ${level + 1} could not be compared: ${c.error?.message ?? "no figures"}.`;
        break;
      }
      const mesh = freshResult("mesh");
      study.rows.push({ level, bound, nodes: mesh?.stats?.nodes ?? null, h: mesh?.longest_edge ?? null, max: c.summary.max_abs, l2: c.summary.max_l2 });
      notify("study");
    }
  } finally {
    if (study.stopping && !study.message) study.message = `Stopped after ${study.rows.length} of ${levels} levels.`;
    if (!study.message) study.message = `Done: ${study.rows.length} levels.`;
    // The case's own mesh setting is back (unless the user set another),
    // and after a whole study its solution is solved again (the first,
    // coarsest level: the cheapest), so what is shown is current.
    if (restore && store.caseId === study.caseId) {
      setBound(original);
      if (study.rows.length === levels && !study.stopping) {
        study.message = `Done: ${levels} levels. Solving the case's own mesh again…`;
        notify("study");
        await runWithPrerequisites("solve");
        study.message = `Done: ${levels} levels; the case's own mesh is solved again.`;
      }
    }
    study.running = false;
    study.stopping = false;
    notify("study");
  }
}

// Stops the study after the level that is running (its run stops now).
export function stopStudy() {
  if (!study.running) return;
  study.stopping = true;
  if (followedRun()) stopRun();
  notify("study");
}

// Forgets the study's table (another case).
export function clearStudy() {
  if (study.running) return;
  Object.assign(study, emptyStudy());
}
