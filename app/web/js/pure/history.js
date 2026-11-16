// Undo and redo of case edits. A step holds, for each case section the
// edit changed, the section's JSON text before and after (sections an
// edit did not change cost nothing, so a large SVG or mesh file is kept
// only by the steps that changed it). Typing in one field is one step:
// an edit with the same `group` as the step on top merges into it until
// the step is sealed (the field was left, or an undo or redo ran). At
// most `limit` steps are kept; the oldest go first. Pure.

export class History {
  constructor({ limit = 100 } = {}) {
    this.limit = limit;
    this.undoSteps = [];
    this.redoSteps = [];
    this.sealed = true;
  }

  // An edit changed `before` to `after` ({section: text}, the same keys;
  // sections whose text did not change are left out). `group` (a field's
  // path) merges typing into one step. Returns false when nothing changed.
  record(before, after, group = null) {
    const changed = Object.keys(after).filter((s) => before[s] !== after[s]);
    if (changed.length === 0) return false;
    this.redoSteps = [];
    const top = this.undoSteps.at(-1);
    if (!this.sealed && group !== null && top && top.group === group) {
      for (const s of changed) {
        if (!(s in top.before)) top.before[s] = before[s];
        top.after[s] = after[s];
      }
      // Typing back to where the step started leaves nothing to undo.
      if (Object.keys(top.after).every((s) => top.after[s] === top.before[s])) {
        this.undoSteps.pop();
        this.sealed = true;
      }
      return true;
    }
    const step = { group, before: {}, after: {} };
    for (const s of changed) {
      step.before[s] = before[s];
      step.after[s] = after[s];
    }
    this.undoSteps.push(step);
    if (this.undoSteps.length > this.limit) this.undoSteps.shift();
    this.sealed = group === null;
    return true;
  }

  // Ends the step on top: the next edit, even in the same field, is a new
  // step.
  seal() {
    this.sealed = true;
  }

  canUndo() {
    return this.undoSteps.length > 0;
  }

  canRedo() {
    return this.redoSteps.length > 0;
  }

  // The sections to put back ({section: text}) to undo the last step, or
  // null.
  undo() {
    const step = this.undoSteps.pop();
    if (!step) return null;
    this.redoSteps.push(step);
    this.sealed = true;
    return { ...step.before };
  }

  // The sections to put back to redo the step undone last, or null.
  redo() {
    const step = this.redoSteps.pop();
    if (!step) return null;
    this.undoSteps.push(step);
    this.sealed = true;
    return { ...step.after };
  }

  clear() {
    this.undoSteps = [];
    this.redoSteps = [];
    this.sealed = true;
  }
}
