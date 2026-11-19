// Undo and redo of case edits: typing in one field is one step; a field
// typed in keeps Ctrl+Z for itself; Restore previous case is in the top
// bar after a replacement.
export default async ({ d, q, tab, type, key, click, until, text, check }) => {
  tab("problem");
  const field = () => q('[data-path="problem.equation.text"] input');
  const original = field().value;
  check("nothing to undo at the start", q("#undo").disabled);
  field().focus();
  for (const value of ["-lap(u) = 1", "-lap(u) = 12", "-lap(u) = 123"]) type(field(), value);
  // While the field was typed in, Ctrl+Z is the field's own.
  key(field(), "z", { ctrlKey: true });
  check("Ctrl+Z in a field being typed in is the field's", field().value === "-lap(u) = 123");
  field().blur();
  const k = q('input[data-path^="problem.parameters."]');
  if (k) {
    k.focus();
    type(k, "7");
    k.blur();
  }
  check("undo is offered", !q("#undo").disabled);
  key(d.body, "z", { ctrlKey: true });
  if (k) {
    await until(() => q('input[data-path^="problem.parameters."]').value !== "7", "the parameter back");
    key(d.body, "z", { ctrlKey: true });
  }
  await until(() => field().value === original, "the typing undone in one step");
  check("typing in one field is one step", field().value === original, field().value);
  key(d.body, "z", { ctrlKey: true, shiftKey: true });
  await until(() => field().value === "-lap(u) = 123", "redo");
  check("Ctrl+Shift+Z redoes", true);
  click(q("#undo"));
  await until(() => field().value === original, "the undo button");
  check("the Undo button undoes", true);
  click(q("#redo"));
  await until(() => field().value === "-lap(u) = 123", "the redo button");
  // A replaced case comes back from the top bar, not from undo.
  click(q("#new-case"));
  await until(() => !q("#restore-case").hidden, "Restore previous case");
  check("a new case has nothing to undo", q("#undo").disabled);
  click(q("#restore-case"));
  await until(() => field()?.value === "-lap(u) = 123", "the previous case back");
  check("Restore previous case brings the edited case back", field()?.value === "-lap(u) = 123");
};
