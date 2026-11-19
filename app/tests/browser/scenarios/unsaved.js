// The page's own edits (conditions after a geometry, normalization) do
// not make a case "unsaved" or an undo step.
export default async ({ example, q, click, until, text, check }) => {
  example("plate_with_hole.json");
  await until(() => text(q("#case-name")).includes("plate_with_hole"), "the example");
  await until(() => q("#geometry-summary .stats"), "its geometry");
  click(q("#run-all"));
  await until(() => text(q("#toast")).includes("up to date") || /failed/.test(text(q("#toast"))), "Run all", 90000);
  check("no unsaved changes after Run all", q("#unsaved").hidden);
  check("nothing to undo", q("#undo").disabled);
};
