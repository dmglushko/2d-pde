// An option the library refuses is marked on its field.
export default async ({ q, tab, panel, button, click, type, until, text, check }) => {
  tab("mesh");
  const angle = q('[data-path="mesh.refiner.options.min_angle_degrees"] input');
  check("the refiner's angle field exists", Boolean(angle));
  type(angle, "50");
  click(button("Mesh", panel("mesh")));
  const marked = await until(() => q("#panel-mesh .field.invalid"), "the marked field", 30000);
  check("the mark is on the option", marked.dataset.path === "mesh.refiner.options.min_angle_degrees", marked.dataset.path);
  check("the message is under it", text(marked.querySelector(".error-text")).length > 0);
  check("the Mesh step says failed", /Failed/.test(text(q("#panel-mesh .step-status"))), text(q("#panel-mesh .step-status")));
  type(angle, "25");
  await until(() => !q("#panel-mesh .field.invalid"), "the mark to clear");
  check("an edit clears the mark", !q("#panel-mesh .field.invalid"));
};
