// The case menu: keyboard use (the menu button pattern), its items, and
// toasts that cover none of the picture's controls.
export default async ({ d, w, q, qa, key, until, text, check }) => {
  const button = q("#case-menu-button");
  const menu = q("#case-menu");
  button.focus();
  key(button, "ArrowDown");
  check("ArrowDown opens the menu", !menu.hidden && button.getAttribute("aria-expanded") === "true");
  check("on its first item", d.activeElement === q("#new-case"), d.activeElement?.id);
  key(menu, "ArrowDown");
  check("ArrowDown moves on", d.activeElement === q("#open-case-button"), d.activeElement?.id);
  key(menu, "End");
  check("End goes to the last item", d.activeElement === qa("#examples [role=menuitem]").at(-1), d.activeElement?.textContent);
  key(menu, "Escape");
  check("Escape closes it and gives the focus back", menu.hidden && d.activeElement === button);
  // An example from the keyboard: open, move to it, Enter (a click).
  key(button, "Enter");
  const item = q('[data-example="wave_square.json"]');
  item.focus();
  item.click();
  check("choosing an item closes the menu", menu.hidden);
  await until(() => text(q("#toast")).includes("wave_square"), "the example");
  // At phone width the name gives way (the bar is one row).
  if (q("#case-name").offsetParent) check("the case's name is on the menu button", /wave_square/.test(text(button)), text(button));
  const toast = q("#toast").getBoundingClientRect();
  const overlaps = (a, b) => a.left < b.right && b.left < a.right && a.top < b.bottom && b.top < a.bottom;
  const covered = [".viewer-main", ".viewer-extra", ".step-head"].filter((s) => q(s)?.offsetParent && overlaps(toast, q(s).getBoundingClientRect()));
  check("the toast covers neither the picture nor the step's head", covered.length === 0, covered.join(", "));
  void w;
};
