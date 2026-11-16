// The case menu of the top bar: New, Open, Save, Restore, Undo, Redo and
// the examples behind one button, so the bar is one row at phone width.
// The menu button pattern: Enter, Space or ArrowDown opens it on the
// first item (ArrowUp on the last); the arrows, Home and End move between
// the items that can be used, Escape closes it and gives the focus back to
// the button, Tab or a click elsewhere closes it, and choosing an item
// closes it.

export function wireCaseMenu(button, menu) {
  const items = () => [...menu.querySelectorAll('[role="menuitem"]')].filter((item) => !item.disabled && !item.hidden);
  const isOpen = () => !menu.hidden;
  const open = (focus = "first") => {
    menu.hidden = false;
    button.setAttribute("aria-expanded", "true");
    const list = items();
    (focus === "last" ? list.at(-1) : list[0])?.focus();
  };
  const close = ({ refocus = true } = {}) => {
    if (!isOpen()) return;
    menu.hidden = true;
    button.setAttribute("aria-expanded", "false");
    if (refocus) button.focus();
  };
  button.addEventListener("click", () => (isOpen() ? close() : open()));
  button.addEventListener("keydown", (event) => {
    if (event.key === "ArrowDown" || event.key === "Enter" || event.key === " ") {
      event.preventDefault();
      open("first");
    } else if (event.key === "ArrowUp") {
      event.preventDefault();
      open("last");
    }
  });
  menu.addEventListener("keydown", (event) => {
    const list = items();
    const i = list.indexOf(document.activeElement);
    const move = { ArrowDown: i + 1, ArrowUp: i - 1, Home: 0, End: list.length - 1 }[event.key];
    if (move !== undefined) {
      event.preventDefault();
      list[(move + list.length) % list.length]?.focus();
    } else if (event.key === "Escape") {
      event.preventDefault();
      close();
    } else if (event.key === "Tab") close({ refocus: false });
  });
  // An item chosen: the menu closes (the item's own handler runs first).
  menu.addEventListener("click", (event) => {
    if (event.target.closest('[role="menuitem"]')) close();
  });
  document.addEventListener("pointerdown", (event) => {
    if (isOpen() && !menu.contains(event.target) && !button.contains(event.target)) close({ refocus: false });
  });
}
