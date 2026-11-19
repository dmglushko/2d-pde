// What the user can hide or size around the picture, remembered in this
// browser: the steps panel (◧ in the top bar, wide screens), the view
// options above the picture (☰ beside the zoom buttons) and the panel of
// the probes and the line profile under it, whose height follows its
// header (drag it, or the arrow keys; dragged low enough it hides, and
// Show or a drag upwards brings it back). The picture follows through
// its resize observer.

const KEYS = {
  sidebar: "pde2d-app-sidebar-hidden",
  toolbar: "pde2d-app-toolbar-hidden",
  extraHidden: "pde2d-app-extra-hidden",
  extraHeight: "pde2d-app-extra-height",
};

// Below this height a dragged panel hides; it is never made smaller
// than EXTRA_MIN while shown.
const EXTRA_HIDE_BELOW = 56;
const EXTRA_MIN = 90;
const EXTRA_DEFAULT = 240;
// The picture keeps at least this much of the column.
const PICTURE_ROOM = 300;

function read(key) {
  try {
    return localStorage.getItem(key);
  } catch {
    return null;
  }
}

function write(key, value) {
  try {
    if (value === null) localStorage.removeItem(key);
    else localStorage.setItem(key, String(value));
  } catch {
    // Not kept: it holds for this visit.
  }
}

const $ = (id) => document.getElementById(id);

// A toggle button that hides `target` (by the class `hiddenClass` on
// `holder`), with its pressed state, title and kept choice.
function wireToggle(button, { key, holder, hiddenClass, titles }) {
  const apply = (hidden) => {
    holder.classList.toggle(hiddenClass, hidden);
    button.setAttribute("aria-pressed", String(!hidden));
    button.title = hidden ? titles.show : titles.hide;
    write(key, hidden ? "1" : null);
  };
  apply(read(key) === "1");
  button.addEventListener("click", () => apply(button.getAttribute("aria-pressed") === "true"));
}

export function wirePanes() {
  wireToggle($("toggle-sidebar"), {
    key: KEYS.sidebar,
    holder: document.querySelector(".layout"),
    hiddenClass: "no-sidebar",
    titles: { hide: "Hide the steps panel (the picture takes the room)", show: "Show the steps panel" },
  });
  wireToggle($("toggle-toolbar"), {
    key: KEYS.toolbar,
    holder: $("viewer-toolbar"),
    hiddenClass: "folded",
    titles: { hide: "Hide the view options above the picture", show: "Show the view options above the picture" },
  });
  wireExtra();
}

function wireExtra() {
  const panel = $("probes");
  const handle = $("extra-handle");
  const toggle = $("extra-toggle");
  const root = document.documentElement;
  let height = Number(read(KEYS.extraHeight)) || EXTRA_DEFAULT;
  const most = () => Math.max(EXTRA_MIN, (panel.parentElement?.clientHeight || window.innerHeight) - PICTURE_ROOM);
  const show = (hidden) => {
    panel.classList.toggle("collapsed", hidden);
    toggle.textContent = hidden ? "Show" : "Hide";
    toggle.setAttribute("aria-expanded", String(!hidden));
    write(KEYS.extraHidden, hidden ? "1" : null);
  };
  const size = (value) => {
    height = Math.round(Math.min(most(), Math.max(EXTRA_MIN, value)));
    root.style.setProperty("--extra-height", `${height}px`);
    handle.setAttribute("aria-valuenow", String(height));
    handle.setAttribute("aria-valuemax", String(Math.round(most())));
    write(KEYS.extraHeight, height);
  };
  size(height);
  show(read(KEYS.extraHidden) === "1");
  toggle.addEventListener("click", () => show(!panel.classList.contains("collapsed")));

  // Dragging the header: up makes the panel taller; down, below
  // EXTRA_HIDE_BELOW, hides it (its height is kept for Show).
  let start = null;
  handle.addEventListener("pointerdown", (event) => {
    // On a phone the panel follows its content (layout.css): no drag.
    if (event.button !== 0 || window.matchMedia("(max-width: 860px)").matches) return;
    const collapsed = panel.classList.contains("collapsed");
    start = { y: event.clientY, height: collapsed ? 0 : height, before: height };
    try {
      handle.setPointerCapture(event.pointerId);
    } catch {
      // A synthetic pointer.
    }
    event.preventDefault();
  });
  handle.addEventListener("pointermove", (event) => {
    if (!start) return;
    const wanted = start.height + start.y - event.clientY;
    if (wanted < EXTRA_HIDE_BELOW) {
      show(true);
      size(start.before);
    } else {
      show(false);
      size(wanted);
    }
  });
  const end = () => (start = null);
  handle.addEventListener("pointerup", end);
  handle.addEventListener("pointercancel", end);
  handle.addEventListener("keydown", (event) => {
    const delta = { ArrowUp: 24, ArrowDown: -24 }[event.key];
    if (!delta) return;
    event.preventDefault();
    if (panel.classList.contains("collapsed")) {
      if (delta > 0) show(false);
      return;
    }
    if (delta < 0 && height + delta < EXTRA_MIN) return show(true);
    size(height + delta);
  });
  window.addEventListener("resize", () => size(height));
}
