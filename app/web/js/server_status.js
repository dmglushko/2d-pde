// What the server is busy with, said in the top bar while this page
// waits for it: "server busy: meshing for another tab (12 s)". While a
// request that waits for the server's session has waited a moment, or a
// long stage of this page runs, the page asks GET /api/status (it never
// waits for the session) every second; the line hides once nothing
// waits. A user in another tab sees why their requests wait.

import { api, oldestWait } from "./api.js";
import { serverBusyText } from "./pure/requests.js";
import { store } from "./state.js";

// A request that has waited this long (ms) is worth explaining.
const SHOW_AFTER_MS = 600;
const POLL_MS = 1000;

export function watchServer(element) {
  let asking = false;
  const hide = () => {
    element.hidden = true;
    element.textContent = "";
  };
  const tick = async () => {
    const waited = oldestWait();
    const waiting = waited !== null && waited >= SHOW_AFTER_MS;
    if (!waiting || asking) {
      if (!waiting) hide();
      return;
    }
    asking = true;
    try {
      const status = await api.status();
      const text = serverBusyText(status);
      // Nothing to explain when the server is busy with exactly what this
      // page asked for and shows itself ("Meshing… 3 s" in its step).
      const own = status?.busy?.yours && [...store.busy].some((stage) => TASK_OF_STAGE[stage]?.includes(status.busy.task));
      if (text && !own && oldestWait() !== null) {
        element.textContent = text;
        element.hidden = false;
      } else hide();
    } catch {
      hide();
    } finally {
      asking = false;
    }
  };
  setInterval(tick, POLL_MS);
}

// The server's tasks (GET /api/status) of each stage of the page. A chunk
// of this page's own run ("run") is not among them: other requests wait
// for it, which is worth saying.
const TASK_OF_STAGE = { geometry: ["geometry"], mesh: ["mesh", "mesh arrays"], problem: ["problem check"], assemble: ["assembly"], field: ["field"], solve: ["solve"] };
