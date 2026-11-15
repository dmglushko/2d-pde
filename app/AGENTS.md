# Agent guide: app/

The graphical interface of pde2d. Read [docs/app.md](../docs/app.md)
first (shape, case format, API, catalog, decisions), then
[README.md](README.md) (how it is used).

## You decide the app's design

**The owner has delegated the app (2026-11-14).** Inside `app/` (and
`docs/app.md`), agents make the design decisions themselves: layout,
case format, API, dependencies of the app, what the page offers. You do
not need to ask the owner first. Record each decision with its reason in
the "Decisions" list of `docs/app.md`.

The delegation stops at the library. The root [AGENTS.md](../AGENTS.md)
still holds for everything outside `app/`: the library's architecture
belongs to the owner. If the app needs something the library does not
offer, raise it as a library change; do not work around the library's
rules in the app, and never reimplement library logic (checks, physics,
meshing) in the app or the page.

## Rules that keep the app honest

- **The library is the only source of truth.** The app forwards the
  library's errors with the case path in front; it does not check ranges
  or physics itself.
- **Components come from the catalog** (`src/catalog.cpp`): a `type` is
  the component's `name()`, defaults are read from the library's options
  structs. The page must not name components.
- **Strict reading**: an unknown key in a case is an error.
- **One thread for symbolic work**: every `Session` call runs under its
  mutex. Do not add work that builds or drops `Expression`s outside it.
- **`httplib.h` only in `src/server.cpp`.** The library never includes
  anything from `app/`.
- **The page has no build step**: plain ES modules. Pure helpers go to
  `web/js/core.js` and get a test in `tests/web.test.mjs`.

## Before finishing a change

1. `tools/format.sh --check` (C++ of the app included).
2. `tools/test.sh`: the app's tests are `app_session`, `app_solve`,
   `app_server`, `app_run_*` (every case in `examples/`), `app_web` and
   `app_web_actions` (Node).
3. For a page change, open it in a browser and use the changed step.
4. Update `docs/app.md` (and `README.md` if use changes).
