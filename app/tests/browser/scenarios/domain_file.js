// A domain file that is slow to read does not replace a case opened
// meanwhile: its read takes a ticket.
export default async ({ example, d, w, q, until, sleep, text, check }) => {
  // Every FileReader of the page answers after 1.5 s.
  const Reader = w.FileReader;
  w.FileReader = class extends Reader {
    readAsText(file) {
      w.setTimeout(() => super.readAsText(file), 1500);
    }
  };
  const svg = '<svg xmlns="http://www.w3.org/2000/svg"><path data-region="edge" d="M 0 0 L 1 0 L 1 1 Z"/></svg>';
  const drop = (name, content) => {
    const transfer = new w.DataTransfer();
    transfer.items.add(new w.File([content], name, { type: "image/svg+xml" }));
    q("#canvas-wrap").dispatchEvent(new w.DragEvent("drop", { dataTransfer: transfer, bubbles: true, cancelable: true }));
  };
  drop("slow.svg", svg);
  example("poisson_rectangle.json");
  await until(() => text(q("#toast")).includes("poisson_rectangle"), "the example");
  await sleep(2500);
  const kept = JSON.parse(w.localStorage.getItem("pde2d-app-case") ?? "null");
  check("the late file does not replace the example", kept?.geometry?.source === "shape", kept?.geometry?.source);
  check("the example's name stays", /poisson_rectangle/.test(text(q("#case-name"))), text(q("#case-name")));
  // Without anything meanwhile, the file is read in.
  drop("slow.svg", svg);
  await until(() => JSON.parse(w.localStorage.getItem("pde2d-app-case") ?? "null")?.geometry?.source === "svg", "the file", 10000);
  check("a file read with nothing meanwhile becomes the geometry", true);
  w.FileReader = Reader;
  void d;
};
