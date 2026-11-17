// A refinement study: the levels of the mesh setting it solves on, and
// the observed orders of convergence between them. Pure: tested in Node
// (app/tests/web.test.mjs).

// The mesh size bound of each level: the case's value, halved level by
// level ({absolute, relative}: the one that is set is halved; the
// relative one when both are).
export function studyLevels(bound, count) {
  const key = bound?.relative !== null && bound?.relative !== undefined ? "relative" : bound?.absolute !== null && bound?.absolute !== undefined ? "absolute" : null;
  if (!key || !(bound[key] > 0)) return null;
  return Array.from({ length: count }, (_, i) => ({ key, value: bound[key] / 2 ** i }));
}

// The observed order of an error between consecutive rows:
// log(e_prev / e) / log(h_prev / h), with h each row's mesh size. Null
// for the first row, where h did not change, or where an error is not a
// positive number.
export function observedOrders(rows, errorOf, sizeOf = (row) => row.h) {
  return rows.map((row, i) => {
    if (i === 0) return null;
    const previous = rows[i - 1];
    const [e0, e1, h0, h1] = [errorOf(previous), errorOf(row), sizeOf(previous), sizeOf(row)];
    if (![e0, e1, h0, h1].every((v) => Number.isFinite(v) && v > 0) || h0 === h1) return null;
    return Math.log(e0 / e1) / Math.log(h0 / h1);
  });
}

// The values of a reference slope h^order through the point (h, e) at
// the sizes `hs`: a straight line of that slope on a log–log chart.
export function referenceSlope(hs, order, h, e) {
  return Array.from(hs, (x) => e * (x / h) ** order);
}

// The rows of an adaptive solve's cycle table: the server's cycles
// ({cycle, nodes, triangles, estimate, marked}) with, when the solve was
// compared with an exact solution, each cycle's errors (`compared`: the
// comparison's "cycles", [{cycle, max_abs, l2}]; null otherwise), and the
// observed rates against the number of nodes N: e ~ N^(-rate), so
// rate = log(e_prev / e) / log(N / N_prev) (null where it cannot be
// computed). An energy-norm error falls like N^(-1/2) on a well-adapted
// mesh of P1 elements, an L2 error like N^(-1).
export function adaptiveRows(cycles, compared = null) {
  const byCycle = new Map((compared ?? []).map((c) => [c.cycle, c]));
  const rows = (cycles ?? []).map((c) => {
    const error = byCycle.get(c.cycle);
    return {
      cycle: c.cycle,
      nodes: c.nodes,
      triangles: c.triangles,
      estimate: c.estimate ?? null,
      marked: c.marked ?? null,
      max: error ? (error.max_abs ?? null) : null,
      l2: error ? (error.l2 ?? null) : null,
    };
  });
  const perNode = (row) => 1 / row.nodes;
  const rates = (key) => observedOrders(rows, (row) => row[key], perNode);
  const estimate = rates("estimate");
  const l2 = rates("l2");
  return rows.map((row, i) => ({ ...row, estimateRate: estimate[i], l2Rate: l2[i] }));
}
