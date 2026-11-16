// One rule for accepting asynchronous answers. A request takes a ticket
// when it starts: its kind ("mesh", "preset", "open", ...), a generation
// (the newest request of its kind has the highest), the scope it was made
// in (the case loaded: store.caseId) and the input key it was made for
// (the stage key, the revision of the sections a preset replaces, the job
// and expression of a comparison). An answer is applied only while its
// ticket still holds: the scope and the key are the same as now and, for
// a kind where a newer request replaces an older one (exclusive, the
// default), no newer ticket of its kind was taken. Pure.

export class Tickets {
  constructor() {
    this.latest = new Map();
  }

  // A ticket for a request of `kind` made in `scope` for `key`.
  issue(kind, scope, key = null, { exclusive = true } = {}) {
    const generation = (this.latest.get(kind) ?? 0) + 1;
    this.latest.set(kind, generation);
    return { kind, generation, scope, key, exclusive };
  }

  // True while no newer request of the ticket's kind was made (always
  // true for a kind that is not exclusive).
  isNewest(ticket) {
    return !ticket.exclusive || this.latest.get(ticket.kind) === ticket.generation;
  }

  // True while the ticket is the newest of its kind and was made in
  // `scope`: its answer may change the state (it may be kept as stale).
  isLive(ticket, scope) {
    return this.isNewest(ticket) && ticket.scope === scope;
  }

  // True while the ticket is live and its input is still `key`: its
  // answer is about what the page shows now.
  holds(ticket, scope, key = null) {
    return this.isLive(ticket, scope) && ticket.key === key;
  }

  // Makes every ticket of `kind` taken so far stale (a request that was
  // cancelled, a run that was stopped).
  supersede(kind) {
    this.latest.set(kind, (this.latest.get(kind) ?? 0) + 1);
  }
}
