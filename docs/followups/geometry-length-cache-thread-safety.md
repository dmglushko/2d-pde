# Geometry length-cache thread-safety follow-up

Status: deferred; this document is a plan, not an implementation.

## Problem

CurveGeometry::length(CurveId) is const but lazily inserts into the mutable lengths_ map in cpp/discretization/curve_geometry.hpp. Two threads querying the same geometry can race when a cache miss overlaps another query. Const access alone is therefore not a concurrency guarantee. Region-length queries and splitting also use this cache.

Concurrent mutation of geometry is a separate contract and is not implicitly supported by this proposal.

## Design decision to make

Choose and document one approach before implementation: synchronized cache access; immutable/eager measurements; or an explicit externally synchronized/single-threaded geometry contract. Prefer safe concurrent read-only queries if their cost and value justify it. Do not add a mutex without addressing copying, moving, transactional edits, and resize replay.

## Proposed work

1. Define supported concurrency: independent geometry copies, concurrent read-only queries on one object, and access during mutation.
2. Implement the selected policy with correct cache invalidation on resize/replacement and preservation through relabeling and rigid transforms.
3. Preserve geometry-local curve IDs, copy independence, replay behavior, and strong exception guarantees.
4. Document reference lifetimes from curves()/curve() when mutations occur; locking a cache does not make returned references safe during edits.

## Acceptance criteria

- Add tests covering simultaneous first-use length/regionLength queries, repeated cache hits, and independent copied geometries for the chosen contract.
- Run a meaningful ThreadSanitizer concurrency harness when supported; distinguish an unavailable runtime from a passing run.
- Full tools/test.sh and appropriate AddressSanitizer/UndefinedBehaviorSanitizer checks pass.
- Resize replay, rollback, curve splitting, renaming, replacement, copy/move behavior, and numerical length accuracy remain correct.
- Document supported concurrency and measure any material query/copy cost before making performance claims.

Implementation and policy selection remain for a future session.
