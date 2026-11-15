#include "edge_bisection.hpp"

#include "local_improvers.hpp"

#include "pde2d/mesh/domain_size.hpp"

#include <algorithm>
#include <array>
#include <map>
#include <numeric>
#include <set>
#include <stdexcept>
#include <tuple>
#include <utility>
#include <vector>

namespace pde2d::mesh::detail {

using geometry::Point;

namespace {

// An edge by its two nodes. Edges are named by nodes, not by (triangle,
// local index), because splits change the triangles around them.
using NodePair = std::array<std::size_t, 2>;

NodePair undirected(std::size_t a, std::size_t b) {
  return {std::min(a, b), std::max(a, b)};
}

// The nodes of local edge i of triangle t: from vertex i + 1 to i + 2.
NodePair edge_nodes(const WorkingMesh &mesh, std::size_t t, int i) {
  const auto &v = mesh.triangles()[t];
  return {v[static_cast<std::size_t>((i + 1) % 3)],
          v[static_cast<std::size_t>((i + 2) % 3)]};
}

// The local index of a node in a triangle (it must be one of its
// vertices).
int local_vertex(const WorkingMesh &mesh, std::size_t t, std::size_t node) {
  const auto &v = mesh.triangles()[t];
  for (int i = 0; i < 3; ++i)
    if (v[static_cast<std::size_t>(i)] == node)
      return i;
  throw std::logic_error("edge bisection: a node is not in its triangle");
}

// The local edge of t joining a and b: the one opposite its third vertex.
int local_edge(const WorkingMesh &mesh, std::size_t t, std::size_t a,
               std::size_t b) {
  const auto &v = mesh.triangles()[t];
  for (int i = 0; i < 3; ++i) {
    std::size_t w = v[static_cast<std::size_t>(i)];
    if (w != a && w != b)
      return i;
  }
  throw std::logic_error("edge bisection: an edge is not in its triangle");
}

// A strict total order on edges: squared length, then node indices. It
// makes "the longest edge" unique, so the closure of longest edges cannot
// cycle (along a chain of waiting triangles the edges grow strictly).
using EdgeKey = std::tuple<double, std::size_t, std::size_t>;

EdgeKey edge_key(const WorkingMesh &mesh, std::size_t t, int i) {
  auto [a, b] = edge_nodes(mesh, t, i);
  Point d = mesh.nodes()[a] - mesh.nodes()[b];
  return {geometry::dot(d, d), std::min(a, b), std::max(a, b)};
}

int longest_local_edge(const WorkingMesh &mesh, std::size_t t) {
  int best = 0;
  for (int i = 1; i < 3; ++i)
    if (edge_key(mesh, t, i) > edge_key(mesh, t, best))
      best = i;
  return best;
}

double longest_length(const WorkingMesh &mesh, std::size_t t) {
  const auto &v = mesh.triangles()[t];
  const auto &p = mesh.nodes();
  return std::max({geometry::distance(p[v[0]], p[v[1]]),
                   geometry::distance(p[v[1]], p[v[2]]),
                   geometry::distance(p[v[2]], p[v[0]])});
}

std::string boundary_edge_text(const WorkingMesh &mesh, std::size_t k) {
  const auto &e = mesh.boundary_edges()[k];
  return "boundary edge (" + std::to_string(e.nodes[0]) + ", " +
         std::to_string(e.nodes[1]) + ") of region '" +
         mesh.region_names()[e.region] + "'";
}

// What the steps of one refine() or run() did, summed.
struct Counts {
  std::size_t splits = 0;
  std::size_t boundary_splits = 0;
  std::size_t retriangulated = 0;
  std::size_t deleted_nodes = 0;
  std::size_t unsplittable = 0;
  std::size_t incompatible = 0;
  std::size_t red = 0;
  std::size_t red_not_possible = 0;
};

// One refinement step on one mesh, with the bookkeeping it needs: which
// triangles this step bisected (their edge 0 is their bisection edge) and
// the parents of everything it made.
class Step {
public:
  Step(WorkingMesh &mesh, const std::string &name, const BisectionRule &rule,
       std::size_t max_points, Counts &counts, StageReport &report)
      : mesh_(mesh), name_(name), rule_(rule), max_points_(max_points),
        counts_(counts), report_(report), input_nodes_(mesh.nodes().size()),
        input_triangles_(mesh.triangles().size()),
        input_vertices_(mesh.triangles()),
        triangle_parent_(mesh.triangles().size()),
        bisected_(mesh.triangles().size(), false),
        changed_(mesh.triangles().size(), false) {
    std::iota(triangle_parent_.begin(), triangle_parent_.end(), 0);
    node_parent_.reserve(input_nodes_);
    for (std::size_t v = 0; v < input_nodes_; ++v)
      node_parent_.push_back({v, v});
  }

  // 1. The edges of the marked triangles (sorted, without repetitions),
  // by the rule, in triangle order.
  void mark(std::span<const std::size_t> marked) {
    std::set<NodePair> seen;
    for (auto t : marked) {
      std::vector<int> local;
      switch (rule_.marks) {
      case BisectionRule::Marks::refinement_edge:
        local = {0};
        break;
      case BisectionRule::Marks::longest_edge:
        local = {longest_local_edge(mesh_, t)};
        break;
      case BisectionRule::Marks::all_edges:
        local = {0, 1, 2};
        break;
      }
      for (int i : local) {
        auto [a, b] = edge_nodes(mesh_, t, i);
        if (seen.insert(undirected(a, b)).second)
          pending_.push_back({a, b});
      }
    }
  }

  // 2. Splits every marked edge, each after the edges it waits for.
  void split_marked_edges() {
    for (const auto &edge : pending_)
      split_with_closure(edge);
  }

  // 3. The red division of every input triangle whose three edges were
  // all split: its bisection pattern (bisected at edge 0, both children
  // bisected again) differs from red by one flip.
  void make_red() {
    for (std::size_t p = 0; p < input_triangles_; ++p) {
      const auto &v = input_vertices_[p];
      auto node_on = [&](std::size_t a, std::size_t b) {
        auto it = split_node_.find(undirected(a, b));
        return it == split_node_.end() ? no_index : it->second;
      };
      // m_i is the new node on the edge opposite v_i.
      const std::size_t m0 = node_on(v[1], v[2]), m1 = node_on(v[2], v[0]),
                        m2 = node_on(v[0], v[1]);
      if (m0 == no_index || m1 == no_index || m2 == no_index)
        continue;
      switch (try_red(v, m0, m1, m2, p)) {
      case Red::made:
        ++counts_.red;
        break;
      case Red::would_invert:
        ++counts_.red_not_possible;
        break;
      case Red::other_pattern:
        // Divided otherwise (a cycle, or a re-triangulation), already
        // reported.
        break;
      }
    }
  }

  // The live triangles this step created or changed, sorted.
  std::vector<std::size_t> touched() const {
    std::vector<std::size_t> result;
    for (auto t : touched_)
      if (!mesh_.is_triangle_deleted(t))
        result.push_back(t);
    std::sort(result.begin(), result.end());
    result.erase(std::unique(result.begin(), result.end()), result.end());
    return result;
  }

  // Input triangles that were refined although not marked.
  std::size_t refined_by_closure(std::span<const std::size_t> marked) const {
    std::size_t count = 0;
    for (std::size_t p = 0; p < input_triangles_; ++p)
      if (changed_[p] && !std::binary_search(marked.begin(), marked.end(), p))
        ++count;
    return count;
  }

  // The parents, or none (why: parents_lost()).
  std::optional<Parents> parents() const {
    if (!parents_lost_.empty())
      return std::nullopt;
    return Parents{triangle_parent_, node_parent_};
  }
  const std::string &parents_lost() const { return parents_lost_; }

private:
  // Where triangle t is bisected.
  int bisection_edge(std::size_t t) const {
    if (bisected_[t])
      return 0;
    return rule_.first == BisectionRule::FirstBisection::longest_edge
               ? longest_local_edge(mesh_, t)
               : 0;
  }

  // The triangle with edge a-b in either direction, and its local edge;
  // {no_index, -1} once the edge is gone (split, or cut off).
  std::pair<std::size_t, int> find(std::size_t a, std::size_t b) const {
    auto found = mesh_.find_edge(a, b);
    if (found.first == no_index)
      found = mesh_.find_edge(b, a);
    return found;
  }

  // Splits `edge` after every edge it waits for. The stack holds the
  // chain of waiting edges: each one waits for the one above it.
  void split_with_closure(NodePair edge) {
    std::vector<NodePair> stack{edge};
    auto on_stack = [&](NodePair e) {
      return std::any_of(stack.begin(), stack.end(), [&](NodePair s) {
        return undirected(s[0], s[1]) == undirected(e[0], e[1]);
      });
    };
    while (!stack.empty()) {
      auto [a, b] = stack.back();
      auto [t, i] = find(a, b);
      if (t == no_index) {
        stack.pop_back();
        continue;
      }
      // Both sides must be bisected at this edge. A side that must first
      // be bisected elsewhere makes this edge wait for that one.
      bool waits = false, incompatible = false;
      std::array<std::pair<std::size_t, int>, 2> sides{
          std::pair{t, i}, std::pair{mesh_.neighbour(t, i), -1}};
      if (sides[1].first != no_index)
        sides[1].second = local_edge(mesh_, sides[1].first, a, b);
      for (auto [s, k] : sides) {
        if (s == no_index)
          continue;
        const int f = bisection_edge(s);
        if (f == k)
          continue;
        NodePair first = edge_nodes(mesh_, s, f);
        if (on_stack(first)) {
          // A cycle: the triangles wait for each other.
          incompatible = true;
          continue;
        }
        stack.push_back(first);
        waits = true;
        break;
      }
      if (waits)
        continue;
      if (!split(t, i)) {
        // An unsplittable boundary edge: every edge waiting for it stays.
        stack.clear();
        continue;
      }
      if (incompatible)
        ++counts_.incompatible;
      stack.pop_back();
    }
  }

  // Splits edge i of triangle t; false if it is a boundary edge that
  // cannot be split.
  bool split(std::size_t t, int i) {
    if (counts_.splits >= max_points_)
      throw std::runtime_error(name_ + ": inserted max_points = " +
                               std::to_string(max_points_) + " points");
    auto [a, b] = edge_nodes(mesh_, t, i);
    const std::size_t u = mesh_.neighbour(t, i);
    const std::size_t parent_t = triangle_parent_[t];
    std::vector<std::size_t> children;
    if (u != no_index) {
      const std::size_t parent_u = triangle_parent_[u];
      const Point p = geometry::midpoint(mesh_.nodes()[a], mesh_.nodes()[b]);
      std::array<std::size_t, 4> c{};
      try {
        c = mesh_.split_interior_edge(t, i, p);
      } catch (const std::invalid_argument &) {
        throw std::runtime_error(
            name_ + ": triangle " + std::to_string(t) +
            " is too thin to bisect in floating point (the midpoint of its "
            "edge does not lie strictly between its end points)");
      }
      grow();
      // {t, u, the other half of t, the other half of u}.
      triangle_parent_[c[2]] = parent_t;
      triangle_parent_[c[3]] = parent_u;
      mark_changed(parent_t);
      mark_changed(parent_u);
      children.assign(c.begin(), c.end());
    } else {
      const std::size_t k = mesh_.boundary_edge_index(t, i);
      if (mesh_.is_unsplittable(k))
        return false; // reported when it first failed
      const std::string text = boundary_edge_text(mesh_, k);
      auto r = mesh_.split_boundary_edge(k);
      if (!r.split) {
        ++counts_.unsplittable;
        report_.messages.push_back(
            text + " cannot be split: " + r.failure +
            "; it stays as it is, so the triangles that need it are not "
            "refined (and a length target may not be met near it)");
        return false;
      }
      ++counts_.boundary_splits;
      grow();
      if (r.retriangulated) {
        // The policy re-triangulated around the edge: its triangles are
        // no longer children of one input triangle, and the nodes it cut
        // off are gone. Their vertex order is the policy's, so they are
        // bisected like input triangles.
        ++counts_.retriangulated;
        counts_.deleted_nodes += r.deleted_nodes;
        ++counts_.splits;
        if (parents_lost_.empty())
          parents_lost_ = "the boundary split policy re-triangulated around " +
                          text +
                          " (its curve point could not be joined to "
                          "the opposite vertex)";
        mark_changed(parent_t);
        for (auto s : r.touched)
          bisected_[s] = false;
        touched_.insert(touched_.end(), r.touched.begin(), r.touched.end());
        return true;
      }
      // A direct split: {t, its other half}.
      triangle_parent_[r.touched[1]] = parent_t;
      mark_changed(parent_t);
      children = r.touched;
    }
    // The new node is the last one. It becomes vertex 0 of every child,
    // so each child's refinement edge is an edge of the split triangle.
    const std::size_t q = mesh_.nodes().size() - 1;
    for (auto s : children) {
      mesh_.set_refinement_edge(s, local_vertex(mesh_, s, q));
      bisected_[s] = true;
    }
    if (a < input_nodes_ && b < input_nodes_)
      node_parent_[q] = undirected(a, b);
    else if (parents_lost_.empty())
      parents_lost_ = "an edge that was not in the input was split";
    split_node_[undirected(a, b)] = q;
    touched_.insert(touched_.end(), children.begin(), children.end());
    ++counts_.splits;
    return true;
  }

  enum class Red { made, would_invert, other_pattern };

  // The red division of input triangle p = (v0, v1, v2), whose edges were
  // split at m0, m1, m2 (m_i opposite v_i).
  Red try_red(const WorkingMesh::TriangleNodes &v, std::size_t m0,
              std::size_t m1, std::size_t m2, std::size_t p) {
    // The bisection pattern has the edge v0-m0 between (m2, m0, v0) and
    // (m1, v0, m0); red has m1-m2 in its place.
    auto [t, i] = find(v[0], m0);
    if (t == no_index)
      return Red::other_pattern;
    const std::size_t u = mesh_.neighbour(t, i);
    if (u == no_index)
      return Red::other_pattern;
    const std::size_t x = mesh_.triangles()[t][static_cast<std::size_t>(i)];
    const std::size_t y = mesh_.triangles()[u][static_cast<std::size_t>(
        local_edge(mesh_, u, v[0], m0))];
    if (undirected(x, y) != undirected(m1, m2) || triangle_parent_[t] != p ||
        triangle_parent_[u] != p)
      return Red::other_pattern;
    // The four red children must exist with these corners before the
    // flip; the two corner triangles at v1 and v2 already do.
    auto [t1, i1] = mesh_.find_edge(m2, v[1]);
    auto [t2, i2] = mesh_.find_edge(m1, m0);
    if (t1 == no_index || t2 == no_index ||
        mesh_.triangles()[t1][static_cast<std::size_t>(i1)] != m0 ||
        mesh_.triangles()[t2][static_cast<std::size_t>(i2)] != v[2])
      return Red::other_pattern;
    // The quadrilateral v0, m2, m0, m1 is a parallelogram on straight
    // edges; a curve point can make it non-convex, and then the middle red
    // child would be inverted.
    if (!mesh_.is_flippable(t, i))
      return Red::would_invert;
    mesh_.flip(t, i);
    touched_.push_back(t);
    touched_.push_back(u);
    // Each red child lists the images of v0, v1, v2 in that order, so its
    // refinement edge is the image of the parent's:
    //   (v0, m2, m1), (m2, v1, m0), (m1, m0, v2), (m0, m1, m2).
    const std::array<std::array<std::size_t, 3>, 4> red{
        {{v[0], m2, m1}, {m2, v[1], m0}, {m1, m0, v[2]}, {m0, m1, m2}}};
    for (const auto &c : red) {
      auto [s, e] = mesh_.find_edge(c[0], c[1]);
      if (s == no_index ||
          mesh_.triangles()[s][static_cast<std::size_t>(e)] != c[2])
        throw std::logic_error("edge bisection: a red child is missing");
      mesh_.set_refinement_edge(s, local_vertex(mesh_, s, c[0]));
    }
    return Red::made;
  }

  // Sizes the per-triangle and per-node bookkeeping to the mesh. New
  // entries have no parent until the split sets one.
  void grow() {
    triangle_parent_.resize(mesh_.triangles().size(), no_index);
    bisected_.resize(mesh_.triangles().size(), false);
    node_parent_.resize(mesh_.nodes().size(), {no_index, no_index});
  }

  void mark_changed(std::size_t parent) {
    if (parent != no_index && parent < changed_.size())
      changed_[parent] = true;
  }

  WorkingMesh &mesh_;
  const std::string &name_;
  const BisectionRule &rule_;
  const std::size_t max_points_;
  Counts &counts_;
  StageReport &report_;
  const std::size_t input_nodes_, input_triangles_;
  const std::vector<WorkingMesh::TriangleNodes> input_vertices_;
  std::vector<std::size_t> triangle_parent_;
  std::vector<std::array<std::size_t, 2>> node_parent_;
  std::vector<bool> bisected_;
  std::vector<bool> changed_; // per input triangle
  std::vector<NodePair> pending_;
  std::map<NodePair, std::size_t> split_node_; // split edge -> its node
  std::vector<std::size_t> touched_;
  std::string parents_lost_;
};

// The metrics both refine() and run() report.
void record(const Counts &c, const BisectionRule &rule, StageReport &report) {
  auto &m = report.metrics;
  m["splits"] = static_cast<double>(c.splits);
  m["boundary_splits"] = static_cast<double>(c.boundary_splits);
  m["retriangulated_splits"] = static_cast<double>(c.retriangulated);
  m["deleted_nodes"] = static_cast<double>(c.deleted_nodes);
  m["unsplittable_edges"] = static_cast<double>(c.unsplittable);
  m["incompatible_splits"] = static_cast<double>(c.incompatible);
  if (rule.red) {
    m["red_triangles"] = static_cast<double>(c.red);
    m["red_not_possible"] = static_cast<double>(c.red_not_possible);
  }
  if (c.incompatible > 0)
    report.messages.push_back(
        std::to_string(c.incompatible) +
        " edges were split before the refinement edge of a triangle on "
        "their side, because the triangles waited for each other in a "
        "cycle (refinement edges that are not compatible): those triangles "
        "are bisected, but not by the newest vertex pattern. Call "
        "set_longest_edges_as_refinement_edges on a mesh that did not come "
        "from bisection");
  if (c.red_not_possible > 0)
    report.messages.push_back(
        std::to_string(c.red_not_possible) +
        " triangles with all three edges halved keep the bisection pattern: "
        "the red division would invert a child (a new boundary point lies "
        "on a curve, too far from its chord)");
}

struct StepResult {
  std::optional<Parents> parents;
  std::string parents_lost;
  std::size_t refined_by_closure = 0;
};

// One step on sorted, distinct marked triangles.
StepResult step(WorkingMesh &mesh, std::span<const std::size_t> marked,
                const std::string &name, const BisectionRule &rule,
                std::size_t max_points, LocalImproverCalls &local,
                Counts &counts, StageReport &report) {
  Step s(mesh, name, rule, max_points, counts, report);
  s.mark(marked);
  s.split_marked_edges();
  if (rule.red)
    s.make_red();
  auto touched = s.touched();
  if (!touched.empty())
    local.after_change(mesh, touched);
  return {s.parents(), s.parents_lost(), s.refined_by_closure(marked)};
}

} // namespace

void check_marked(const WorkingMesh &mesh, std::span<const std::size_t> marked,
                  const std::string &refiner) {
  for (auto t : marked) {
    if (t >= mesh.triangles().size())
      throw std::invalid_argument(
          refiner + ": marked triangle " + std::to_string(t) +
          " is out of range (the mesh has " +
          std::to_string(mesh.triangles().size()) + " triangles)");
    if (mesh.is_triangle_deleted(t))
      throw std::invalid_argument(refiner + ": marked triangle " +
                                  std::to_string(t) + " is deleted");
  }
}

std::optional<Parents>
EdgeBisection::refine(WorkingMesh &mesh, std::span<const std::size_t> marked,
                      std::span<const LocalImprover *const> local_improvers,
                      StageReport &report) const {
  check_marked(mesh, marked, name_);
  std::vector<std::size_t> sorted(marked.begin(), marked.end());
  std::sort(sorted.begin(), sorted.end());
  sorted.erase(std::unique(sorted.begin(), sorted.end()), sorted.end());
  if (sorted.empty())
    report.messages.push_back(
        "nothing was marked; the mesh is returned unchanged");
  Counts counts;
  LocalImproverCalls local(local_improvers);
  StepResult r =
      step(mesh, sorted, name_, rule_, max_points_, local, counts, report);
  report.metrics["marked"] = static_cast<double>(sorted.size());
  report.metrics["refined_by_closure"] =
      static_cast<double>(r.refined_by_closure);
  record(counts, rule_, report);
  local.record(report);
  if (!r.parents)
    report.messages.push_back("parents dropped: " + r.parents_lost);
  return std::move(r.parents);
}

void EdgeBisection::run(WorkingMesh &mesh, const MeshSpec &spec,
                        std::span<const LocalImprover *const> local_improvers,
                        StageReport &report) const {
  validate(spec);
  const double size = domain_size(mesh);
  const double h = resolve(spec.max_edge_length, size);
  report.metrics["domain_size"] = size;
  report.metrics["max_edge_length"] = h;
  Counts counts;
  LocalImproverCalls local(local_improvers);
  std::size_t steps = 0;
  // Each step refines every triangle with a long edge. The loop ends when
  // none is left, or when a step could split nothing (every long edge
  // waits for a boundary edge that cannot be split).
  while (true) {
    std::vector<std::size_t> marked;
    for (std::size_t t = 0; t < mesh.triangles().size(); ++t)
      if (!mesh.is_triangle_deleted(t) && longest_length(mesh, t) > h)
        marked.push_back(t);
    if (marked.empty())
      break;
    const std::size_t before = counts.splits;
    (void)step(mesh, marked, name_, rule_, max_points_, local, counts, report);
    ++steps;
    if (counts.splits == before)
      break;
  }
  report.metrics["steps"] = static_cast<double>(steps);
  record(counts, rule_, report);
  local.record(report);
}

} // namespace pde2d::mesh::detail
