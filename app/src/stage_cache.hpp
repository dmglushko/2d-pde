#pragma once

#include "domain_input.hpp"
#include "options.hpp"

#include "pde2d/fem/assembled_problem.hpp"
#include "pde2d/fem/meshed_problem.hpp"
#include "pde2d/mesh/mesh.hpp"
#include "pde2d/mesh/report.hpp"

#include <cstdint>
#include <deque>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

// The last result of each stage, keyed by the case sections it was built
// from, so changing the problem does not remesh (docs/app.md, "The stage
// cache"). Not thread-safe: the Session calls it under its mutex.
namespace pde2d::app {

// A mesh the Mesher or Remesher made, with the session's id for it.
struct MeshOutcome {
  std::uint64_t id = 0;
  std::shared_ptr<const mesh::Mesh> mesh;
  std::vector<mesh::StageReport> reports;
  bool target_met = false;
  double longest_edge = 0.0;
  double max_edge_length = 0.0;
  double domain_size = 0.0;
};

// The "assembly" section: the time to assemble at (0 by default) and
// which time terms to build (Assembler::assemble's max_time_order, 2 by
// default). Throws PathError.
inline constexpr std::string_view assembly_section_keys[] = {"t",
                                                             "max_time_order"};
struct AssemblySettings {
  double t = 0.0;
  int max_time_order = 2;
};
AssemblySettings assembly_settings(const Json &case_json);

// The key of a run's inputs: the geometry, mesh, problem and solve
// sections. Computed from the text alone, so a caller can find a run of
// the case without building anything.
std::string solve_key(const Json &case_json);

class StageCache {
public:
  // Each builds what is missing (or changed) and throws StageError.
  const Domain &domain(const Json &case_json);
  std::shared_ptr<const MeshOutcome> mesh(const Json &case_json);
  std::shared_ptr<const fem::MeshedProblem> meshed(const Json &case_json);
  // With `settings` (the case's, or another t for a field).
  const fem::AssembledProblem &assembled(const Json &case_json,
                                         const AssemblySettings &settings);
  // A mesh this cache built recently (the last `recent_meshes`), or null.
  std::shared_ptr<const MeshOutcome> find_mesh(std::uint64_t id) const;
  // The id of a mesh this cache built recently, or 0.
  std::uint64_t id_of(const mesh::Mesh *mesh) const;
  // A new mesh id, for a mesh made elsewhere (an adaptive run's last
  // mesh): never one this cache gives.
  std::uint64_t new_mesh_id() { return next_mesh_++; }
  // `count` new mesh ids in a row; the first.
  std::uint64_t new_mesh_ids(std::uint64_t count) {
    const std::uint64_t first = next_mesh_;
    next_mesh_ += count;
    return first;
  }

  static constexpr std::size_t recent_meshes = 16;

private:
  std::string domain_key_;
  std::shared_ptr<const Domain> domain_;
  std::string mesh_key_;
  std::shared_ptr<const MeshOutcome> mesh_;
  std::string meshed_key_;
  std::shared_ptr<const fem::MeshedProblem> meshed_;
  std::string assembled_key_;
  std::shared_ptr<const fem::AssembledProblem> assembled_;
  std::uint64_t next_mesh_ = 1;
  std::deque<std::shared_ptr<const MeshOutcome>> recent_;
};

} // namespace pde2d::app
