#include "stage_cache.hpp"

#include "app_error.hpp"
#include "catalog.hpp"
#include "problem_input.hpp"

#include "pde2d/fem/assembler.hpp"

#include <cmath>
#include <format>
#include <utility>

namespace pde2d::app {

namespace {

std::string key_of(const Json &case_json, std::string_view name) {
  return member(case_json, name).dump();
}

} // namespace

AssemblySettings assembly_settings(const Json &case_json) {
  AssemblySettings out;
  const Json &assembly = member(case_json, "assembly");
  if (assembly.is_null())
    return out;
  require_object(assembly, "assembly");
  static constexpr std::string_view known[] = {"t", "max_time_order"};
  reject_unknown_keys(assembly, "assembly", known);
  if (const Json &t = member(assembly, "t"); !t.is_null())
    out.t = require_number(t, "assembly.t");
  if (const Json &order = member(assembly, "max_time_order");
      !order.is_null()) {
    const double value = require_number(order, "assembly.max_time_order");
    // The library checks the value; the app only refuses what an int
    // cannot hold.
    if (value != std::floor(value) || std::abs(value) > 1e9)
      throw PathError("assembly.max_time_order", "expected a whole number");
    out.max_time_order = static_cast<int>(value);
  }
  return out;
}

std::string solve_key(const Json &case_json) {
  return key_of(case_json, "geometry") + "\n" + key_of(case_json, "mesh") +
         "\n" + key_of(case_json, "problem") + "\n" +
         key_of(case_json, "solve");
}

const Domain &StageCache::domain(const Json &case_json) {
  std::string key = key_of(case_json, "geometry");
  if (!domain_ || key != domain_key_) {
    domain_.reset();
    domain_ = staged("geometry", [&] {
      return std::make_shared<const Domain>(
          build_domain(member(case_json, "geometry")));
    });
    domain_key_ = std::move(key);
  }
  return *domain_;
}

std::shared_ptr<const MeshOutcome> StageCache::mesh(const Json &case_json) {
  const Domain &input = domain(case_json);
  std::string key = domain_key_ + "\n" + key_of(case_json, "mesh");
  if (!mesh_ || key != mesh_key_) {
    mesh_.reset();
    const MeshSetup setup = staged(
        "mesh", [&] { return build_mesh_setup(member(case_json, "mesh")); });
    mesh::MeshResult result = staged("mesh", [&] {
      return input.geometry ? setup.mesher().run(*input.geometry, setup.spec)
                            : setup.remesher().run(*input.mesh, setup.spec);
    });
    auto outcome = std::make_shared<MeshOutcome>();
    outcome->id = next_mesh_++;
    outcome->mesh = std::make_shared<const mesh::Mesh>(std::move(result.mesh));
    outcome->reports = std::move(result.reports);
    outcome->target_met = result.target_met;
    outcome->longest_edge = result.longest_edge;
    outcome->max_edge_length = result.max_edge_length;
    outcome->domain_size = result.domain_size;
    mesh_ = std::move(outcome);
    mesh_key_ = std::move(key);
    recent_.push_back(mesh_);
    if (recent_.size() > recent_meshes)
      recent_.pop_front();
  }
  return mesh_;
}

std::shared_ptr<const fem::MeshedProblem>
StageCache::meshed(const Json &case_json) {
  const std::shared_ptr<const MeshOutcome> outcome = mesh(case_json);
  std::string key = mesh_key_ + "\n" + key_of(case_json, "problem");
  if (!meshed_ || key != meshed_key_) {
    assembled_.reset();
    meshed_.reset();
    const fem::Problem problem = staged(
        "problem", [&] { return build_problem(member(case_json, "problem")); });
    meshed_ = staged("meshed_problem", [&] {
      return std::make_shared<const fem::MeshedProblem>(outcome->mesh, problem);
    });
    meshed_key_ = std::move(key);
  }
  return meshed_;
}

const fem::AssembledProblem &
StageCache::assembled(const Json &case_json, const AssemblySettings &settings) {
  const double t = settings.t;
  const auto problem = meshed(case_json);
  std::string key =
      meshed_key_ + "\n" + std::format("{} {}", t, settings.max_time_order);
  if (!assembled_ || key != assembled_key_) {
    // A previous result of the same MeshedProblem lets assembly reuse its
    // constant parts; the result is the same bit for bit.
    const fem::AssembledProblem *previous =
        assembled_ && assembled_->meshed_problem() == problem ? assembled_.get()
                                                              : nullptr;
    auto next = staged("assembly", [&] {
      return std::make_shared<const fem::AssembledProblem>(
          fem::Assembler().assemble(problem, t, previous,
                                    settings.max_time_order));
    });
    assembled_ = std::move(next);
    assembled_key_ = std::move(key);
  }
  return *assembled_;
}

std::shared_ptr<const MeshOutcome>
StageCache::find_mesh(std::uint64_t id) const {
  for (const auto &outcome : recent_)
    if (outcome->id == id)
      return outcome;
  return nullptr;
}

std::uint64_t StageCache::id_of(const mesh::Mesh *mesh) const {
  for (const auto &outcome : recent_)
    if (outcome->mesh.get() == mesh)
      return outcome->id;
  return 0;
}

} // namespace pde2d::app
