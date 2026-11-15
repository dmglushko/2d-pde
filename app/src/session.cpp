#include "session.hpp"

#include "catalog.hpp"
#include "domain_input.hpp"
#include "problem_input.hpp"
#include "views.hpp"

#include "pde2d/fem/assembler.hpp"
#include "pde2d/fem/compiled_expression.hpp"
#include "pde2d/fem/meshed_problem.hpp"
#include "pde2d/io/pde2d_mesh.hpp"
#include "pde2d/io/svg.hpp"

#include <cmath>
#include <format>
#include <optional>
#include <sstream>
#include <utility>
#include <vector>

#ifndef PDE2D_VERSION
#define PDE2D_VERSION "unknown"
#endif

namespace pde2d::app {

namespace {

constexpr std::string_view case_format = "pde2d-app-case";
constexpr int case_version = 1;

// Runs one stage and turns its exception into a StageError.
template <class Step> auto staged(std::string_view stage, Step step) {
  try {
    return step();
  } catch (const StageError &) {
    throw;
  } catch (const std::invalid_argument &error) {
    throw StageError(std::string(stage), "input", error.what());
  } catch (const std::runtime_error &error) {
    throw StageError(std::string(stage), "algorithm", error.what());
  } catch (const std::exception &error) {
    throw StageError(std::string(stage), "internal", error.what());
  }
}

const Json &section(const Json &case_json, std::string_view key) {
  return member(case_json, key);
}

std::string key_of(const Json &case_json, std::string_view name) {
  return section(case_json, name).dump();
}

// The time the case assembles at ("assembly": {"t": ...}), 0 by default.
double assembly_time(const Json &case_json) {
  const Json &assembly = section(case_json, "assembly");
  if (assembly.is_null())
    return 0.0;
  require_object(assembly, "assembly");
  static constexpr std::string_view known[] = {"t"};
  reject_unknown_keys(assembly, "assembly", known);
  const Json &t = member(assembly, "t");
  return t.is_null() ? 0.0 : require_number(t, "assembly.t");
}

Json values_json(const std::vector<double> &values) {
  Json out = Json::array();
  for (const double value : values)
    out.push_back(std::isfinite(value) ? Json(value) : Json(nullptr));
  return out;
}

Json operator_json(const fem::Operator &op) {
  const auto nonzeros = [](const fem::CsrMatrix &matrix) -> std::size_t {
    return matrix.pattern ? matrix.pattern->nonzeros() : 0;
  };
  Json out = Json::object();
  out["is_zero"] = op.always_zero;
  out["depends_on_t"] = op.depends_on_t;
  out["free_nonzeros"] = nonzeros(op.free);
  out["dirichlet_nonzeros"] = nonzeros(op.dirichlet);
  return out;
}

} // namespace

// --- state ------------------------------------------------------------------

struct MeshOutcome {
  std::shared_ptr<const mesh::Mesh> mesh;
  std::vector<mesh::StageReport> reports;
  bool target_met = false;
  double longest_edge = 0.0;
  double max_edge_length = 0.0;
  double domain_size = 0.0;
};

// The last result of each stage and the key (the case sections) it was
// built from.
struct Session::State {
  std::string domain_key;
  std::shared_ptr<const Domain> domain;
  std::string mesh_key;
  std::shared_ptr<const MeshOutcome> mesh;
  std::string meshed_key;
  std::shared_ptr<const fem::MeshedProblem> meshed;
  std::string assembled_key;
  std::shared_ptr<const fem::AssembledProblem> assembled;

  const Domain &get_domain(const Json &case_json) {
    std::string key = key_of(case_json, "geometry");
    if (!domain || key != domain_key) {
      domain.reset();
      domain = staged("geometry", [&] {
        return std::make_shared<const Domain>(
            build_domain(section(case_json, "geometry")));
      });
      domain_key = std::move(key);
    }
    return *domain;
  }

  const MeshOutcome &get_mesh(const Json &case_json) {
    const Domain &input = get_domain(case_json);
    std::string key = domain_key + "\n" + key_of(case_json, "mesh");
    if (!mesh || key != mesh_key) {
      mesh.reset();
      const MeshSetup setup = staged(
          "mesh", [&] { return build_mesh_setup(section(case_json, "mesh")); });
      mesh::MeshResult result = staged("mesh", [&] {
        return input.geometry ? setup.mesher().run(*input.geometry, setup.spec)
                              : setup.remesher().run(*input.mesh, setup.spec);
      });
      auto outcome = std::make_shared<MeshOutcome>();
      outcome->mesh =
          std::make_shared<const mesh::Mesh>(std::move(result.mesh));
      outcome->reports = std::move(result.reports);
      outcome->target_met = result.target_met;
      outcome->longest_edge = result.longest_edge;
      outcome->max_edge_length = result.max_edge_length;
      outcome->domain_size = result.domain_size;
      mesh = std::move(outcome);
      mesh_key = std::move(key);
    }
    return *mesh;
  }

  const std::shared_ptr<const fem::MeshedProblem> &
  get_meshed(const Json &case_json) {
    const MeshOutcome &outcome = get_mesh(case_json);
    std::string key = mesh_key + "\n" + key_of(case_json, "problem");
    if (!meshed || key != meshed_key) {
      assembled.reset();
      meshed.reset();
      const fem::Problem problem = staged("problem", [&] {
        return build_problem(section(case_json, "problem"));
      });
      meshed = staged("meshed_problem", [&] {
        return std::make_shared<const fem::MeshedProblem>(outcome.mesh,
                                                          problem);
      });
      meshed_key = std::move(key);
    }
    return meshed;
  }

  // At `time`, or at the case's assembly time when it is not given.
  const fem::AssembledProblem &
  get_assembled(const Json &case_json,
                std::optional<double> time = std::nullopt) {
    const double t =
        time ? *time
             : staged("assembly", [&] { return assembly_time(case_json); });
    const auto &problem = get_meshed(case_json);
    std::string key = meshed_key + "\n" + std::format("{}", t);
    if (!assembled || key != assembled_key) {
      // A previous result of the same MeshedProblem lets assembly reuse
      // its constant parts; the result is the same bit for bit.
      const fem::AssembledProblem *previous =
          assembled && assembled->meshed_problem() == problem ? assembled.get()
                                                              : nullptr;
      auto next = staged("assembly", [&] {
        return std::make_shared<const fem::AssembledProblem>(
            fem::Assembler().assemble(problem, t, previous));
      });
      assembled = std::move(next);
      assembled_key = std::move(key);
    }
    return *assembled;
  }
};

Session::Session() : state_(std::make_unique<State>()) {}
Session::~Session() = default;

// --- the catalog and the default case ---------------------------------------

Json Session::catalog() {
  Json solve = Json::object();
  solve["id"] = "solve";
  solve["label"] = "Solve";
  solve["available"] = false;
  solve["message"] =
      "The library has no solvers yet. They are planned as pde2d_solve "
      "(docs/decisions.md, \"Solvers and time integration\"); this step "
      "fills in when they land.";

  Json sections = Json::array();
  sections.push_back(mesh_catalog_json());
  sections.push_back(std::move(solve));

  Json out = Json::object();
  out["format"] = case_format;
  out["version"] = case_version;
  out["library_version"] = PDE2D_VERSION;
  out["shapes"] = shape_catalog_json();
  out["sections"] = std::move(sections);
  out["problem"] = problem_catalog_json();
  out["default_case"] = default_case();
  return out;
}

Json Session::default_case() {
  Json geometry = default_geometry_section();
  Json problem = default_problem_section();
  // One condition per region of the default shape, so the default case
  // runs to the end.
  const Domain domain = build_domain(geometry);
  for (const std::string &region : domain.geometry->region_names())
    problem["boundary_conditions"][region] = default_condition();

  Json assembly = Json::object();
  assembly["t"] = 0.0;
  Json out = Json::object();
  out["format"] = case_format;
  out["version"] = case_version;
  out["geometry"] = std::move(geometry);
  out["mesh"] = default_mesh_section();
  out["problem"] = std::move(problem);
  out["assembly"] = std::move(assembly);
  out["solve"] = nullptr;
  return out;
}

void check_case(const Json &case_json) {
  require_object(case_json, "case");
  static constexpr std::string_view known[] = {"format", "version", "geometry",
                                               "mesh",   "problem", "assembly",
                                               "solve",  "view",    "notes"};
  reject_unknown_keys(case_json, "case", known);
  if (const Json &format = member(case_json, "format"); !format.is_null()) {
    if (require_string(format, "format") != case_format)
      throw std::invalid_argument("format: this is not a pde2d app case");
  }
  if (const Json &version = member(case_json, "version"); !version.is_null()) {
    if (require_number(version, "version") > case_version)
      throw std::invalid_argument(
          "version: the case was written by a newer version of the app");
  }
}

// --- stages -----------------------------------------------------------------

Json Session::geometry(const Json &case_json) {
  const std::scoped_lock lock(mutex_);
  staged("case", [&] { check_case(case_json); });
  return staged("geometry",
                [&] { return domain_json(state_->get_domain(case_json)); });
}

Json Session::mesh(const Json &case_json) {
  const std::scoped_lock lock(mutex_);
  staged("case", [&] { check_case(case_json); });
  const MeshOutcome &outcome = state_->get_mesh(case_json);
  Json out = Json::object();
  out["mesh"] = mesh_json(*outcome.mesh);
  out["stats"] = mesh_stats_json(*outcome.mesh);
  out["target_met"] = outcome.target_met;
  out["longest_edge"] = outcome.longest_edge;
  out["max_edge_length"] = outcome.max_edge_length;
  out["domain_size"] = outcome.domain_size;
  out["reports"] = reports_json(outcome.reports);
  return out;
}

Json Session::check_problem(const Json &case_json) {
  const std::scoped_lock lock(mutex_);
  staged("case", [&] { check_case(case_json); });
  return staged("problem", [&] {
    return app::check_problem(section(case_json, "problem"));
  });
}

Json Session::assemble(const Json &case_json) {
  const std::scoped_lock lock(mutex_);
  staged("case", [&] { check_case(case_json); });
  const fem::AssembledProblem &assembled = state_->get_assembled(case_json);
  const fem::MeshedProblem &meshed = *assembled.meshed_problem();

  Json dirichlet = Json::array();
  for (const fem::DirichletNode &node : meshed.dirichlet_nodes())
    dirichlet.push_back(node.node);
  Json meshed_json = Json::object();
  meshed_json["report"] = report_json(meshed.report());
  meshed_json["dirichlet_nodes"] = std::move(dirichlet);
  meshed_json["free_nodes"] = meshed.free_nodes().size();
  meshed_json["time_order"] = fem::time_order(meshed);
  meshed_json["coefficients_depend_on_t"] =
      fem::coefficients_depend_on_t(meshed);
  meshed_json["data_depends_on_t"] = fem::data_depends_on_t(meshed);

  Json operators = Json::object();
  operators["mass"] = operator_json(assembled.mass());
  operators["damping"] = operator_json(assembled.damping());
  operators["stiffness"] = operator_json(assembled.stiffness());
  double norm = 0.0;
  for (const double value : assembled.right_hand_side())
    norm += value * value;

  Json assembly = Json::object();
  assembly["report"] = report_json(assembled.report());
  assembly["t"] = assembled.time();
  assembly["time_order"] = assembled.time_order();
  assembly["operators"] = std::move(operators);
  assembly["load_depends_on_t"] = assembled.load_depends_on_t();
  assembly["right_hand_side_depends_on_t"] =
      assembled.right_hand_side_depends_on_t();
  assembly["right_hand_side_norm"] = std::sqrt(norm);
  if (const auto &pattern = assembled.stiffness().free.pattern)
    assembly["sparsity"] = sparsity_json(*pattern, 160);

  Json out = Json::object();
  out["meshed_problem"] = std::move(meshed_json);
  out["assembly"] = std::move(assembly);
  return out;
}

Json Session::field(const Json &request) {
  const std::scoped_lock lock(mutex_);
  const Json &case_json = member(request, "case");
  staged("case", [&] { check_case(case_json); });
  const Json &spec = staged("field", [&]() -> const Json & {
    return require_object(member(request, "field"), "field");
  });
  const std::string kind = staged("field", [&] {
    return require_string(member(spec, "kind"), "field.kind");
  });
  const double t = staged("field", [&] {
    const Json &value = member(spec, "t");
    return value.is_null() ? assembly_time(case_json)
                           : require_number(value, "field.t");
  });

  Json out = Json::object();
  out["kind"] = kind;
  out["t"] = t;
  if (kind == "expression") {
    const MeshOutcome &outcome = state_->get_mesh(case_json);
    const auto text = staged("field", [&] {
      return require_string(member(spec, "text"), "field.text");
    });
    const fem::CompiledExpression compiled = staged("field", [&] {
      const fem::Parameters parameters =
          read_parameters(member(section(case_json, "problem"), "parameters"));
      return fem::CompiledExpression(fem::Expression(text), parameters);
    });
    std::vector<double> values;
    values.reserve(outcome.mesh->nodes.size());
    for (const geometry::Point &p : outcome.mesh->nodes)
      values.push_back(compiled(p.x, p.y, t));
    out["label"] = text;
    out["values"] = values_json(values);
    return out;
  }
  if (kind == "initial_u" || kind == "initial_u_t") {
    const auto &meshed = state_->get_meshed(case_json);
    const auto values = staged("field", [&] {
      return kind == "initial_u" ? meshed->initial_values(t)
                                 : meshed->initial_velocities(t);
    });
    if (!values)
      throw StageError("field", "input",
                       kind == "initial_u"
                           ? "the problem has time order 0, so it has no "
                             "initial values"
                           : "the problem has time order below 2, so it has "
                             "no initial velocities");
    out["label"] = kind == "initial_u" ? "u at the start" : "u_t at the start";
    out["values"] = values_json(*values);
    return out;
  }
  if (kind == "dirichlet") {
    const auto &meshed = state_->get_meshed(case_json);
    std::vector<double> values(meshed->mesh()->nodes.size(), std::nan(""));
    staged("field", [&] {
      for (std::size_t k = 0; k < meshed->dirichlet_nodes().size(); ++k)
        values[meshed->dirichlet_nodes()[k].node] =
            meshed->dirichlet_value(k, t);
    });
    out["label"] = "Dirichlet values";
    out["values"] = values_json(values);
    return out;
  }
  if (kind == "right_hand_side") {
    const fem::AssembledProblem &assembled =
        state_->get_assembled(case_json, t);
    const fem::MeshedProblem &meshed = *assembled.meshed_problem();
    std::vector<double> values(meshed.mesh()->nodes.size(), std::nan(""));
    const auto rhs = assembled.right_hand_side();
    for (std::size_t i = 0; i < rhs.size(); ++i)
      values[meshed.free_nodes()[i]] = rhs[i];
    out["t"] = assembled.time();
    out["label"] = "assembled right-hand side b";
    out["values"] = values_json(values);
    return out;
  }
  throw StageError("field", "input",
                   "field.kind: unknown field '" + kind +
                       "' (expression, initial_u, initial_u_t, dirichlet or "
                       "right_hand_side)");
}

ExportedFile Session::export_file(const Json &case_json,
                                  std::string_view kind) {
  const std::scoped_lock lock(mutex_);
  staged("case", [&] { check_case(case_json); });
  std::ostringstream out;
  if (kind == "geometry.svg") {
    const Domain &domain = state_->get_domain(case_json);
    if (!domain.geometry)
      throw StageError("export", "input",
                       "an imported mesh has no curve geometry to export");
    staged("export", [&] { io::export_svg(*domain.geometry, out); });
    return {.name = "geometry.svg",
            .content_type = "image/svg+xml",
            .content = out.str()};
  }
  const MeshOutcome &outcome = state_->get_mesh(case_json);
  if (kind == "mesh.pde2d") {
    staged("export", [&] { io::export_pde2d(*outcome.mesh, out); });
    return {.name = "mesh.pde2d",
            .content_type = "text/plain",
            .content = out.str()};
  }
  if (kind == "mesh.svg") {
    staged("export", [&] { io::export_mesh_svg(*outcome.mesh, out); });
    return {.name = "mesh.svg",
            .content_type = "image/svg+xml",
            .content = out.str()};
  }
  throw StageError("export", "input",
                   "unknown export '" + std::string(kind) +
                       "' (mesh.pde2d, mesh.svg or geometry.svg)");
}

Json Session::run(const Json &case_json) {
  Json out = Json::object();
  out["ok"] = false;
  try {
    {
      Json domain = geometry(case_json);
      Json summary = Json::object();
      summary["kind"] = domain["kind"];
      summary["regions"] = domain["regions"];
      summary["domain_size"] = domain["domain_size"];
      out["geometry"] = std::move(summary);
    }
    {
      Json result = mesh(case_json);
      result.erase("mesh");
      out["mesh"] = std::move(result);
    }
    {
      Json check = check_problem(case_json);
      const bool ok = check["ok"].get<bool>();
      out["problem"] = std::move(check);
      if (!ok)
        throw StageError("problem", "input",
                         "the problem has errors (see problem.errors)");
    }
    {
      Json result = assemble(case_json);
      result["meshed_problem"].erase("dirichlet_nodes");
      result["assembly"].erase("sparsity");
      out["assembly"] = std::move(result);
    }
    out["ok"] = true;
  } catch (const StageError &error) {
    Json item = Json::object();
    item["stage"] = error.stage();
    item["kind"] = error.kind();
    item["message"] = error.what();
    out["error"] = std::move(item);
  }
  return out;
}

} // namespace pde2d::app
