#include "sweep.hpp"

#include "case_set.hpp"
#include "session.hpp"

#include <cmath>
#include <format>
#include <stdexcept>

namespace pde2d::app {

namespace {

// The seconds of the solve stage: its reports' times.
double solve_seconds(const Json &summary) {
  double total = 0.0;
  for (const Json &report : member(member(summary, "solve"), "reports"))
    if (member(report, "time_ms").is_number())
      total += report["time_ms"].get<double>() / 1000.0;
  return total;
}

Json row_of(const Json &values, const Json &summary) {
  Json row = Json::object();
  row["values"] = values;
  row["ok"] = summary["ok"];
  const Json &mesh = member(summary, "mesh");
  row["nodes"] = member(member(mesh, "stats"), "nodes");
  row["longest_edge"] = member(mesh, "longest_edge");
  row["solve_seconds"] = summary.contains("solve")
                             ? Json(solve_seconds(summary))
                             : Json(nullptr);
  const Json &comparison = member(summary, "comparison");
  for (const char *key : {"max_abs", "max_rms", "max_l2"})
    row[key] = member(comparison, key);
  if (summary.contains("error"))
    row["error"] = summary["error"]["message"];
  return row;
}

// log(e0 / e1) / log(h0 / h1), or null.
Json rate(const Json &previous, const Json &row, const char *error) {
  const Json &e0 = member(previous, error), &e1 = member(row, error);
  const Json &h0 = member(previous, "longest_edge"),
             &h1 = member(row, "longest_edge");
  if (!e0.is_number() || !e1.is_number() || !h0.is_number() ||
      !h1.is_number())
    return nullptr;
  const double ratio_h = h0.get<double>() / h1.get<double>();
  const double ratio_e = e0.get<double>() / e1.get<double>();
  if (!(ratio_h > 0) || std::abs(std::log(ratio_h)) < 1e-9 || !(ratio_e > 0))
    return nullptr;
  return std::log(ratio_e) / std::log(ratio_h);
}

std::string cell(const Json &value) {
  if (value.is_null())
    return "-";
  if (value.is_number_float())
    return std::format("{:.4g}", value.get<double>());
  if (value.is_string())
    return value.get<std::string>();
  return value.dump();
}

// The columns of a table: the swept paths, then the figures.
std::vector<std::string> columns(const Json &sweep) {
  std::vector<std::string> out;
  for (const Json &axis : sweep["axes"])
    out.push_back(axis.get<std::string>());
  for (const char *name : {"ok", "nodes", "longest_edge", "solve_seconds",
                           "max_abs", "max_l2", "rate_max", "rate_l2"})
    out.emplace_back(name);
  return out;
}

Json value_of(const Json &row, const std::string &column) {
  if (row["values"].contains(column))
    return row["values"][column];
  return member(row, column);
}

} // namespace

SweepAxis parse_sweep_argument(std::string_view argument) {
  const std::size_t equals = argument.find('=');
  if (equals == std::string_view::npos || equals == 0 ||
      equals + 1 == argument.size())
    throw std::invalid_argument("--sweep needs PATH=v1,v2,..., got '" +
                                std::string(argument) + "'");
  SweepAxis axis{.path = std::string(argument.substr(0, equals)), .values = {}};
  std::string_view rest = argument.substr(equals + 1);
  while (true) {
    const std::size_t comma = rest.find(',');
    axis.values.emplace_back(rest.substr(0, comma));
    if (axis.values.back().empty())
      throw std::invalid_argument("--sweep " + axis.path +
                                  ": an empty value in '" +
                                  std::string(argument) + "'");
    if (comma == std::string_view::npos)
      break;
    rest = rest.substr(comma + 1);
  }
  return axis;
}

Json run_sweep(Session &session, const Json &case_json,
               std::span<const SweepAxis> axes) {
  Json out = Json::object();
  out["axes"] = Json::array();
  for (const SweepAxis &axis : axes)
    out["axes"].push_back(axis.path);
  out["rows"] = Json::array();
  std::vector<std::size_t> index(axes.size(), 0);
  const auto count = [&] {
    std::size_t total = 1;
    for (const SweepAxis &axis : axes)
      total *= axis.values.size();
    return total;
  }();
  for (std::size_t n = 0; n < count; ++n) {
    // The combination n, the last axis fastest.
    std::size_t rest = n;
    for (std::size_t a = axes.size(); a-- > 0;) {
      index[a] = rest % axes[a].values.size();
      rest /= axes[a].values.size();
    }
    Json variant = case_json;
    Json values = Json::object();
    for (std::size_t a = 0; a < axes.size(); ++a) {
      const std::string &text = axes[a].values[index[a]];
      set_case_value(variant, axes[a].path, text);
      values[axes[a].path] = set_value(text);
    }
    Json row = row_of(values, session.run(variant));
    Json &rows = out["rows"];
    if (!rows.empty()) {
      row["rate_max"] = rate(rows.back(), row, "max_abs");
      row["rate_l2"] = rate(rows.back(), row, "max_l2");
    } else {
      row["rate_max"] = nullptr;
      row["rate_l2"] = nullptr;
    }
    rows.push_back(std::move(row));
  }
  return out;
}

std::string sweep_table(const Json &sweep) {
  const std::vector<std::string> names = columns(sweep);
  std::vector<std::vector<std::string>> cells;
  cells.emplace_back(names);
  for (const Json &row : sweep["rows"]) {
    std::vector<std::string> line;
    for (const std::string &name : names)
      line.push_back(cell(value_of(row, name)));
    cells.push_back(std::move(line));
  }
  std::vector<std::size_t> widths(names.size(), 0);
  for (const auto &line : cells)
    for (std::size_t c = 0; c < line.size(); ++c)
      widths[c] = std::max(widths[c], line[c].size());
  std::string out;
  for (const auto &line : cells) {
    for (std::size_t c = 0; c < line.size(); ++c)
      out += std::format("{}{:>{}}", c == 0 ? "" : "  ", line[c], widths[c]);
    out += '\n';
  }
  for (const Json &row : sweep["rows"])
    if (row.contains("error"))
      out += std::format("failed ({}): {}\n", row["values"].dump(),
                         row["error"].get<std::string>());
  return out;
}

std::string sweep_csv(const Json &sweep) {
  const std::vector<std::string> names = columns(sweep);
  std::string out;
  for (std::size_t c = 0; c < names.size(); ++c)
    out += (c == 0 ? "" : ",") + names[c];
  out += '\n';
  for (const Json &row : sweep["rows"]) {
    for (std::size_t c = 0; c < names.size(); ++c) {
      const Json value = value_of(row, names[c]);
      out += c == 0 ? "" : ",";
      if (value.is_string()) {
        std::string text = value.get<std::string>();
        for (std::size_t at = text.find('"'); at != std::string::npos;
             at = text.find('"', at + 2))
          text.insert(at, 1, '"');
        out += '"' + text + '"';
      } else if (!value.is_null())
        out += value.dump();
    }
    out += '\n';
  }
  return out;
}

} // namespace pde2d::app
