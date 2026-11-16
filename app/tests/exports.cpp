// Exports (docs/app.md, "Exports"): they never compute a run, they carry
// their provenance (with the build's commit and the case's hash), the VTK
// file holds the mesh and every snapshot, the VTK series is a stored zip
// with a manifest; errors carry their path as a field; the problem check
// gives no facts for a text that did not parse; --set edits a case by
// path, --sweep runs every combination.

#include "case_set.hpp"
#include "domain_input.hpp"
#include "session.hpp"
#include "sha256.hpp"
#include "sweep.hpp"
#include "zip.hpp"

#include "app_support.hpp"

#include "support/check.hpp"

#include <chrono>
#include <cstdint>
#include <format>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

using namespace pde2d;
using app::Json;
using test::check;
using test::expect_throws;

namespace {

Json time_case(int snapshots, double dt) {
  Json case_json = app::Session::default_case();
  case_json["problem"]["equation"] = {
      {"m1", "1"}, {"a_xx", "-1"}, {"a_yy", "-1"}, {"f", "1"}};
  case_json["problem"]["initial"] = {{"u", "0"}};
  case_json["solve"]["mode"] = "time";
  case_json["solve"]["time"] = {
      {"t0", 0.0},
      {"t1", 1.0},
      {"snapshots", snapshots},
      {"method", {{"type", "implicit_euler"}, {"options", Json::object()}}},
      {"step", {{"type", "fixed_step"}, {"options", {{"dt", dt}}}}}};
  return case_json;
}

template <class F> app::StageError stage_error(F &&f) {
  try {
    f();
  } catch (const app::StageError &error) {
    return error;
  }
  throw test::Failure("expected a StageError");
}

// CRC-32 bit by bit (not the app's table), for checking the zip.
std::uint32_t crc_bitwise(std::string_view data) {
  std::uint32_t crc = 0xFFFFFFFFu;
  for (const char c : data) {
    crc ^= static_cast<unsigned char>(c);
    for (int bit = 0; bit < 8; ++bit)
      crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
  }
  return ~crc;
}

std::uint64_t field(std::string_view bytes, std::size_t at, int size) {
  if (at + static_cast<std::size_t>(size) > bytes.size())
    throw test::Failure("a zip field past the end");
  std::uint64_t value = 0;
  for (int i = size - 1; i >= 0; --i)
    value = (value << 8) |
            static_cast<unsigned char>(bytes[at + static_cast<std::size_t>(i)]);
  return value;
}

struct ZipMember {
  std::string name;
  std::string data;
  std::uint64_t offset = 0;
};

// Reads a stored zip by its local headers, checks every CRC and size, and
// checks that the central directory (reached from the end record, through
// the Zip64 record when there is one) lists the same members at the same
// offsets.
std::vector<ZipMember> read_zip(std::string_view zip) {
  std::vector<ZipMember> members;
  std::size_t at = 0;
  while (field(zip, at, 4) == 0x04034b50) {
    check(field(zip, at + 8, 2) == 0, "stored");
    const auto crc = static_cast<std::uint32_t>(field(zip, at + 14, 4));
    const auto compressed = field(zip, at + 18, 4);
    const auto size = field(zip, at + 22, 4);
    check(compressed == size, "stored: the sizes are equal");
    const auto name_length = field(zip, at + 26, 2);
    const auto extra = field(zip, at + 28, 2);
    ZipMember member;
    member.offset = at;
    member.name = std::string(zip.substr(at + 30, name_length));
    member.data = std::string(zip.substr(at + 30 + name_length + extra, size));
    check(member.data.size() == size, member.name + ": its size");
    check(crc_bitwise(member.data) == crc, member.name + ": its CRC");
    at += 30 + name_length + extra + size;
    members.push_back(std::move(member));
  }
  const std::size_t end = zip.size() - 22;
  check(field(zip, end, 4) == 0x06054b50, "the end record is last");
  std::uint64_t entries = field(zip, end + 10, 2);
  std::uint64_t directory = field(zip, end + 16, 4);
  if (directory == 0xFFFFFFFFu) {
    const std::size_t locator = end - 20;
    check(field(zip, locator, 4) == 0x07064b50, "the Zip64 locator");
    const auto record = field(zip, locator + 8, 8);
    check(field(zip, record, 4) == 0x06064b50, "the Zip64 end record");
    entries = field(zip, record + 32, 8);
    directory = field(zip, record + 48, 8);
  }
  check(directory == at, "the central directory follows the members");
  check(entries == members.size(), "as many entries as members");
  std::size_t p = directory;
  for (const ZipMember &member : members) {
    check(field(zip, p, 4) == 0x02014b50, "a central record");
    check(field(zip, p + 16, 4) == crc_bitwise(member.data), "its CRC");
    const auto name_length = field(zip, p + 28, 2);
    const auto extra = field(zip, p + 30, 2);
    const auto comment = field(zip, p + 32, 2);
    check(std::string(zip.substr(p + 46, name_length)) == member.name,
          "its name");
    std::uint64_t offset = field(zip, p + 42, 4);
    if (offset == 0xFFFFFFFFu) {
      check(field(zip, p + 46 + name_length, 2) == 0x0001, "Zip64 extra");
      offset = field(zip, p + 46 + name_length + 4, 8);
    }
    check(offset == member.offset, member.name + ": its offset");
    p += 46 + name_length + extra + comment;
  }
  return members;
}

std::vector<std::string> lines_of(const std::string &text) {
  std::vector<std::string> out;
  std::istringstream in(text);
  for (std::string line; std::getline(in, line);)
    out.push_back(line);
  return out;
}

} // namespace

int main() {
  return test::run({
      {"a solution export never computes",
       [] {
         app::Session session({.background_runs = true});
         const Json case_json = time_case(400, 0.0025);
         // No run of the case: refused, nothing started.
         const app::StageError none = stage_error(
             [&] { (void)session.export_file(case_json, "solution.csv"); });
         check(none.kind() == "not_ready", none.kind());
         check(std::string(none.what()).find("solve it first") !=
                   std::string::npos,
               none.what());
         // A run that is not done (stopped at once): refused too.
         const Json started = session.solve(case_json);
         const auto job = started["job"].get<std::uint64_t>();
         const Json stopped = session.stop_solve(job);
         if (stopped["state"] == "stopped") {
           for (const char *kind : {"solution.csv", "solution.vtk"}) {
             const app::StageError error = stage_error(
                 [&] { (void)session.export_file(case_json, kind); });
             check(error.kind() == "not_ready", error.kind());
             check(std::string(error.what()).find("stopped") !=
                       std::string::npos,
                   error.what());
           }
           check(session.solve_progress(job)["state"] == "stopped",
                 "the export did not continue the run");
         }
         // Another client's run is not this client's.
         app::Session plain;
         const Json small = time_case(2, 0.25);
         (void)plain.solve(small, "a");
         check(stage_error([&] {
                 (void)plain.export_file(small, "solution.csv",
                                         {.client = "b", .case_name = ""});
               }).kind() == "not_ready",
               "per client");
         const auto file = plain.export_file(small, "solution.csv",
                                             {.client = "a", .case_name = ""});
         check(file.text().find("x,y,u(t=0),u(t=0.5),u(t=1)") !=
                   std::string::npos,
               "the client's run");
       }},
      {"exports carry their provenance",
       [] {
         app::Session session;
         const Json case_json = time_case(2, 0.25);
         (void)session.solve(case_json);
         const app::ExportContext context{.client = "local",
                                          .case_name = "study.json"};
         const std::string csv =
             session.export_file(case_json, "solution.csv", context).text();
         const auto csv_lines = lines_of(csv);
         check(csv_lines[0].starts_with("# produced by pde2d_app "), "app");
         check(csv.find("# case: study.json\n") != std::string::npos, "case");
         check(csv.find("# run: ") != std::string::npos, "run");
         check(csv.find("# settings: {\"geometry\"") != std::string::npos,
               "settings");

         const std::string mtx =
             session.export_file(case_json, "stiffness.mtx", context).text();
         const auto mtx_lines = lines_of(mtx);
         check(mtx_lines[0] == "%%MatrixMarket matrix coordinate real general",
               "the banner stays first");
         check(mtx.find("\n% produced by pde2d_app ") != std::string::npos,
               "% comments");
         check(mtx.find("\"assembly\"") != std::string::npos,
               "the assembly settings");

         const std::string svg =
             session.export_file(case_json, "mesh.svg", context).text();
         check(svg.find("<!--\n  produced by pde2d_app") != std::string::npos,
               "an XML comment");
         // .pde2d keeps the old format exactly: no comment.
         const std::string pde2d =
             session.export_file(case_json, "mesh.pde2d", context).text();
         check(pde2d.find("pde2d_app") == std::string::npos, "unchanged");

         const Json provenance = Json::parse(
             session.export_file(case_json, "provenance.json", context).text());
         check(provenance["format"] == "pde2d-app-provenance", "format");
         check(provenance["case"] == case_json, "the full case");
         check(provenance["case_name"] == "study.json", "the name");
         check(provenance["library"]["version"].is_string(), "versions");
         check(provenance["run"]["state"] == "done" &&
                   provenance["run"]["reports"].size() == 1,
               "the run's report");
         // Without a run of the case, provenance has none.
         Json other = case_json;
         other["solve"]["time"]["snapshots"] = 4;
         check(Json::parse(
                   session.export_file(other, "provenance.json").text())["run"]
                   .is_null(),
               "no run");
       }},
      {"the VTK file has the mesh and one array per snapshot",
       [] {
         app::Session session;
         const Json case_json = time_case(2, 0.25);
         const auto values = session.solution_values(case_json);
         const Json mesh = app::test_support::mesh_view(session, case_json);
         const std::size_t nodes = mesh["nodes"].size() / 2;
         const std::size_t triangles = mesh["triangles"].size() / 3;
         const app::ExportedFile file =
             session.export_file(case_json, "solution.vtk");
         check(static_cast<bool>(file.stream), "streamed");
         const auto lines = lines_of(file.text());
         check(lines[0] == "# vtk DataFile Version 3.0", "version");
         check(lines[1].starts_with("pde2d_app ") && lines[1].size() < 256,
               "title");
         check(lines[2] == "ASCII" && lines[3] == "DATASET UNSTRUCTURED_GRID",
               "kind");
         check(lines[4] == "POINTS " + std::to_string(nodes) + " double",
               lines[4]);
         std::size_t at = 5;
         for (std::size_t i = 0; i < nodes; ++i, ++at) {
           std::istringstream in(lines[at]);
           double x = 0, y = 0, z = 1;
           in >> x >> y >> z;
           check(x == mesh["nodes"][2 * i].get<double>() &&
                     y == mesh["nodes"][2 * i + 1].get<double>() && z == 0,
                 "point " + std::to_string(i));
         }
         check(lines[at] == "CELLS " + std::to_string(triangles) + " " +
                                std::to_string(4 * triangles),
               lines[at]);
         at += 1 + triangles;
         check(lines[at] == "CELL_TYPES " + std::to_string(triangles),
               lines[at]);
         check(lines[at + 1] == "5", "triangles");
         at += 1 + triangles;
         check(lines[at] == "POINT_DATA " + std::to_string(nodes), lines[at]);
         ++at;
         for (std::size_t k = 0; k < values.size(); ++k) {
           check(lines[at].starts_with("SCALARS u(t=") &&
                     lines[at].ends_with(" double 1"),
                 lines[at]);
           check(lines[at + 1] == "LOOKUP_TABLE default", "table");
           at += 2;
           for (std::size_t i = 0; i < nodes; ++i, ++at)
             check(std::stod(lines[at]) == values[k][i],
                   "value " + std::to_string(i) + " exactly");
         }
         check(at == lines.size(), "nothing after the last array");
       }},
      {"errors carry the path of their field",
       [] {
         app::Session session;
         Json case_json = app::Session::default_case();
         case_json["mesh"]["refiner"]["options"]["min_angle_degrees"] = 50;
         app::StageError error =
             stage_error([&] { (void)session.mesh(case_json); });
         check(error.path() == "mesh.refiner.options.min_angle_degrees",
               error.path());
         check(std::string(error.what())
                   .starts_with("mesh.refiner.options.min_angle_degrees: "),
               "the message keeps its path in front");
         check(app::error_json(error)["path"] == error.path(), "in JSON");

         // A shape's option, like a component's.
         case_json = app::Session::default_case();
         case_json["geometry"]["options"]["height"] = -1;
         error = stage_error([&] { (void)session.geometry(case_json); });
         check(error.path() == "geometry.options.height", error.path());

         // A region edit.
         case_json = app::Session::default_case();
         case_json["geometry"]["region_edits"] = {
             {{"op", "rename"}, {"from", "nowhere"}, {"to", "x"}}};
         error = stage_error([&] { (void)session.geometry(case_json); });
         check(error.path() == "geometry.region_edits[0]", error.path());

         // The solve settings.
         case_json = app::Session::default_case();
         case_json["solve"]["mode"] = "time";
         case_json["solve"]["time"]["step"]["options"]["dt"] = -1;
         error = stage_error([&] { (void)session.solve(case_json); });
         check(error.path() == "solve.time.step.options.dt", error.path());

         // A coefficient.
         case_json = app::Session::default_case();
         case_json["problem"]["equation"]["c"] = "sin(";
         error = stage_error([&] { (void)session.assemble(case_json); });
         check(error.path() == "problem.equation.c", error.path());

         // A library message that names no field has no path.
         case_json = app::Session::default_case();
         case_json["problem"]["boundary_conditions"].erase("left");
         error = stage_error([&] { (void)session.assemble(case_json); });
         check(error.stage() == "meshed_problem" && error.path().empty(),
               error.stage() + " " + error.path());
       }},
      {"a text that did not parse has no facts",
       [] {
         app::Session session;
         Json case_json = app::Session::default_case();
         case_json["problem"]["equation"]["a_yy"] = "sin(";
         const Json check_result = session.check_problem(case_json);
         check(!check_result["ok"].get<bool>(), "an error");
         check(check_result["errors"][0]["path"] == "problem.equation.a_yy",
               "its path");
         check(!check_result["facts"].contains("problem.equation.a_yy"),
               "no facts for it");
         check(check_result["facts"]["problem.equation.a_xx"]["zero"] == false,
               "the others keep theirs");
       }},
      {"CRC-32 and SHA-256 give the standard check values",
       [] {
         check(app::crc32("123456789") == 0xCBF43926u, "CRC-32");
         check(app::crc32("6789", app::crc32("12345")) == 0xCBF43926u,
               "continued");
         check(app::sha256_hex("") == "e3b0c44298fc1c149afbf4c8996fb924"
                                      "27ae41e4649b934ca495991b7852b855",
               "empty");
         check(app::sha256_hex("abc") == "ba7816bf8f01cfea414140de5dae2223"
                                         "b00361a396177a9cb410ff61f20015ad",
               "abc");
         check(
             app::sha256_hex(
                 "abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq") ==
                 "248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db"
                 "06c1",
             "two blocks");
         app::Sha256 pieces;
         for (int i = 0; i < 1000; ++i)
           pieces.update("a");
         check(pieces.hex() == app::sha256_hex(std::string(1000, 'a')),
               "in pieces");
       }},
      {"a zip's structure: local headers, central directory, CRCs, Zip64",
       [] {
         for (const std::uint64_t zip64_from :
              {std::uint64_t{0xFFFFFFFFu}, std::uint64_t{0}}) {
           std::string zip;
           app::ZipWriter writer([&](std::string_view piece) { zip += piece; },
                                 zip64_from);
           writer.add("a.txt", "hello");
           writer.add("empty", "");
           writer.add("b/c.vtk", std::string(70000, 'x'));
           writer.finish();
           const auto members = read_zip(zip);
           check(members.size() == 3 && members[0].data == "hello" &&
                     members[1].data.empty() && members[2].name == "b/c.vtk",
                 "the members");
           if (zip64_from == 0)
             check(zip.find("PK\x06\x06") != std::string::npos,
                   "Zip64 records");
           expect_throws<std::logic_error>([&] { writer.add("late", "x"); },
                                           "after the central directory");
         }
       }},
      {"the VTK series is a zip of one file per snapshot and a manifest",
       [] {
         app::Session session;
         const Json case_json = time_case(4, 0.05);
         const Json progress = session.solve(case_json);
         const app::ExportedFile file =
             session.export_file(case_json, "solution-vtk.zip",
                                 {.client = "local", .case_name = "s.json"});
         check(file.name == "solution-vtk.zip" &&
                   file.content_type == "application/zip",
               "a zip");
         const auto members = read_zip(file.text());
         check(members.size() == 5 + 2, "five snapshots, manifest, provenance");
         for (std::size_t k = 0; k < 5; ++k) {
           check(members[k].name == std::format("solution_{:04}.vtk", k),
                 members[k].name);
           const std::string &vtk = members[k].data;
           check(vtk.starts_with("# vtk DataFile Version 3.0\n"), "VTK");
           check(vtk.find("SCALARS u double 1") != std::string::npos &&
                     vtk.find("SCALARS u(t") == std::string::npos,
                 "one array named u");
           check(lines_of(vtk)[1].find("case sha256") != std::string::npos,
                 "provenance in the title");
         }
         const Json series = Json::parse(members[5].data);
         check(members[5].name == "solution.vtk.series", "the manifest");
         check(series["file-series-version"] == "1.0", "its version");
         check(series["files"].size() == 5, "every file");
         for (std::size_t k = 0; k < 5; ++k) {
           check(series["files"][k]["name"] == members[k].name, "its name");
           check(series["files"][k]["time"] == progress["times"][k],
                 "its time");
         }
         const Json provenance = Json::parse(members[6].data);
         check(members[6].name == "solution-vtk.provenance.json" &&
                   provenance["file"] == "solution-vtk.zip" &&
                   provenance["case_name"] == "s.json",
               "the provenance");
         // The members one by one (the command line writes them as files)
         // are the zip's.
         std::vector<std::string> names;
         file.members([&](std::string_view name, std::string_view content) {
           names.emplace_back(name);
           check(content == members[names.size() - 1].data, "the same file");
         });
         check(names.size() == members.size(), "the same members");
       }},
      {"provenance names the build and hashes the case",
       [] {
         app::Session session;
         const Json case_json = time_case(2, 0.25);
         (void)session.solve(case_json);
         const Json provenance = Json::parse(
             session.export_file(case_json, "provenance.json").text());
         const Json &build = provenance["app"];
         const std::string commit = build["commit"].get<std::string>();
         check(commit == "unknown" ||
                   (commit.size() == 40 &&
                    commit.find_first_not_of("0123456789abcdef") ==
                        std::string::npos),
               "a commit: " + commit);
         check(build["dirty"].is_boolean() || build["dirty"].is_null(),
               "a dirty flag");
         const std::string hash = provenance["case_sha256"].get<std::string>();
         check(hash.size() == 64 && hash == app::case_hash(case_json),
               "the case's hash");
         // Canonical: the order of keys does not change it, a value does.
         Json reordered = Json::object();
         std::vector<std::string> keys;
         for (const auto &[key, value] : case_json.items())
           keys.push_back(key);
         for (auto it = keys.rbegin(); it != keys.rend(); ++it)
           reordered[*it] = case_json[*it];
         check(reordered.dump() != case_json.dump() &&
                   app::case_hash(reordered) == hash,
               "keys in another order");
         Json changed = case_json;
         changed["solve"]["time"]["t1"] = 2.0;
         check(app::case_hash(changed) != hash, "another value");
         const std::string csv =
             session.export_file(case_json, "solution.csv").text();
         check(csv.find("# case sha256: " + hash + "\n") != std::string::npos,
               "in the CSV");
         check(csv.find("(commit ") != std::string::npos, "the commit too");
         const std::string vtk =
             session.export_file(case_json, "solution.vtk").text();
         const std::string title = lines_of(vtk)[1];
         check(title.find("case sha256 " + hash.substr(0, 16)) !=
                       std::string::npos &&
                   title.size() <= 255,
               "in the VTK title: " + title);
       }},
      {"--sweep runs every combination in one session",
       [] {
         const app::SweepAxis axis = app::parse_sweep_argument(
             "mesh.spec.max_edge_length.relative=0.2,0.1");
         check(axis.path == "mesh.spec.max_edge_length.relative" &&
                   axis.values.size() == 2,
               "parsed");
         expect_throws<std::invalid_argument>(
             [] { (void)app::parse_sweep_argument("a.b="); }, "PATH=v1");
         expect_throws<std::invalid_argument>(
             [] { (void)app::parse_sweep_argument("a.b=1,,2"); },
             "an empty value");
         // A convergence study: u = x^2 + y^2 with f = -4 is not in the P1
         // space, so the error falls with the mesh size.
         Json case_json = app::Session::default_case();
         case_json["problem"]["equation"]["f"] = "-4";
         for (auto &[region, condition] :
              case_json["problem"]["boundary_conditions"].items())
           condition = {{"alpha", "1"}, {"g", "x^2 + y^2"}};
         case_json["view"] = {{"exact_solution", "x^2 + y^2"}};
         const std::vector<app::SweepAxis> axes = {
             axis, app::parse_sweep_argument("solve.linear_solver.type="
                                             "sparse_lu,simplicial_ldlt")};
         app::Session session;
         const Json sweep = app::run_sweep(session, case_json, axes);
         check(sweep["rows"].size() == 4, "two by two");
         const Json &rows = sweep["rows"];
         check(rows[0]["values"]["solve.linear_solver.type"] == "sparse_lu" &&
                   rows[1]["values"]["solve.linear_solver.type"] ==
                       "simplicial_ldlt" &&
                   rows[2]["values"]["mesh.spec.max_edge_length.relative"] ==
                       0.1,
               "the first axis slowest");
         for (const Json &row : rows)
           check(row["ok"] == true && row["max_l2"].is_number(), row.dump());
         check(rows[2]["nodes"].get<int>() > rows[0]["nodes"].get<int>(),
               "a finer mesh");
         check(rows[0]["nodes"] == rows[1]["nodes"], "the same mesh reused");
         check(rows[2]["max_l2"].get<double>() <
                   rows[0]["max_l2"].get<double>(),
               "a smaller error");
         check(rows[2]["rate_l2"].get<double>() > 1.0, "about order 2");
         const std::string table = app::sweep_table(sweep);
         check(table.find("rate_l2") != std::string::npos, "a table");
         const std::string csv = app::sweep_csv(sweep);
         check(lines_of(csv).size() == 5 &&
                   csv.starts_with("mesh.spec.max_edge_length.relative,"
                                   "solve.linear_solver.type,ok,"),
               "a CSV: " + csv);
         expect_throws<app::PathError>(
             [&] {
               const std::vector<app::SweepAxis> bad = {
                   app::parse_sweep_argument("mesh.nope.x=1")};
               (void)app::run_sweep(session, case_json, bad);
             },
             "mesh.nope.x");
       }},
      {"--set replaces a value of the case by path",
       [] {
         Json case_json = app::Session::default_case();
         app::apply_set_argument(case_json,
                                 "mesh.spec.max_edge_length.relative=0.02");
         check(case_json["mesh"]["spec"]["max_edge_length"]["relative"] == 0.02,
               "a number");
         app::apply_set_argument(
             case_json,
             R"(mesh.improvers=[{"type": "delaunay_flips", "options": {}}])");
         app::apply_set_argument(case_json,
                                 "mesh.improvers[0].options.use=final_only");
         check(case_json["mesh"]["improvers"][0]["options"]["use"] ==
                   "final_only",
               "an index; text that is not JSON is a string");
         app::apply_set_argument(case_json, "solve.mode=\"time\"");
         check(case_json["solve"]["mode"] == "time", "a JSON string");
         app::apply_set_argument(case_json, "mesh.refiner=null");
         check(case_json["mesh"]["refiner"].is_null(), "null");
         app::apply_set_argument(case_json, "problem.parameters.k=2.5");
         check(case_json["problem"]["parameters"]["k"] == 2.5,
               "a new last key");
         expect_throws<app::PathError>(
             [&] { app::apply_set_argument(case_json, "mesh.nope.x=1"); },
             "mesh.nope.x: 'nope' does not exist in mesh");
         expect_throws<app::PathError>(
             [&] {
               app::apply_set_argument(case_json, "mesh.improvers[7].type=x");
             },
             "mesh.improvers has 1 items, so there is no [7]");
         expect_throws<app::PathError>(
             [&] { app::apply_set_argument(case_json, "solve.mode.x=1"); },
             "is not an object");
         expect_throws<std::invalid_argument>(
             [&] { app::apply_set_argument(case_json, "no_equals"); },
             "PATH=VALUE");
         // The edited case runs.
         Json run_case = app::Session::default_case();
         app::apply_set_argument(run_case, "geometry.options.width=3");
         app::Session session;
         check(session.run(run_case)["ok"].get<bool>(), "runs");
       }},
  });
}
