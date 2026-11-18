// pde2d_app: the graphical interface of the pde2d library (a local web
// server), and a command line that runs a saved case without it.
// See app/README.md.

#include "case_set.hpp"
#include "server.hpp"
#include "session.hpp"
#include "sweep.hpp"

#include <csignal>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#ifndef PDE2D_APP_WEB_DIR
#define PDE2D_APP_WEB_DIR "web"
#endif
#ifndef PDE2D_APP_EXAMPLES_DIR
#define PDE2D_APP_EXAMPLES_DIR ""
#endif

namespace {

using pde2d::app::Json;

constexpr std::string_view usage = R"(Usage:
  pde2d_app [--host HOST] [--port PORT] [--web DIR] [--token TOKEN]
            [--memory-budget BYTES]
      Starts the interface; open the printed address in a browser.
      Default: 127.0.0.1:8765 and the page files of this checkout. A host
      that is not loopback needs a token on every request: TOKEN, or a
      random one; the printed address carries it. --memory-budget bounds
      what the kept runs of all pages hold (default 1G; a number of bytes,
      or with K, M or G for powers of 1024).
  pde2d_app --run CASE.json [--set PATH=VALUE]... [--output DIR]
      Runs a saved case without the browser (geometry, mesh, problem,
      assembly, solve) and prints a JSON summary. Exit code 0 when every
      stage ran. --set replaces one value of the case first (PATH dotted
      with [i] indices, VALUE read as JSON, else as text), for example
      --set mesh.spec.max_edge_length.relative=0.02. With --output,
      writes summary.json, mesh.pde2d, mesh.svg, solution.csv,
      solution.vtk, the series solution_NNNN.vtk with solution.vtk.series
      (ParaView opens it with its times), provenance.json and the
      assembled system at assembly.t (stiffness.mtx, mass.mtx,
      damping.mtx, rhs.mtx) there; for an adaptive solve also its last
      mesh (solution-mesh.pde2d).
  pde2d_app --run CASE.json --sweep PATH=V1,V2,... [--sweep ...]
            [--set PATH=VALUE]... [--output DIR]
      Runs the case once per combination of the swept values (the first
      --sweep varies slowest), reusing what a combination does not change,
      and prints one row per run: the values, mesh nodes, solve time and,
      with view.exact_solution in the case, the error and its observed
      order. With --output, writes sweep.json and sweep.csv there.
  pde2d_app --default-case
      Prints the case a new session starts with.
)";

pde2d::app::Server *running_server = nullptr;

extern "C" void on_signal(int) {
  if (running_server != nullptr)
    running_server->stop();
}

std::string read_file(const std::filesystem::path &path) {
  std::ifstream in(path, std::ios::binary);
  if (!in)
    throw std::runtime_error("cannot read " + path.string());
  return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

void write_text(const std::filesystem::path &path, std::string_view text) {
  std::ofstream out(path, std::ios::binary);
  out.write(text.data(), static_cast<std::streamsize>(text.size()));
  if (!out)
    throw std::runtime_error("cannot write " + path.string());
}

void write_file(const std::filesystem::path &path,
                const pde2d::app::ExportedFile &file) {
  std::ofstream out(path, std::ios::binary);
  if (file.stream)
    file.stream([&](std::string_view piece) {
      out.write(piece.data(), static_cast<std::streamsize>(piece.size()));
    });
  else
    out << file.content;
  if (!out)
    throw std::runtime_error("cannot write " + path.string());
}

int run_case(const std::filesystem::path &case_path,
             const std::vector<std::string> &sets,
             const std::vector<pde2d::app::SweepAxis> &sweeps,
             const std::optional<std::filesystem::path> &output) {
  Json case_json = Json::parse(read_file(case_path));
  for (const std::string &set : sets)
    pde2d::app::apply_set_argument(case_json, set);
  pde2d::app::Session session;
  if (!sweeps.empty()) {
    const Json sweep = pde2d::app::run_sweep(session, case_json, sweeps);
    if (output) {
      std::filesystem::create_directories(*output);
      write_text(*output / "sweep.json", sweep.dump(2) + "\n");
      write_text(*output / "sweep.csv", pde2d::app::sweep_csv(sweep));
    }
    std::cout << pde2d::app::sweep_table(sweep);
    bool ok = true;
    for (const Json &row : sweep["rows"])
      ok = ok && row["ok"].get<bool>();
    return ok ? 0 : 1;
  }
  Json summary = session.run(case_json);
  if (output) {
    std::filesystem::create_directories(*output);
    write_text(*output / "summary.json", summary.dump(2) + "\n");
  }
  if (summary["ok"].get<bool>() && output) {
    const pde2d::app::ExportContext context{
        .client = std::string(pde2d::app::local_client),
        .case_name = case_path.filename().string()};
    for (const std::string_view kind :
         {"mesh.pde2d", "mesh.svg", "solution.csv", "solution.vtk",
          "solution-vtk.zip", "provenance.json", "stiffness.mtx", "mass.mtx",
          "damping.mtx", "rhs.mtx", "solution-mesh.pde2d"}) {
      // The solution's own mesh only when it is not the Mesh step's; a run
      // on several meshes (adaptive in time) only as the VTK series.
      const Json &mode = summary["solve"]["mode"];
      const bool adapted = mode == "adaptive" || mode == "adaptive_time";
      if (kind == "solution-mesh.pde2d" && !adapted)
        continue;
      if ((kind == "solution.csv" || kind == "solution.vtk") &&
          mode == "adaptive_time")
        continue;
      const auto file = session.export_file(case_json, kind, context);
      // An archive's files go into the directory itself.
      if (file.members)
        file.members([&](std::string_view name, std::string_view content) {
          write_text(*output / name, content);
        });
      else
        write_file(*output / file.name, file);
    }
  }
  std::cout << summary.dump(2) << '\n';
  return summary["ok"].get<bool>() ? 0 : 1;
}

// "1G", "512M", "300000000": bytes, with K, M or G (optionally followed
// by "iB" or "B") for powers of 1024.
std::size_t parse_bytes(const std::string &text) {
  std::size_t used = 0;
  const double number = std::stod(text, &used);
  std::string unit = text.substr(used);
  for (const std::string_view tail : {"iB", "B"})
    if (unit.size() > tail.size() && unit.ends_with(tail)) {
      unit.resize(unit.size() - tail.size());
      break;
    }
  double scale = 1.0;
  if (unit == "K" || unit == "k")
    scale = 1024.0;
  else if (unit == "M")
    scale = 1024.0 * 1024.0;
  else if (unit == "G")
    scale = 1024.0 * 1024.0 * 1024.0;
  else if (!unit.empty() && unit != "B")
    throw std::invalid_argument("--memory-budget: unknown unit '" + unit +
                                "' (K, M or G)");
  const double bytes = number * scale;
  if (!(bytes >= 1) || bytes > 1e18)
    throw std::invalid_argument("--memory-budget: expected a size above 0");
  return static_cast<std::size_t>(bytes);
}

int serve(pde2d::app::ServerOptions options) {
  const std::string host = options.host;
  pde2d::app::Server server(std::move(options));
  const int port = server.bind();
  running_server = &server;
  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);
  if (!pde2d::app::Server::is_loopback(host))
    std::cout << "pde2d app: WARNING: listening on " << host
              << ", so other machines can reach this app. Every request "
                 "needs the token below; share the address only with "
                 "people you trust. Use --host 127.0.0.1 to keep it on "
                 "this machine."
              << std::endl;
  const std::string query =
      server.token().empty() ? "" : "?token=" + server.token();
  std::cout << "pde2d app: open http://"
            << (host == "0.0.0.0" ? "127.0.0.1" : host) << ':' << port << '/'
            << query << " in a browser (Ctrl+C stops it)" << std::endl;
  server.listen();
  running_server = nullptr;
  return 0;
}

} // namespace

int main(int argc, char **argv) {
  const std::span<char *> args(argv, static_cast<std::size_t>(argc));
  pde2d::app::ServerOptions options;
  options.web_dir = PDE2D_APP_WEB_DIR;
  options.examples_dir = PDE2D_APP_EXAMPLES_DIR;
  std::optional<std::filesystem::path> case_path;
  std::optional<std::filesystem::path> output;
  std::vector<std::string> sets;
  std::vector<pde2d::app::SweepAxis> sweeps;
  try {
    for (std::size_t i = 1; i < args.size(); ++i) {
      const std::string_view arg = args[i];
      const auto value = [&]() -> std::string {
        if (i + 1 >= args.size())
          throw std::invalid_argument(std::string(arg) + " needs a value");
        return args[++i];
      };
      if (arg == "--help" || arg == "-h") {
        std::cout << usage;
        return 0;
      }
      if (arg == "--default-case") {
        std::cout << pde2d::app::Session::default_case().dump(2) << '\n';
        return 0;
      }
      if (arg == "--host")
        options.host = value();
      else if (arg == "--port")
        options.port = std::stoi(value());
      else if (arg == "--web")
        options.web_dir = value();
      else if (arg == "--run")
        case_path = value();
      else if (arg == "--output")
        output = value();
      else if (arg == "--set")
        sets.push_back(value());
      else if (arg == "--sweep")
        sweeps.push_back(pde2d::app::parse_sweep_argument(value()));
      else if (arg == "--memory-budget")
        options.session.memory_budget = parse_bytes(value());
      else if (arg == "--token")
        options.token = value();
      else
        throw std::invalid_argument("unknown argument " + std::string(arg));
    }
    if (!sets.empty() && !case_path)
      throw std::invalid_argument("--set needs --run");
    if (!sweeps.empty() && !case_path)
      throw std::invalid_argument("--sweep needs --run");
    if (case_path)
      return run_case(*case_path, sets, sweeps, output);
    return serve(std::move(options));
  } catch (const std::exception &error) {
    std::cerr << "pde2d_app: " << error.what() << "\n\n" << usage;
    return 2;
  }
}
