// pde2d_app: the graphical interface of the pde2d library (a local web
// server), and a command line that runs a saved case without it.
// See app/README.md.

#include "server.hpp"
#include "session.hpp"

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

#ifndef PDE2D_APP_WEB_DIR
#define PDE2D_APP_WEB_DIR "web"
#endif
#ifndef PDE2D_APP_EXAMPLES_DIR
#define PDE2D_APP_EXAMPLES_DIR ""
#endif

namespace {

using pde2d::app::Json;

constexpr std::string_view usage = R"(Usage:
  pde2d_app [--host HOST] [--port PORT] [--web DIR]
      Starts the interface; open the printed address in a browser.
      Default: 127.0.0.1:8765 and the page files of this checkout.
  pde2d_app --run CASE.json [--output DIR]
      Runs a saved case without the browser (geometry, mesh, problem,
      assembly, solve) and prints a JSON summary. Exit code 0 when every
      stage ran. With --output, writes mesh.pde2d, mesh.svg and
      solution.csv there.
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

void write_file(const std::filesystem::path &path, const std::string &text) {
  std::ofstream out(path, std::ios::binary);
  out << text;
  if (!out)
    throw std::runtime_error("cannot write " + path.string());
}

int run_case(const std::filesystem::path &case_path,
             const std::optional<std::filesystem::path> &output) {
  const Json case_json = Json::parse(read_file(case_path));
  pde2d::app::Session session;
  Json summary = session.run(case_json);
  if (summary["ok"].get<bool>() && output) {
    std::filesystem::create_directories(*output);
    for (const std::string_view kind :
         {"mesh.pde2d", "mesh.svg", "solution.csv"}) {
      const auto file = session.export_file(case_json, kind);
      write_file(*output / file.name, file.content);
    }
  }
  std::cout << summary.dump(2) << '\n';
  return summary["ok"].get<bool>() ? 0 : 1;
}

int serve(pde2d::app::ServerOptions options) {
  const std::string host = options.host;
  pde2d::app::Server server(std::move(options));
  const int port = server.bind();
  running_server = &server;
  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);
  std::cout << "pde2d app: open http://"
            << (host == "0.0.0.0" ? "127.0.0.1" : host) << ':' << port
            << "/ in a browser (Ctrl+C stops it)" << std::endl;
  server.listen();
  running_server = nullptr;
  return 0;
}

} // namespace

int main(int argc, char **argv) {
  const std::span<char *> args(argv, static_cast<std::size_t>(argc));
  pde2d::app::ServerOptions options{.host = "127.0.0.1",
                                    .port = 8765,
                                    .web_dir = PDE2D_APP_WEB_DIR,
                                    .examples_dir = PDE2D_APP_EXAMPLES_DIR};
  std::optional<std::filesystem::path> case_path;
  std::optional<std::filesystem::path> output;
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
      else
        throw std::invalid_argument("unknown argument " + std::string(arg));
    }
    if (case_path)
      return run_case(*case_path, output);
    return serve(std::move(options));
  } catch (const std::exception &error) {
    std::cerr << "pde2d_app: " << error.what() << "\n\n" << usage;
    return 2;
  }
}
