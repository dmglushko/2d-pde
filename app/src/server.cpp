#include "server.hpp"

// cpp-httplib is used only in this file.
#include "httplib.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <span>
#include <tuple>
#include <iterator>
#include <stdexcept>
#include <vector>
#include <string_view>
#include <utility>

namespace pde2d::app {

namespace {

bool is_loopback(std::string_view host) {
  return host == "127.0.0.1" || host == "localhost" || host == "::1";
}

// The host name of a Host header, without its port.
std::string_view host_name(std::string_view header) {
  if (header.starts_with('[')) {
    const auto end = header.find(']');
    return end == std::string_view::npos ? header : header.substr(1, end - 1);
  }
  return header.substr(0, header.find(':'));
}

void send_json(httplib::Response &response, const Json &body,
               int status = 200) {
  response.status = status;
  response.set_content(body.dump(), "application/json");
}

void send_error(httplib::Response &response, int status, std::string_view stage,
                std::string_view kind, std::string_view message) {
  Json error = Json::object();
  error["stage"] = stage;
  error["kind"] = kind;
  error["message"] = message;
  Json body = Json::object();
  body["error"] = std::move(error);
  send_json(response, body, status);
}

// Parses the request body and runs `work` on it, turning failures into
// error responses: 400 for a body that is not JSON, 422 for input the
// library refused or an algorithm that could not finish, 500 otherwise.
template <class Work>
void handle(const httplib::Request &request, httplib::Response &response,
            Work work) {
  Json body;
  try {
    body = Json::parse(request.body);
  } catch (const Json::parse_error &error) {
    send_error(response, 400, "request", "input",
               std::string("the request is not JSON: ") + error.what());
    return;
  }
  try {
    work(body, response);
  } catch (const StageError &error) {
    send_error(response, error.kind() == "internal" ? 500 : 422, error.stage(),
               error.kind(), error.what());
  } catch (const std::exception &error) {
    send_error(response, 500, "server", "internal", error.what());
  }
}

std::string read_text(const std::filesystem::path &path) {
  std::ifstream in(path, std::ios::binary);
  return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

// A run's id from a request {"job": n, ...} with only the keys `known`.
std::uint64_t job_of(const Json &body,
                     std::span<const std::string_view> known) {
  require_object(body, "request");
  reject_unknown_keys(body, "request", known);
  const double job = require_number(member(body, "job"), "job");
  if (job != std::floor(job) || job < 1 || job > 9e15)
    throw std::invalid_argument("job: expected a run's id");
  return static_cast<std::uint64_t>(job);
}

std::size_t count_of(const Json &body, std::string_view key) {
  const double value = require_number(member(body, key), std::string(key));
  if (value != std::floor(value) || value < 0 || value > 1e9)
    throw std::invalid_argument(std::string(key) +
                                ": expected a whole number from 0");
  return static_cast<std::size_t>(value);
}

// Runs `read` on the request, as a StageError of the solve stage when it
// refuses it.
template <class Read> auto read_request(Read read) {
  try {
    return read();
  } catch (const std::invalid_argument &error) {
    throw StageError("solve", "input", error.what());
  }
}

} // namespace

struct Server::Impl {
  ServerOptions options;
  // Time runs continue in the background, chunk by chunk.
  Session session{SessionOptions{.background_runs = true}};
  httplib::Server http;
};

Server::Server(ServerOptions options) : impl_(std::make_unique<Impl>()) {
  impl_->options = std::move(options);
  const std::filesystem::path &web = impl_->options.web_dir;
  if (!std::filesystem::is_regular_file(web / "index.html"))
    throw std::invalid_argument("the web directory '" + web.string() +
                                "' has no index.html");
  httplib::Server &http = impl_->http;
  Session &session = impl_->session;
  const bool check_host = is_loopback(impl_->options.host);

  http.set_mount_point("/", web.string());
  http.set_payload_max_length(impl_->options.max_body_bytes);
  http.set_default_headers({{"Cache-Control", "no-store"}});

  // A page on another site must not drive this server: API requests must
  // name this machine in their Host header (against DNS rebinding) and
  // send JSON (which a cross-site form cannot do without a preflight that
  // this server never answers).
  http.set_pre_routing_handler([check_host](const httplib::Request &request,
                                            httplib::Response &response) {
    if (!request.path.starts_with("/api/"))
      return httplib::Server::HandlerResponse::Unhandled;
    if (check_host &&
        !is_loopback(host_name(request.get_header_value("Host")))) {
      send_error(response, 403, "request", "input",
                 "requests must come from this machine");
      return httplib::Server::HandlerResponse::Handled;
    }
    if (request.method == "POST" && !request.get_header_value("Content-Type")
                                         .starts_with("application/json")) {
      send_error(response, 415, "request", "input",
                 "send the request as application/json");
      return httplib::Server::HandlerResponse::Handled;
    }
    return httplib::Server::HandlerResponse::Unhandled;
  });

  http.Get("/api/catalog",
           [](const httplib::Request &, httplib::Response &response) {
             send_json(response, Session::catalog());
           });
  // The example cases: their names and notes, and each case by name.
  const std::filesystem::path examples = impl_->options.examples_dir;
  http.Get("/api/examples", [examples](const httplib::Request &,
                                       httplib::Response &response) {
    Json list = Json::array();
    std::vector<std::filesystem::path> files;
    if (!examples.empty() && std::filesystem::is_directory(examples))
      for (const auto &entry : std::filesystem::directory_iterator(examples))
        if (entry.path().extension() == ".json")
          files.push_back(entry.path());
    std::ranges::sort(files);
    for (const auto &file : files) {
      Json item = Json::object();
      item["name"] = file.filename().string();
      try {
        item["notes"] = member(Json::parse(read_text(file)), "notes");
      } catch (const std::exception &) {
        continue; // not a readable case: not offered
      }
      list.push_back(std::move(item));
    }
    send_json(response, list);
  });
  http.Get(
      R"(/api/examples/([a-z0-9_]+\.json))",
      [examples](const httplib::Request &request, httplib::Response &response) {
        const auto file = examples / request.matches[1].str();
        if (examples.empty() || !std::filesystem::is_regular_file(file)) {
          send_error(response, 404, "request", "input", "no such example");
          return;
        }
        response.set_content(read_text(file), "application/json");
      });
  http.Get("/api/default-case",
           [](const httplib::Request &, httplib::Response &response) {
             send_json(response, Session::default_case());
           });

  const auto route = [&http](const std::string &path, auto stage) {
    http.Post(path, [stage](const httplib::Request &request,
                            httplib::Response &response) {
      handle(request, response, [&](const Json &body, httplib::Response &out) {
        send_json(out, stage(body));
      });
    });
  };
  route("/api/geometry",
        [&session](const Json &body) { return session.geometry(body); });
  route("/api/mesh",
        [&session](const Json &body) { return session.mesh(body); });
  route("/api/problem",
        [&session](const Json &body) { return session.check_problem(body); });
  route("/api/assemble",
        [&session](const Json &body) { return session.assemble(body); });
  route("/api/solve",
        [&session](const Json &body) { return session.solve(body); });
  // A run's progress and stopping it: {"job": n}.
  route("/api/solve/progress", [&session](const Json &body) {
    static constexpr std::string_view known[] = {"job"};
    return session.solve_progress(
        read_request([&] { return job_of(body, known); }));
  });
  route("/api/solve/stop", [&session](const Json &body) {
    static constexpr std::string_view known[] = {"job"};
    return session.stop_solve(
        read_request([&] { return job_of(body, known); }));
  });
  // Snapshots of a run as float32 (Session::solution_values_float32):
  // {"job": n, "first": k, "count": c}.
  http.Post("/api/solve/values", [&session](const httplib::Request &request,
                                            httplib::Response &response) {
    handle(request, response, [&](const Json &body, httplib::Response &out) {
      static constexpr std::string_view known[] = {"job", "first", "count"};
      const auto [job, first, count] = read_request([&] {
        return std::tuple(job_of(body, known), count_of(body, "first"),
                          count_of(body, "count"));
      });
      out.set_content(session.solution_values_float32(job, first, count),
                      "application/octet-stream");
    });
  });
  route("/api/preset",
        [&session](const Json &body) { return session.apply_preset(body); });
  route("/api/field",
        [&session](const Json &body) { return session.field(body); });

  http.Post(R"(/api/export/([a-z0-9.]+))", [&session](
                                               const httplib::Request &request,
                                               httplib::Response &response) {
    handle(request, response, [&](const Json &body, httplib::Response &out) {
      const ExportedFile file =
          session.export_file(body, request.matches[1].str());
      out.set_header("Content-Disposition",
                     "attachment; filename=\"" + file.name + "\"");
      out.set_content(file.content, file.content_type);
    });
  });
}

Server::~Server() = default;

int Server::bind() {
  if (impl_->options.port == 0) {
    const int port = impl_->http.bind_to_any_port(impl_->options.host);
    if (port < 0)
      throw std::runtime_error("cannot listen on " + impl_->options.host);
    impl_->options.port = port;
    return port;
  }
  if (!impl_->http.bind_to_port(impl_->options.host, impl_->options.port))
    throw std::runtime_error(
        "cannot listen on " + impl_->options.host + ":" +
        std::to_string(impl_->options.port) +
        " (is another copy running? choose another port with --port)");
  return impl_->options.port;
}

void Server::listen() { impl_->http.listen_after_bind(); }

void Server::stop() { impl_->http.stop(); }

} // namespace pde2d::app
