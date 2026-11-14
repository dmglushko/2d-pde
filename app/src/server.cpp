#include "server.hpp"

// cpp-httplib is used only in this file.
#include "httplib.h"

#include <algorithm>
#include <fstream>
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

} // namespace

struct Server::Impl {
  ServerOptions options;
  Session session;
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
