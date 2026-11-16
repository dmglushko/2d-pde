#include "server.hpp"

// cpp-httplib is used only in this file.
#include "httplib.h"

#include <algorithm>
#include <atomic>
#include <cctype>
#include <cstdint>
#include <filesystem>
#include <format>
#include <fstream>
#include <iterator>
#include <random>
#include <span>
#include <stdexcept>
#include <string_view>
#include <tuple>
#include <utility>
#include <vector>

namespace pde2d::app {

namespace {

constexpr const char *client_header = "X-PDE2D-Client";
constexpr const char *token_header = "X-PDE2D-Token";
constexpr const char *case_name_header = "X-PDE2D-Case-Name";

// The host name of a Host header, without its port.
std::string_view host_name(std::string_view header) {
  if (header.starts_with('[')) {
    const auto end = header.find(']');
    return end == std::string_view::npos ? header : header.substr(1, end - 1);
  }
  return header.substr(0, header.find(':'));
}

bool same_text(std::string_view a, std::string_view b) {
  return a.size() == b.size() &&
         std::ranges::equal(a, b, [](unsigned char x, unsigned char y) {
           return std::tolower(x) == std::tolower(y);
         });
}

// Whether an Origin header names the same host and port as the Host
// header ("http://127.0.0.1:8765" and "127.0.0.1:8765").
bool origin_matches(std::string_view origin, std::string_view host) {
  const auto scheme = origin.find("://");
  if (scheme == std::string_view::npos)
    return false;
  return same_text(origin.substr(scheme + 3), host);
}

// Compares in time that does not depend on where the texts differ.
bool same_secret(std::string_view a, std::string_view b) {
  if (a.size() != b.size())
    return false;
  unsigned char diff = 0;
  for (std::size_t i = 0; i < a.size(); ++i)
    diff |= static_cast<unsigned char>(a[i] ^ b[i]);
  return diff == 0;
}

std::string random_token() {
  std::random_device device;
  std::string out;
  for (int i = 0; i < 4; ++i)
    out += std::format("{:08x}", device());
  return out;
}

void send_json(httplib::Response &response, const Json &body,
               int status = 200) {
  response.status = status;
  response.set_content(body.dump(), "application/json");
}

void send_error(httplib::Response &response, int status, std::string_view stage,
                std::string_view kind, std::string_view message,
                std::string_view path = {}) {
  Json error = Json::object();
  error["stage"] = stage;
  error["kind"] = kind;
  error["message"] = message;
  error["path"] = path.empty() ? Json(nullptr) : Json(path);
  Json body = Json::object();
  body["error"] = std::move(error);
  send_json(response, body, status);
}

int status_of(const StageError &error) {
  if (error.kind() == "internal")
    return 500;
  // A result that is not ready yet, a run a newer solve replaced.
  if (error.kind() == "not_ready" || error.kind() == "replaced")
    return 409;
  // A run that was dropped or that this server never had.
  if (error.kind() == "expired")
    return 410;
  return 422;
}

// The client of a request: its X-PDE2D-Client header (letters, digits,
// '-' and '_', at most 64), or local_client without one.
std::string client_of(const httplib::Request &request) {
  std::string id = request.get_header_value(client_header);
  if (id.empty())
    return std::string(local_client);
  const bool valid = id.size() <= 64 && std::ranges::all_of(id, [](char c) {
                       return std::isalnum(static_cast<unsigned char>(c)) ||
                              c == '-' || c == '_';
                     });
  if (!valid)
    throw StageError("request", "input",
                     std::string(client_header) +
                         ": letters, digits, '-' and '_', at most 64",
                     "");
  return id;
}

std::string read_text(const std::filesystem::path &path) {
  std::ifstream in(path, std::ios::binary);
  return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

} // namespace

struct Server::Impl {
  ServerOptions options;
  std::unique_ptr<Session> session;
  httplib::Server http;
  // Session calls being served or waiting for the session's mutex.
  std::atomic<int> active{0};

  // Parses the request body and runs `work(body, client, response)`,
  // turning failures into error responses: 400 for a body that is not
  // JSON, 409 for a result that is not ready or a run that was replaced,
  // 410 for a run that is gone, 422 for input the library refused or an
  // algorithm that could not finish, 429 when too many requests already
  // wait for the session, 500 otherwise. A call that does not wait for the
  // session (`waits` false: progress, stop, values, probes) is never
  // refused for being busy and does not count.
  template <class Work>
  void handle(const httplib::Request &request, httplib::Response &response,
              Work work, bool waits = true) {
    struct Slot {
      std::atomic<int> *count;
      ~Slot() {
        if (count != nullptr)
          --*count;
      }
    };
    if (waits && ++active > options.max_waiting + 1) {
      --active;
      send_error(response, 429, "request", "busy",
                 "the app is busy with earlier requests; try again in a "
                 "moment");
      return;
    }
    const Slot slot{waits ? &active : nullptr};
    Json body;
    try {
      body = Json::parse(request.body);
    } catch (const Json::parse_error &error) {
      send_error(response, 400, "request", "input",
                 std::string("the request is not JSON: ") + error.what());
      return;
    }
    try {
      work(body, client_of(request), response);
    } catch (const StageError &error) {
      send_error(response, status_of(error), error.stage(), error.kind(),
                 error.what(), error.path());
    } catch (const std::exception &error) {
      send_error(response, 500, "server", "internal", error.what());
    }
  }
};

bool Server::is_loopback(std::string_view host) {
  return host == "127.0.0.1" || host == "localhost" || host == "::1";
}

Server::Server(ServerOptions options) : impl_(std::make_unique<Impl>()) {
  impl_->options = std::move(options);
  const std::filesystem::path &web = impl_->options.web_dir;
  if (!std::filesystem::is_regular_file(web / "index.html"))
    throw std::invalid_argument("the web directory '" + web.string() +
                                "' has no index.html");
  const bool loopback = is_loopback(impl_->options.host);
  if (!loopback && impl_->options.token.empty())
    impl_->options.token = random_token();
  impl_->session = std::make_unique<Session>(impl_->options.session);
  httplib::Server &http = impl_->http;
  Session &session = *impl_->session;
  Impl &impl = *impl_;

  http.set_mount_point("/", web.string());
  http.set_payload_max_length(impl_->options.max_body_bytes);
  // No page this server sends names its address (and the token the
  // first page load carries) to another site.
  http.set_default_headers(
      {{"Cache-Control", "no-store"}, {"Referrer-Policy", "no-referrer"}});

  // A page on another site must not drive this server: API requests must
  // name this machine in their Host header when it listens on loopback
  // (against DNS rebinding), carry the token when there is one, come from
  // this server's own page when they say where from (Origin), and send
  // JSON (which a cross-site form cannot do without a preflight that this
  // server never answers).
  const std::string token = impl_->options.token;
  http.set_pre_routing_handler(
      [loopback, token](const httplib::Request &request,
                        httplib::Response &response) {
        if (!request.path.starts_with("/api/"))
          return httplib::Server::HandlerResponse::Unhandled;
        const std::string host = request.get_header_value("Host");
        if (loopback && !is_loopback(host_name(host))) {
          send_error(response, 403, "request", "input",
                     "requests must come from this machine");
          return httplib::Server::HandlerResponse::Handled;
        }
        // The token comes in its header only: ?token= is for opening the
        // page, which keeps it and removes it from the address bar.
        if (!token.empty()) {
          const std::string given = request.get_header_value(token_header);
          if (!same_secret(given, token)) {
            send_error(response, 403, "request", "input",
                       "this server needs its token: open the address it "
                       "printed, with ?token=...");
            return httplib::Server::HandlerResponse::Handled;
          }
        }
        if (request.method == "POST") {
          if (request.has_header("Origin") &&
              !origin_matches(request.get_header_value("Origin"), host)) {
            send_error(response, 403, "request", "input",
                       "requests must come from this server's own page");
            return httplib::Server::HandlerResponse::Handled;
          }
          if (!request.get_header_value("Content-Type")
                   .starts_with("application/json")) {
            send_error(response, 415, "request", "input",
                       "send the request as application/json");
            return httplib::Server::HandlerResponse::Handled;
          }
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
  http.Get("/api/case-schema",
           [](const httplib::Request &, httplib::Response &response) {
             response.status = 200;
             response.set_content(Session::case_schema().dump(2),
                                  "application/schema+json");
           });
  // What the session is busy with; never waits for it.
  http.Get("/api/status", [&session, &impl](const httplib::Request &request,
                                            httplib::Response &response) {
    try {
      Json body = session.status(client_of(request));
      body["waiting"] = std::max(0, impl.active.load());
      send_json(response, body);
    } catch (const StageError &error) {
      send_error(response, status_of(error), error.stage(), error.kind(),
                 error.what(), error.path());
    }
  });

  // A call answered in JSON: stage(body, client). `waits`: whether it
  // waits for the session mutex (false for the calls that read only the
  // runs' bookkeeping).
  const auto route = [&http, &impl](const std::string &path, auto stage,
                                    bool waits = true) {
    http.Post(path, [stage, &impl, waits](const httplib::Request &request,
                                          httplib::Response &response) {
      impl.handle(
          request, response,
          [&](const Json &body, const std::string &client,
              httplib::Response &out) { send_json(out, stage(body, client)); },
          waits);
    });
  };
  // A call answered in bytes.
  const auto binary = [&http, &impl](const std::string &path, auto stage,
                                     bool waits = true) {
    http.Post(path, [stage, &impl, waits](const httplib::Request &request,
                                          httplib::Response &response) {
      impl.handle(
          request, response,
          [&](const Json &body, const std::string &client,
              httplib::Response &out) {
            out.set_content(stage(body, client), "application/octet-stream");
          },
          waits);
    });
  };
  route("/api/geometry",
        [&session](const Json &body, const std::string &client) {
          return session.geometry(body, client);
        });
  route("/api/mesh", [&session](const Json &body, const std::string &client) {
    return session.mesh(body, client);
  });
  binary("/api/mesh/arrays",
         [&session](const Json &body, const std::string &client) {
           return session.mesh_arrays(body, client);
         });
  route("/api/problem",
        [&session](const Json &body, const std::string &client) {
          return session.check_problem(body, client);
        });
  route("/api/assemble",
        [&session](const Json &body, const std::string &client) {
          return session.assemble(body, client);
        });
  route("/api/solve", [&session](const Json &body, const std::string &client) {
    return session.solve(body, client);
  });
  // A run's progress and stopping it: {"job": n}.
  static constexpr std::string_view job_only[] = {"job"};
  const auto job_request = [](const Json &body) {
    return staged("solve", [&] { return request_job(body, job_only); });
  };
  route(
      "/api/solve/progress",
      [&session, job_request](const Json &body, const std::string &client) {
        return session.solve_progress(job_request(body), client);
      },
      false);
  // The client's run of a case, if it still has one (a reloaded page).
  route(
      "/api/solve/current",
      [&session](const Json &body, const std::string &client) {
        return session.current_run(body, client);
      },
      false);
  route(
      "/api/solve/stop",
      [&session, job_request](const Json &body, const std::string &client) {
        return session.stop_solve(job_request(body), client);
      },
      false);
  // Snapshots of a run as float32 (Session::solution_values_float32):
  // {"job": n, "first": k, "count": c}.
  binary(
      "/api/solve/values",
      [&session](const Json &body, const std::string &client) {
        static constexpr std::string_view known[] = {"job", "first", "count"};
        const auto [job, first, count] = staged("solve", [&] {
          return std::tuple(request_job(body, known),
                            request_count(body, "first"),
                            request_count(body, "count"));
        });
        return session.solution_values_float32(job, first, count, client);
      },
      false);
  route("/api/solve/reference",
        [&session](const Json &body, const std::string &client) {
          return session.keep_reference(body, client);
        });
  route("/api/solve/reference/clear",
        [&session](const Json &, const std::string &client) {
          return session.clear_reference(client);
        });
  route("/api/solve/compare",
        [&session](const Json &body, const std::string &client) {
          return session.compare(body, client);
        });
  binary("/api/solve/compare/difference",
         [&session](const Json &body, const std::string &client) {
           return session.compare_difference(body, client);
         });
  route(
      "/api/solve/probes",
      [&session](const Json &body, const std::string &client) {
        return session.probes(body, client);
      },
      false);
  route("/api/preset", [&session](const Json &body, const std::string &client) {
    return session.apply_preset(body, client);
  });
  route("/api/field", [&session](const Json &body, const std::string &client) {
    return session.field(body, client);
  });

  http.Post(R"(/api/export/([a-z0-9.]+))", [&session, &impl](
                                               const httplib::Request &request,
                                               httplib::Response &response) {
    impl.handle(
        request, response,
        [&](const Json &body, const std::string &client,
            httplib::Response &out) {
          const ExportContext context{
              .client = client,
              .case_name = httplib::detail::decode_url(
                  request.get_header_value(case_name_header), false)};
          ExportedFile file =
              session.export_file(body, request.matches[1].str(), context);
          out.set_header("Content-Disposition",
                         "attachment; filename=\"" + file.name + "\"");
          if (!file.stream) {
            out.set_content(file.content, file.content_type);
            return;
          }
          // Written as it is sent; the stream holds what it
          // needs (no expressions), so it runs outside the
          // session's mutex.
          out.set_chunked_content_provider(
              file.content_type, [stream = std::move(file.stream)](
                                     std::size_t, httplib::DataSink &sink) {
                stream([&](std::string_view piece) {
                  if (!piece.empty())
                    sink.write(piece.data(), piece.size());
                });
                sink.done();
                return true;
              });
        });
  });
}

Server::~Server() = default;

const std::string &Server::token() const { return impl_->options.token; }

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
