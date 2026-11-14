// The HTTP layer: the page's files and the JSON API on a free port, with
// the checks that keep other sites from driving the server.

#include "server.hpp"

#include "support/check.hpp"

#include "httplib.h"

#include <thread>

#ifndef PDE2D_APP_WEB_DIR
#error "PDE2D_APP_WEB_DIR must name app/web"
#endif

using namespace pde2d;
using app::Json;
using test::check;

namespace {

// Starts a server on a free port for the length of one test.
class Running {
public:
  Running()
      : server_({.host = "127.0.0.1", .port = 0, .web_dir = PDE2D_APP_WEB_DIR}),
        port_(server_.bind()), thread_([this] { server_.listen(); }) {}
  ~Running() {
    server_.stop();
    thread_.join();
  }
  httplib::Client client() const {
    httplib::Client client("127.0.0.1", port_);
    client.set_read_timeout(120);
    return client;
  }

private:
  app::Server server_;
  int port_;
  std::thread thread_;
};

Json post(httplib::Client &client, const std::string &path, const Json &body,
          int expected_status) {
  const auto response = client.Post(path, body.dump(), "application/json");
  check(static_cast<bool>(response), "a response from " + path);
  check(response->status == expected_status,
        path + " returned " + std::to_string(response->status) + ": " +
            response->body);
  return Json::parse(response->body);
}

} // namespace

int main() {
  return test::run({
      {"the page and the catalog are served",
       [] {
         const Running running;
         auto client = running.client();
         const auto page = client.Get("/");
         check(page && page->status == 200, "index.html");
         check(page->body.find("<title>") != std::string::npos, "html");
         const auto catalog = client.Get("/api/catalog");
         check(catalog && catalog->status == 200, "catalog");
         check(Json::parse(catalog->body)["sections"][0]["id"] == "mesh",
               "catalog content");
       }},
      {"the stages answer in JSON",
       [] {
         const Running running;
         auto client = running.client();
         const Json case_json =
             Json::parse(client.Get("/api/default-case")->body);
         const Json geometry = post(client, "/api/geometry", case_json, 200);
         check(geometry["regions"].size() == 4, "regions");
         const Json mesh = post(client, "/api/mesh", case_json, 200);
         check(mesh["target_met"].get<bool>(), "target met");
         const Json problem = post(client, "/api/problem", case_json, 200);
         check(problem["ok"].get<bool>(), "problem ok");
         const Json assembly = post(client, "/api/assemble", case_json, 200);
         check(assembly["assembly"]["operators"]["stiffness"]["is_zero"] ==
                   false,
               "stiffness");
         const auto file = client.Post("/api/export/mesh.svg", case_json.dump(),
                                       "application/json");
         check(file && file->status == 200 &&
                   file->get_header_value("Content-Disposition")
                           .find("mesh.svg") != std::string::npos,
               "export");
       }},
      {"errors are JSON with their stage",
       [] {
         const Running running;
         auto client = running.client();
         Json case_json = Json::parse(client.Get("/api/default-case")->body);
         case_json["mesh"]["refiner"]["type"] = "magic";
         const Json error = post(client, "/api/mesh", case_json, 422);
         check(error["error"]["stage"] == "mesh", "stage");
         check(error["error"]["kind"] == "input", "kind");
         const auto bad = client.Post("/api/mesh", "{", "application/json");
         check(bad && bad->status == 400, "not JSON");
       }},
      {"other sites cannot drive the server",
       [] {
         const Running running;
         auto client = running.client();
         const auto form = client.Post("/api/mesh", "{}", "text/plain");
         check(form && form->status == 415, "JSON only");
         httplib::Headers headers = {{"Host", "evil.example:80"}};
         const auto rebound = client.Get("/api/catalog", headers);
         check(rebound && rebound->status == 403, "Host checked");
       }},
  });
}
