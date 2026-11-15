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
      : server_({.host = "127.0.0.1",
                 .port = 0,
                 .web_dir = PDE2D_APP_WEB_DIR,
                 .examples_dir = PDE2D_APP_EXAMPLES_DIR}),
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
         check(assembly["assembly"]["operators"]["stiffness"]["always_zero"] ==
                   false,
               "stiffness");
         const Json solved = post(client, "/api/solve", case_json, 200);
         check(solved["values"].size() == 1 &&
                   solved["values"][0].size() ==
                       mesh["stats"]["nodes"].get<std::size_t>(),
               "a value per node");
         const auto file = client.Post("/api/export/mesh.svg", case_json.dump(),
                                       "application/json");
         check(file && file->status == 200 &&
                   file->get_header_value("Content-Disposition")
                           .find("mesh.svg") != std::string::npos,
               "export");
       }},
      {"the examples are listed and served",
       [] {
         const Running running;
         auto client = running.client();
         const Json list = Json::parse(client.Get("/api/examples")->body);
         check(list.size() == 4, "four examples");
         const std::string name = list[0]["name"].get<std::string>();
         const auto example = client.Get("/api/examples/" + name);
         check(example && example->status == 200, "served");
         check(Json::parse(example->body)["format"] == "pde2d-app-case",
               "a case");
         const auto missing = client.Get("/api/examples/none.json");
         check(missing && missing->status == 404, "missing");
         const auto escape = client.Get("/api/examples/..%2Fsecret.json");
         check(escape && escape->status != 200, "no path escape");
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
      {"large bodies are refused",
       [] {
         app::Server server({.host = "127.0.0.1",
                             .port = 0,
                             .web_dir = PDE2D_APP_WEB_DIR,
                             .examples_dir = {},
                             .max_body_bytes = 1024});
         const int port = server.bind();
         std::thread thread([&] { server.listen(); });
         httplib::Client client("127.0.0.1", port);
         const auto response = client.Post("/api/mesh", std::string(4096, ' '),
                                           "application/json");
         const int status = response ? response->status : 0;
         server.stop();
         thread.join();
         check(status == 413, "status " + std::to_string(status));
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
