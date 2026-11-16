// The HTTP layer: the page's files and the JSON API on a free port, with
// the checks that keep other sites from driving the server.

#include "server.hpp"

#include "support/check.hpp"

#include "httplib.h"

#include <chrono>
#include <thread>

#ifndef PDE2D_APP_WEB_DIR
#error "PDE2D_APP_WEB_DIR must name app/web"
#endif

using namespace pde2d;
using app::Json;
using test::check;

namespace {

app::ServerOptions test_options() {
  app::ServerOptions options;
  options.port = 0;
  options.web_dir = PDE2D_APP_WEB_DIR;
  options.examples_dir = PDE2D_APP_EXAMPLES_DIR;
  return options;
}

// Starts a server on a free port for the length of one test.
class Running {
public:
  explicit Running(app::ServerOptions options = test_options())
      : server_(std::move(options)), port_(server_.bind()),
        thread_([this] { server_.listen(); }) {}
  ~Running() {
    server_.stop();
    thread_.join();
  }
  httplib::Client client() const {
    httplib::Client client("127.0.0.1", port_);
    client.set_read_timeout(120);
    return client;
  }
  int port() const { return port_; }

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
         const Json arrays_request = {{"mesh", mesh["mesh"]["id"]}};
         const auto arrays = client.Post(
             "/api/mesh/arrays", arrays_request.dump(), "application/json");
         check(arrays && arrays->status == 200 &&
                   arrays->body.size() ==
                       mesh["mesh"]["arrays"]["bytes"].get<std::size_t>(),
               "the mesh's arrays in binary");
         const Json problem = post(client, "/api/problem", case_json, 200);
         check(problem["ok"].get<bool>(), "problem ok");
         const Json assembly = post(client, "/api/assemble", case_json, 200);
         check(assembly["assembly"]["operators"]["stiffness"]["always_zero"] ==
                   false,
               "stiffness");
         const Json solved = post(client, "/api/solve", case_json, 200);
         const std::size_t nodes = mesh["stats"]["nodes"].get<std::size_t>();
         check(solved["values"]["snapshots"] == 1 &&
                   solved["values"]["nodes"] == nodes,
               "the layout of the values");
         check(solved["state"] == "done", "a static solve is done at once");
         const Json range = {
             {"job", solved["job"]}, {"first", 0}, {"count", 1}};
         const auto values =
             client.Post("/api/solve/values", range.dump(), "application/json");
         check(values && values->status == 200 &&
                   values->get_header_value("Content-Type") ==
                       "application/octet-stream" &&
                   values->body.size() == 4 * nodes,
               "a float32 per node");
         post(client, "/api/solve/values",
              {{"job", solved["job"]}, {"first", 0}, {"count", 2}}, 422);
         post(client, "/api/solve/values", {{"job", solved["job"]}, {"x", 1}},
              422);
         const Json progress =
             post(client, "/api/solve/progress", {{"job", solved["job"]}}, 200);
         check(progress["state"] == "done" && progress["job"] == solved["job"],
               "progress");
         post(client, "/api/solve/progress", {{"job", 999}}, 410);
         // A time run goes on in the background until it is done or
         // stopped.
         Json timed = case_json;
         timed["solve"]["mode"] = "time";
         const Json run = post(client, "/api/solve", timed, 200);
         check(run["mode"] == "time" && run["values"]["snapshots"] >= 1,
               "a time run's first chunk");
         const Json stopped =
             post(client, "/api/solve/stop", {{"job", run["job"]}}, 200);
         check(stopped["state"] == "stopped" || stopped["state"] == "done",
               "stopped");
         const Json preset =
             post(client, "/api/preset",
                  {{"case", case_json}, {"preset", "wave"}}, 200);
         check(preset["problem"]["equation"]["m2"] == "1" &&
                   preset["solve"]["mode"] == "time",
               "a preset fills the problem and the solve section");
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
         check(list.size() == 6, "six examples");
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
         app::ServerOptions options = test_options();
         options.max_body_bytes = 1024;
         app::Server server(options);
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
         // A POST that says it comes from another page is refused; from
         // this server's own page it is served.
         const std::string own =
             "http://127.0.0.1:" + std::to_string(running.port());
         const Json case_json = app::Session::default_case();
         const auto foreign =
             client.Post("/api/geometry", {{"Origin", "http://evil.example"}},
                         case_json.dump(), "application/json");
         check(foreign && foreign->status == 403, "Origin checked");
         const auto same = client.Post("/api/geometry", {{"Origin", own}},
                                       case_json.dump(), "application/json");
         check(same && same->status == 200, "own Origin");
       }},
      {"a token is required when there is one",
       [] {
         app::ServerOptions options = test_options();
         options.token = "s3cret";
         const Running running(options);
         auto client = running.client();
         const auto missing = client.Get("/api/catalog");
         check(missing && missing->status == 403, "no token");
         check(Json::parse(missing->body)["error"]["message"]
                       .get<std::string>()
                       .find("token") != std::string::npos,
               "says so");
         const auto wrong =
             client.Get("/api/catalog", {{"X-PDE2D-Token", "s3creT"}});
         check(wrong && wrong->status == 403, "a wrong token");
         const auto header =
             client.Get("/api/catalog", {{"X-PDE2D-Token", "s3cret"}});
         check(header && header->status == 200, "in the header");
         // The address carries it to the page only; API calls send the
         // header.
         const auto query = client.Get("/api/catalog?token=s3cret");
         check(query && query->status == 403, "not in an API address");
         const auto page = client.Get("/");
         check(page && page->status == 200, "the page itself needs none");
         // A host beyond this machine always has a token, made at random.
         app::ServerOptions open = test_options();
         open.host = "0.0.0.0";
         const app::Server server(open);
         check(server.token().size() == 32, "a random token");
         check(app::Server(test_options()).token().empty(), "none on loopback");
       }},
      {"too many waiting requests are answered 429",
       [] {
         app::ServerOptions options = test_options();
         options.max_waiting = 0; // one request at a time
         const Running running(options);
         Json slow = app::Session::default_case();
         slow["mesh"]["spec"]["max_edge_length"]["relative"] = 0.003;
         std::thread first([&] {
           auto client = running.client();
           (void)client.Post("/api/mesh", slow.dump(), "application/json");
         });
         std::this_thread::sleep_for(std::chrono::milliseconds(300));
         auto client = running.client();
         const auto busy =
             client.Post("/api/geometry", app::Session::default_case().dump(),
                         "application/json");
         first.join();
         check(busy && busy->status == 429,
               "busy: " + std::to_string(busy ? busy->status : 0));
         check(Json::parse(busy->body)["error"]["kind"] == "busy", "kind");
         const auto later =
             client.Post("/api/geometry", app::Session::default_case().dump(),
                         "application/json");
         check(later && later->status == 200, "served once it is free");
       }},
      {"status and lightweight calls answer while the session is busy",
       [] {
         app::ServerOptions options = test_options();
         options.max_waiting = 0; // one waiting request at a time
         const Running running(options);
         Json slow = app::Session::default_case();
         slow["mesh"]["spec"]["max_edge_length"]["relative"] = 0.003;
         std::jthread first([&] {
           auto client = running.client();
           (void)client.Post("/api/mesh", {{"X-PDE2D-Client", "mesher"}},
                             slow.dump(), "application/json");
         });
         auto client = running.client();
         Json status;
         const auto until =
             std::chrono::steady_clock::now() + std::chrono::seconds(10);
         while (std::chrono::steady_clock::now() < until) {
           const auto answer = client.Get("/api/status");
           check(answer && answer->status == 200, "status answers");
           status = Json::parse(answer->body);
           if (!status["busy"].is_null())
             break;
         }
         check(status["busy"]["task"] == "mesh", status.dump());
         check(status["busy"]["client"] == "mesher" &&
                   status["busy"]["yours"] == false,
               "whose");
         check(status["busy"]["seconds"].is_number() &&
                   status["busy"]["since"].is_string(),
               "since when");
         check(status["waiting"].get<int>() >= 1 &&
                   status["memory"]["budget"].is_number(),
               "waiting and memory");
         // A progress call does not wait and is not refused as busy.
         const auto start = std::chrono::steady_clock::now();
         const auto progress =
             client.Post("/api/solve/progress", Json{{"job", 5}}.dump(),
                         "application/json");
         const double seconds = std::chrono::duration<double>(
                                    std::chrono::steady_clock::now() - start)
                                    .count();
         check(progress && progress->status == 410,
               "expired, not 429: " +
                   std::to_string(progress ? progress->status : 0));
         check(seconds < 0.5, "at once: " + std::to_string(seconds) + " s");
         check(client.Get("/api/status")->status == 200, "still busy, answers");
         first.join();
         const Json idle = Json::parse(client.Get("/api/status")->body);
         check(idle["busy"].is_null(), "idle afterwards");
       }},
      {"the case schema and the headers every answer has",
       [] {
         const Running running;
         auto client = running.client();
         const auto schema = client.Get("/api/case-schema");
         check(schema && schema->status == 200, "served");
         check(schema->get_header_value("Content-Type") ==
                   "application/schema+json",
               "its type");
         check(Json::parse(schema->body)["title"] == "pde2d app case", "it");
         const auto page = client.Get("/");
         check(page->get_header_value("Referrer-Policy") == "no-referrer",
               "no referrer from the page");
         check(schema->get_header_value("Referrer-Policy") == "no-referrer",
               "nor from the API");
       }},
      {"runs are per client, and exports wait for a done run",
       [] {
         const Running running;
         auto client = running.client();
         const Json case_json = app::Session::default_case();
         const httplib::Headers a = {{"X-PDE2D-Client", "tab-a"}};
         const httplib::Headers b = {{"X-PDE2D-Client", "tab-b"}};
         const auto early = client.Post("/api/export/solution.csv", a,
                                        case_json.dump(), "application/json");
         check(early && early->status == 409, "not solved yet");
         check(Json::parse(early->body)["error"]["kind"] == "not_ready",
               "not ready");
         const auto solved =
             client.Post("/api/solve", a, case_json.dump(), "application/json");
         check(solved && solved->status == 200, "solved");
         const Json job = {{"job", Json::parse(solved->body)["job"]}};
         const auto mine = client.Post("/api/solve/progress", a, job.dump(),
                                       "application/json");
         check(mine && mine->status == 200, "a's run for a");
         const auto theirs = client.Post("/api/solve/progress", b, job.dump(),
                                         "application/json");
         check(theirs && theirs->status == 410, "not b's");
         check(Json::parse(theirs->body)["error"]["kind"] == "expired",
               "an unknown run is expired");
         const auto bad =
             client.Post("/api/solve/progress", {{"X-PDE2D-Client", "a b"}},
                         job.dump(), "application/json");
         check(bad && bad->status == 422, "a malformed client id");
         // The CSV streams once the run is done.
         const auto csv = client.Post("/api/export/solution.csv",
                                      {{"X-PDE2D-Client", "tab-a"},
                                       {"X-PDE2D-Case-Name", "my%20case.json"}},
                                      case_json.dump(), "application/json");
         check(csv && csv->status == 200, "exported");
         check(csv->body.starts_with("# produced by pde2d_app"), "provenance");
         check(csv->body.find("# case: my case.json\n") != std::string::npos,
               "the case's name");
         check(csv->body.find("\nx,y,u(t=0)\n") != std::string::npos,
               "the table");
       }},
      {"an error names its field",
       [] {
         const Running running;
         auto client = running.client();
         Json case_json = app::Session::default_case();
         case_json["mesh"]["refiner"]["options"]["min_angle_degrees"] = 50;
         const Json error = post(client, "/api/mesh", case_json, 422)["error"];
         check(error["path"] == "mesh.refiner.options.min_angle_degrees",
               error.dump());
         check(error["message"].get<std::string>().starts_with(
                   "mesh.refiner.options.min_angle_degrees: "),
               "readable");
       }},
  });
}
