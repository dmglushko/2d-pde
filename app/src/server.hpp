#pragma once

#include "session.hpp"

#include <filesystem>
#include <cstddef>
#include <functional>
#include <memory>
#include <string>
#include <string_view>

// The local HTTP server: the page's files and a JSON API over a Session
// (docs/app.md, "The API"). It listens on one address; the default,
// 127.0.0.1, is reachable only from this machine (and from Windows, when
// it runs in WSL).
namespace pde2d::app {

struct ServerOptions {
  std::string host = "127.0.0.1";
  int port = 8765; // 0 picks a free port
  std::filesystem::path web_dir;
  // Example cases (*.json) offered by GET /api/examples; may be empty.
  std::filesystem::path examples_dir;
  // Larger request bodies are refused (413). A case carries its SVG or
  // mesh text, so this is generous.
  std::size_t max_body_bytes = 64 * 1024 * 1024;
  // Required on every /api/ request (the X-PDE2D-Token header or
  // ?token=) when set. A host that is not loopback always needs one: the
  // server makes a random one when this is empty.
  std::string token;
  // API requests that may wait for the session while one is served; one
  // more is answered 429.
  int max_waiting = 6;
  // How runs go (the server's always run in the background).
  SessionOptions session{.background_runs = true};
};

class Server {
public:
  // Throws std::invalid_argument when web_dir has no index.html.
  explicit Server(ServerOptions options);
  ~Server();
  Server(const Server &) = delete;
  Server &operator=(const Server &) = delete;

  // Binds the address; throws std::runtime_error when it cannot. Returns
  // the port (useful with port 0).
  int bind();
  // Serves until stop() is called from another thread.
  void listen();
  void stop();
  // The token /api/ requests need, or empty when they need none.
  const std::string &token() const;
  // Whether `host` is this machine only (127.0.0.1, localhost, ::1).
  static bool is_loopback(std::string_view host);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace pde2d::app
