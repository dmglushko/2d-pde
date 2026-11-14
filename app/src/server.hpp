#pragma once

#include "session.hpp"

#include <filesystem>
#include <cstddef>
#include <functional>
#include <memory>
#include <string>

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

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace pde2d::app
