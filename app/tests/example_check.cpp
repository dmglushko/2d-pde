// app_check_<example>: compares an example's --run summary (summary.json,
// written by app_run_<example>) with the bounds in example_checks.json.
//
//   app_example_check CHECKS.json EXAMPLE SUMMARY.json
//
// Exits 0 when every bound holds; prints each figure with its bound.

#include <nlohmann/json.hpp>

#include <fstream>
#include <iostream>
#include <iterator>
#include <span>
#include <string>

namespace {

using Json = nlohmann::json;

Json read(const std::string &path) {
  std::ifstream in(path, std::ios::binary);
  if (!in)
    throw std::runtime_error("cannot read " + path);
  return Json::parse(std::string(std::istreambuf_iterator<char>(in),
                                 std::istreambuf_iterator<char>()));
}

// The value at a dotted path, or null.
const Json *at(const Json &root, const std::string &path) {
  const Json *node = &root;
  std::size_t start = 0;
  while (start <= path.size()) {
    const std::size_t dot = path.find('.', start);
    const std::string key = path.substr(start, dot - start);
    if (!node->is_object() || !node->contains(key))
      return nullptr;
    node = &(*node)[key];
    if (dot == std::string::npos)
      break;
    start = dot + 1;
  }
  return node;
}

} // namespace

int main(int argc, char **argv) {
  const std::span<char *> args(argv, static_cast<std::size_t>(argc));
  if (args.size() != 4) {
    std::cerr << "usage: app_example_check CHECKS.json EXAMPLE SUMMARY.json\n";
    return 2;
  }
  try {
    const Json checks = read(args[1]);
    const std::string example = args[2];
    const Json summary = read(args[3]);
    const Json &examples = checks.at("examples");
    if (!examples.contains(example)) {
      std::cerr << "FAIL " << example << " has no entry in " << args[1]
                << '\n';
      return 1;
    }
    bool ok = summary.value("ok", false);
    if (!ok)
      std::cout << "FAIL the run did not finish: " << summary.dump() << '\n';
    for (const auto &[path, bound] : examples[example].items()) {
      const Json *value = at(summary, path);
      std::string verdict = "PASS";
      if (value == nullptr)
        verdict = "FAIL (missing)";
      else if (bound.contains("equals") && *value != bound["equals"])
        verdict = "FAIL";
      else if ((bound.contains("min") || bound.contains("max")) &&
               !value->is_number())
        verdict = "FAIL (not a number)";
      else if ((bound.contains("min") &&
                value->get<double>() < bound["min"].get<double>()) ||
               (bound.contains("max") &&
                value->get<double>() > bound["max"].get<double>()))
        verdict = "FAIL";
      if (verdict != "PASS")
        ok = false;
      std::cout << verdict << ' ' << example << ' ' << path << " = "
                << (value != nullptr ? value->dump() : "none") << ", expected "
                << bound.dump() << '\n';
    }
    return ok ? 0 : 1;
  } catch (const std::exception &error) {
    std::cerr << "FAIL " << error.what() << '\n';
    return 1;
  }
}
