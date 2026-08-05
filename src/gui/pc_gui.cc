#include "gui/runner.hpp"

#include <charconv>
#include <iostream>
#include <optional>
#include <spdlog/spdlog.h>
#include <string>
#include <string_view>
#include <utility>

namespace {

std::optional<int> parseLogLevel(std::string_view value) {
  int level = 0;
  const auto result =
      std::from_chars(value.data(), value.data() + value.size(), level);
  if (result.ec != std::errc{} || result.ptr != value.data() + value.size() ||
      level < 0 || level > 3) {
    return std::nullopt;
  }
  return level;
}

} // namespace

int main(int argc, char **argv) {
  kpt::gui::WorkbenchLaunchRequest request;
  int log_level = 2;
  for (int index = 1; index < argc; ++index) {
    const std::string_view argument(argv[index]);
    if (argument == "-h" || argument == "--help") {
      std::cout << "usage: kpt_gui [--smoke-test] "
                   "[-l|--log-level <0|1|2|3>]\n";
      return 0;
    }
    if (argument == "--smoke-test") {
      request.smoke_test = true;
      continue;
    }
    if (argument == "-l" || argument == "--log-level") {
      if (++index >= argc) {
        std::cerr << argument << " requires a value\n";
        return 1;
      }
      const auto parsed = parseLogLevel(argv[index]);
      if (!parsed) {
        std::cerr << "--log-level must be 0, 1, 2, or 3\n";
        return 1;
      }
      log_level = *parsed;
      continue;
    }
    constexpr std::string_view prefix = "--log-level=";
    if (argument.starts_with(prefix)) {
      const auto parsed = parseLogLevel(argument.substr(prefix.size()));
      if (!parsed) {
        std::cerr << "--log-level must be 0, 1, 2, or 3\n";
        return 1;
      }
      log_level = *parsed;
      continue;
    }
    std::cerr << "unknown option: " << argument << '\n';
    return 1;
  }
  constexpr spdlog::level::level_enum levels[] = {
      spdlog::level::err, spdlog::level::warn, spdlog::level::info,
      spdlog::level::debug};
  spdlog::set_level(levels[log_level]);
  return kpt::gui::runWorkbench(std::move(request));
}
