#include "gui/runner.hpp"

#include <string>
#include <utility>

int main(int argc, char **argv) {
  kpt::gui::WorkbenchLaunchRequest request;
  request.smoke_test = argc > 1 && std::string(argv[1]) == "--smoke-test";
  return kpt::gui::runWorkbench(std::move(request));
}
