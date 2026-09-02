#include "ppbng_hsi/hsi_production_node_factory.hpp"

#include <array>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

namespace
{

struct CameraArguments
{
  std::string ros_namespace;
  std::string node_name;
  std::vector<std::string> parameters;
  std::vector<std::string> remaps;
};

struct ProcessArguments
{
  CameraArguments fx10e;
  CameraArguments swir;
  std::vector<std::string> rcl_arguments;
};

std::string consume_value(int & index, const int argc, char ** argv, const char * option)
{
  if (++index >= argc) {
    throw std::invalid_argument(std::string(option) + " requires a value");
  }
  return argv[index];
}

ProcessArguments parse_arguments(const int argc, char ** argv)
{
  ProcessArguments result{{"/fx10e", "camera", {}, {}}, {"/swir", "camera", {}, {}}, {argv[0]}};
  for (int index = 1; index < argc; ++index) {
    const std::string option = argv[index];
    if (option == "--fx10e-namespace") {
      result.fx10e.ros_namespace = consume_value(index, argc, argv, option.c_str());
    } else if (option == "--swir-namespace") {
      result.swir.ros_namespace = consume_value(index, argc, argv, option.c_str());
    } else if (option == "--fx10e-name") {
      result.fx10e.node_name = consume_value(index, argc, argv, option.c_str());
    } else if (option == "--swir-name") {
      result.swir.node_name = consume_value(index, argc, argv, option.c_str());
    } else if (option == "--fx10e-param") {
      result.fx10e.parameters.push_back(consume_value(index, argc, argv, option.c_str()));
    } else if (option == "--swir-param") {
      result.swir.parameters.push_back(consume_value(index, argc, argv, option.c_str()));
    } else if (option == "--fx10e-remap") {
      result.fx10e.remaps.push_back(consume_value(index, argc, argv, option.c_str()));
    } else if (option == "--swir-remap") {
      result.swir.remaps.push_back(consume_value(index, argc, argv, option.c_str()));
    } else {
      result.rcl_arguments.push_back(option);
    }
  }
  if (result.fx10e.parameters.empty() || result.swir.parameters.empty()) {
    throw std::invalid_argument("both cameras require explicit launch-provided parameters");
  }
  return result;
}

rclcpp::NodeOptions node_options(const CameraArguments & camera)
{
  std::vector<std::string> arguments{"--ros-args", "-r", "__ns:=" + camera.ros_namespace,
    "-r", "__node:=" + camera.node_name};
  for (const auto & parameter : camera.parameters) {
    arguments.insert(arguments.end(), {"-p", parameter});
  }
  for (const auto & remap : camera.remaps) {
    arguments.insert(arguments.end(), {"-r", remap});
  }
  return rclcpp::NodeOptions().use_global_arguments(false).arguments(arguments);
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    const auto process = parse_arguments(argc, argv);
    std::vector<char *> rcl_argv;
    rcl_argv.reserve(process.rcl_arguments.size());
    for (const auto & argument : process.rcl_arguments) {
      rcl_argv.push_back(const_cast<char *>(argument.c_str()));
    }
    int rcl_argc = static_cast<int>(rcl_argv.size());
    rclcpp::init(rcl_argc, rcl_argv.data());

    auto fx10e = ppbng_hsi::make_hsi_production_node(node_options(process.fx10e));
    auto swir = ppbng_hsi::make_hsi_production_node(node_options(process.swir));
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2U);
    executor.add_node(fx10e);
    executor.add_node(swir);
    executor.spin();
    executor.remove_node(swir);
    executor.remove_node(fx10e);
    swir.reset();
    fx10e.reset();
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "hsi_dual_production_node: " << error.what() << '\n';
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return 2;
  }
}
