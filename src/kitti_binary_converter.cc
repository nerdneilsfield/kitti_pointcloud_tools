/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   kitti_binary_converter.cc                          :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: dengqi <dengqi935@outl>                    +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2020/09/07 11:15:45 by dengqi            #+#    #+#             */
/*   Updated: 2020/09/07 16:03:29 by dengqi           ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include <spdlog/spdlog.h>

#include <popl.hpp>
#include <sstream>
#include <string>

#include "kitti_binary.h"

using namespace kitti_binary_tools;

int main(int argc, char* argv[]) {
  spdlog::debug("call init for kitti binary_converted");

  int log_level = 2;
  std::string input_directory;
  std::string output_directory;
  std::string prefix_str;
  std::string output_type;

  int start_index = -1;
  int end_index = -1;
  int width = 0;

  // define the popl ops
  popl::OptionParser op("Allowed options");
  auto help_option = op.add<popl::Switch>("h", "help", "produce help message");

  const std::string convert_type_help =
      "pcd :: output as pcd file \n"
      "ply: output as ply file";
  auto convert_type_option =
      op.add<popl::Value<std::string>>("t", "type", convert_type_help, "map");
  auto log_level_option = op.add<popl::Value<int>>(
      "l", "log_level",
      "log level:\n 0 for error, \n 1 for warning,\n 2 for "
      "info, \n 3 for debug, \n 4 for silent",
      2);
  auto input_directory_option = op.add<popl::Value<std::string>>(
      "i", "in", "input directory of kitti binary", "test");
  auto output_directory_option = op.add<popl::Value<std::string>>(
      "o", "out", "input directory of kitti binary", "test");
  auto prefix_option = op.add<popl::Value<std::string>>(
      "p", "prefix", "the prefix of input files(only work in sequences");
  auto start_index_option = op.add<popl::Value<int>>(
      "s", "start", "start index of sequence(include)", -1);
  auto end_index_option = op.add<popl::Value<int>>(
      "e", "end", "end index of sequence( not include)", -1);
  auto width_option = op.add<popl::Value<int>>(
      "w", "width", "the width of sequence data: 000111 is 6", 10);

  // assign the args
  convert_type_option->assign_to(&output_type);
  log_level_option->assign_to(&log_level);
  input_directory_option->assign_to(&input_directory);
  output_directory_option->assign_to(&output_directory);
  prefix_option->assign_to(&prefix_str);
  start_index_option->assign_to(&start_index);
  end_index_option->assign_to(&end_index);
  width_option->assign_to(&width);
  // parse the args
  op.parse(argc, argv);
  // dispaly the help options
  if (help_option->is_set()) {
    std::cout << op << std::endl;
    return 0;
  }

  // set the log level
  switch (log_level) {
    case 0:
      spdlog::set_level(spdlog::level::err);
      break;
    case 1:
      spdlog::set_level(spdlog::level::warn);
      break;
    case 2:
      spdlog::set_level(spdlog::level::info);
      break;
    case 3:
      spdlog::set_level(spdlog::level::debug);
      break;
    default:
      spdlog::set_level(spdlog::level::info);
      break;
  }

  if (input_directory[input_directory.length() - 1] != '/') {
    input_directory = input_directory + "/";
  }

  if (output_directory[output_directory.length() - 1] != '/') {
    output_directory = output_directory + "/";
  }

  spdlog::debug(
      "[kiiti_binary_converter] args:\n\tconvert_type: {},\n\tlog_level:"
      "{},\n\tinput_directory: {}",
      output_type, log_level, input_directory);
  spdlog::debug(
      "[kiiti_binary_converter] args:\n\toutput_directory:{},\n\tprefix_str:{}",
      output_directory, prefix_str);
  spdlog::debug(
      "[kiiti_binary_converter] args:\n\tstart_index:{},\n\tend_index:{} "
      "\n\twidth:{}",
      start_index, end_index, width);

  std::string input_file_name;
  std::string output_file_name;
  if (output_type == "pcd") {
    for (int i = start_index; i < end_index; i++) {
      std::stringstream ss;
      ss << prefix_str << std::setw(width) << std::setfill('0') << i;
      input_file_name = input_directory + ss.str() + ".bin";
      output_file_name = output_directory + ss.str() + ".pcd";
      KittiBinary kitti_binary(input_file_name);
      kitti_binary.SaveAsPCD(output_file_name);
      ss.clear();
    }
  } else if (output_type == "ply") {
    for (int i = start_index; i < end_index; i++) {
      std::stringstream ss;
      ss << prefix_str << std::setw(width) << std::setfill('0') << i;
      input_file_name = input_file_name + ss.str() + ".bin";
      output_file_name = output_file_name + ss.str() + ".ply";
      KittiBinary kitti_binary(input_file_name);
      kitti_binary.SaveAsPLY(output_file_name);
      ss.clear();
    }
  }
}
