// This file is part of GenZ-LIO, released under the GNU GPL v2.
//
// Fixed-map pure localization entry point (ROS 2). Command line as Super-LIO's
// run_loclite_online: --config <GenZ-LIO YAML> [--map_path <dir>] [--ros-args ...].
#include "LocalizationNode.hpp"

#include <rclcpp/rclcpp.hpp>

#include <exception>
#include <iostream>
#include <memory>
#include <thread>

int main(int argc, char **argv) {
    try {
        auto cli = loclite::ParseCommandLine(argc, argv);
        if (cli.help) {
            std::cout << "Usage: ros2 run genz_lio run_loclite_online --config <GenZ-LIO YAML> "
                         "[--map_path <directory containing global.pcd>] [--ros-args ...]\n";
            return 0;
        }
        if (cli.config.empty())
            throw std::runtime_error("--config is required (use config/loclite_livox.yaml)");
        auto &args = cli.args;
        // The YAML first; ordinary ROS command-line overrides follow it.
        args.insert(args.begin() + 1, {"--ros-args", "-p", "config_path:=" + cli.config, "--"});
        if (!cli.map_path.empty())
            args.insert(args.end(), {"--ros-args", "-p", "system.map_path:=" + cli.map_path});
        std::vector<char *> raw;
        for (auto &arg : args) raw.push_back(arg.data());
        rclcpp::init(static_cast<int>(raw.size()), raw.data());

        auto node = std::make_shared<genz_lio::ros_wrapper::LocalizationNode>();
        // As in genz_lio_node: reception and scan processing on separate executors,
        // so a blocking cold-start alignment does not stall the input callbacks.
        rclcpp::executors::SingleThreadedExecutor input_executor;
        rclcpp::executors::SingleThreadedExecutor processing_executor;
        input_executor.add_node(node);
        processing_executor.add_callback_group(node->processingGroup(), node->get_node_base_interface());

        std::exception_ptr processing_error;
        std::thread processing_thread([&] {
            try {
                processing_executor.spin();
            } catch (...) {
                processing_error = std::current_exception();
                rclcpp::shutdown();
            }
        });
        try {
            input_executor.spin();
        } catch (...) {
            rclcpp::shutdown();
            processing_thread.join();
            throw;
        }
        processing_executor.cancel();
        processing_thread.join();
        if (processing_error) std::rethrow_exception(processing_error);
        rclcpp::shutdown();
        return 0;
    } catch (const std::exception &error) {
        std::cerr << "genz_lio localization: " << error.what() << '\n';
        if (rclcpp::ok()) rclcpp::shutdown();
        return 1;
    }
}
