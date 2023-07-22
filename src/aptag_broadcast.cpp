//
// Created by ola on 7/19/23.
//

#include <memory>
#include <string>
#include <map>
#include <vector>
//
#include "rclcpp/rclcpp.hpp"
//
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.h>

#include <yaml-cpp/yaml.h>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <Eigen/Dense>
#include <filesystem>

class AptagFramePublisher : public rclcpp::Node {
public:
    AptagFramePublisher(const std::string id, Eigen::Matrix4d transformation_matrix, const std::string frame_id)
            : Node("aptag_frame_publisher"+ id) {
//          Initialize the transform broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        clock_ = rclcpp::Clock(rcl_clock_type_e::RCL_ROS_TIME);
        auto func = [this]() -> void { timer_callback(); };
        timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), func);
        id_ = id;
        frame_id_ = frame_id;
        transformation_matrix_ = transformation_matrix;

    }

    void timer_callback() {
        Eigen::Affine3d affine(transformation_matrix_);

        Eigen::Quaterniond quaternion(affine.linear());
        Eigen::Vector3d translation(affine.translation());

        geometry_msgs::msg::TransformStamped t;

        // Fill in the message
        t.header.frame_id = frame_id_;
        t.child_frame_id =  id_;

        // Turtle only exists in 2D, thus we get x and y translation
        // coordinates from the message and set the z coordinate to 0
        t.transform.translation.x = translation.x();
        t.transform.translation.y = translation.y() ;
        t.transform.translation.z = translation.z() ;

        // For the same reason, turtle can only rotate around one axis
        // and this why we set rotation in x and y to 0 and obtain
        // rotation in z axis from the message
        t.transform.rotation.x = quaternion.x();
        t.transform.rotation.y = quaternion.y();
        t.transform.rotation.z = quaternion.z();
        t.transform.rotation.w = quaternion.w();

        t.header.stamp = clock_.now();
        // Send the transformation
        tf_broadcaster_->sendTransform(t);
        // Print the translation and rotation components separately
        std::cout << "Translation: " << t.transform.translation.x << ", " << t.transform.translation.y << ", " << t.transform.translation.z << std::endl;
        std::cout << "Rotation: " << t.transform.rotation.x << ", " << t.transform.rotation.y << ", " << t.transform.rotation.z << ", " << t.transform.rotation.w << std::endl;
    }

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Clock clock_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string id_;
    std::string frame_id_;
    Eigen::Matrix4d transformation_matrix_;
};

int main(int argc, char **argv) {

    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exe;
    std::vector<std::shared_ptr<AptagFramePublisher>> nodes;

    std::filesystem::path pkg_dir = ament_index_cpp::get_package_share_directory("aptags_tf_broadcast");
    auto file_path = pkg_dir / "config" / "transformation_matrix.yaml";

    // Load YAML file
    YAML::Node yaml_data = YAML::LoadFile(file_path);

    // Extract transformations from YAML data
    YAML::Node transformations = yaml_data["transformations"];
    for (const auto& transformation : transformations) {
        // Extract ID and matrix
        const std::string matrix_id = transformation["id"].as<std::string>();
        const std::string frame_id = transformation["frame_id"].as<std::string>();

        Eigen::Matrix4d matrix;
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                matrix(i, j) = transformation["matrix"][i][j].as<double>();
            }
        }
        std::cout << matrix << std::endl;

        auto node = std::make_shared<AptagFramePublisher>(matrix_id, matrix, frame_id);
        exe.add_node(node);
        nodes.push_back(node);
    }

    exe.spin();

    rclcpp::shutdown();

    return 0;
}