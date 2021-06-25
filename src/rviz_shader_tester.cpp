#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <chrono>

using namespace std::chrono_literals;

inline
geometry_msgs::msg::Point Point(double x, double y, double z)
{
  geometry_msgs::msg::Point msg;
  msg.x = x;
  msg.y = y;
  msg.z = z;
  return msg;
}

class Publisher : public rclcpp::Node
{
public:
  Publisher() : Node("shader_tester_publisher")
  {
    publisher_ = 
        this->create_publisher<visualization_msgs::msg::Marker>("shader_tester", 1000);

    create_marker();

    timer_ = this->create_wall_timer(
        10ms, std::bind(&Publisher::timer_callback, this));
  }

private:

  void timer_callback()
  {
    auto cur_time = this->now();
    publisher_->publish(face_msg_);
  }

  void create_marker()
  {
    face_msg_.header.frame_id = "world";
    face_msg_.ns = "tri_faces";
    face_msg_.id = 0;
    face_msg_.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
    face_msg_.action = visualization_msgs::msg::Marker::ADD;

    face_msg_.lifetime = rclcpp::Duration::from_nanoseconds(0);
    face_msg_.frame_locked = true;

    face_msg_.scale.x = 1.0;
    face_msg_.scale.y = 1.0;
    face_msg_.scale.z = 1.0;

    face_msg_.points.clear();
    face_msg_.points.resize(3);
    face_msg_.colors.clear();
    face_msg_.colors.resize(3);
    face_msg_.uv_coordinates.clear();
    face_msg_.uv_coordinates.resize(3);

    face_msg_.points.at(0) = Point(5, 0, 0);
    face_msg_.points.at(1) = Point(-5, -5, 0);
    face_msg_.points.at(2) = Point(-5, 5, 0);

    // Color the entire triangle white.
    face_msg_.colors.at(0).r = 1.0;
    face_msg_.colors.at(0).g = 1.0;
    face_msg_.colors.at(0).b = 1.0;
    face_msg_.colors.at(0).a = 1.0;

    face_msg_.colors.at(1).r = 1.0;
    face_msg_.colors.at(1).g = 1.0;
    face_msg_.colors.at(1).b = 1.0;
    face_msg_.colors.at(1).a = 1.0;

    face_msg_.colors.at(2).r = 1.0;
    face_msg_.colors.at(2).g = 1.0;
    face_msg_.colors.at(2).b = 1.0;
    face_msg_.colors.at(2).a = 1.0;

    // Define the texture coordinates for the triangle.
    face_msg_.uv_coordinates.at(0).u = 1.0;
    face_msg_.uv_coordinates.at(0).v = 0.0;

    face_msg_.uv_coordinates.at(1).u = 0.0;
    face_msg_.uv_coordinates.at(1).v = 0.0;

    face_msg_.uv_coordinates.at(2).u = 0.0;
    face_msg_.uv_coordinates.at(2).v = 0.0;

    face_msg_.texture_map = "/tmp/texture.png";
  }

  visualization_msgs::msg::Marker face_msg_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
  // Initialize ros systems.
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("rviz_shader_tester_node");
  rclcpp::spin(std::make_shared<Publisher>());
  rclcpp::shutdown();
  return 0;
}
