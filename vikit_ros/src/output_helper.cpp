// Copyright (c) 2023 VulcanYJX
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <https://www.gnu.org/licenses/>.

#include <vikit/output_helper.h>
// #include <visualization_msgs/Marker.h>

namespace vk {
namespace output_helper {

void 
publishTfTransform(const Sophus::SE3& T, const rclcpp::Time& stamp,
                        const std::string& frame_id, const std::string& child_frame_id,
                        std::shared_ptr<tf2_ros::TransformBroadcaster> br)
{
    // Create a TransformStamped message
    geometry_msgs::msg::TransformStamped transform_stamped;
    // Set the header with the frame_id and timestamp
    transform_stamped.header.stamp = stamp;
    transform_stamped.header.frame_id = frame_id;
    // Set the child frame ID
    transform_stamped.child_frame_id = child_frame_id;
    // Set the translation
    transform_stamped.transform.translation.x = T.translation().x();
    transform_stamped.transform.translation.y = T.translation().y();
    transform_stamped.transform.translation.z = T.translation().z();
    // Convert the Sophus quaternion (Eigen::Quaterniond) to a geometry_msgs quaternion
    Eigen::Quaterniond q(T.rotation_matrix());
    transform_stamped.transform.rotation.x = q.x();
    transform_stamped.transform.rotation.y = q.y();
    transform_stamped.transform.rotation.z = q.z();
    transform_stamped.transform.rotation.w = q.w();
    // Broadcast the transform
    br->sendTransform(transform_stamped);
}

void 
publishPointMarker(
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub,
    const Vector3d &pos,
    const string &ns,
    const rclcpp::Time &timestamp,
    int id,
    int action,
    double marker_scale,
    const Vector3d &color,
    rclcpp::Duration lifetime)
{
    // Create a Marker message
    auto msg = visualization_msgs::msg::Marker();
    msg.header.frame_id = "world"; // Frame of reference for the marker
    msg.header.stamp = timestamp; // Timestamp for the marker
    msg.ns = ns;                  // Namespace for grouping markers
    msg.id = id;                  // Unique ID for this marker
    msg.type = visualization_msgs::msg::Marker::CUBE; // Shape: CUBE
    msg.action = action;          // Action: Add, modify, or delete the marker
    // Set marker scale
    msg.scale.x = marker_scale; // X dimension
    msg.scale.y = marker_scale; // Y dimension
    msg.scale.z = marker_scale; // Z dimension
    // Set marker color
    msg.color.a = 1.0;          // Alpha (opacity)
    msg.color.r = color[0];     // Red
    msg.color.g = color[1];     // Green
    msg.color.b = color[2];     // Blue
    // Set marker lifetime
    msg.lifetime = lifetime;    // Duration before the marker disappears
    // Set marker position
    msg.pose.position.x = pos[0]; // X position
    msg.pose.position.y = pos[1]; // Y position
    msg.pose.position.z = pos[2]; // Z position
    // Publish the message
    pub->publish(msg);
}

void 
publishLineMarker(
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub,
    const Vector3d &start,
    const Vector3d &end,
    const string &ns,
    const rclcpp::Time &timestamp,
    int id,
    int action,
    double marker_scale,
    const Vector3d &color,
    rclcpp::Duration lifetime)
{
    // Create a Marker message
    auto msg = visualization_msgs::msg::Marker();
    msg.header.frame_id = "world"; // Frame of reference for the marker
    msg.header.stamp = timestamp; // Timestamp for the marker
    msg.ns = ns;                  // Namespace for grouping markers
    msg.id = id;                  // Unique ID for this marker
    msg.type = visualization_msgs::msg::Marker::LINE_STRIP; // Shape: LINE_STRIP
    msg.action = action;          // Action: Add, modify, or delete the marker
    // Set marker scale (applies to line width)
    msg.scale.x = marker_scale;
    // Set marker color
    msg.color.a = 1.0;          // Alpha (opacity)
    msg.color.r = color[0];     // Red
    msg.color.g = color[1];     // Green
    msg.color.b = color[2];     // Blue
    // Set marker lifetime
    msg.lifetime = lifetime;    // Duration before the marker disappears
    // Set points for the line
    msg.points.resize(2);
    msg.points[0].x = start[0]; // Start point X
    msg.points[0].y = start[1]; // Start point Y
    msg.points[0].z = start[2]; // Start point Z
    msg.points[1].x = end[0];   // End point X
    msg.points[1].y = end[1];   // End point Y
    msg.points[1].z = end[2];   // End point Z
    // Publish the message
    pub->publish(msg);
}

void 
publishArrowMarker(
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub,
    const Vector3d &pos,
    const Vector3d &dir,
    double scale,
    const string &ns,
    const rclcpp::Time &timestamp,
    int id,
    int action,
    double marker_scale,
    const Vector3d &color)
{
    // Create a Marker message
    auto msg = visualization_msgs::msg::Marker();
    msg.header.frame_id = "world"; // Frame of reference for the marker
    msg.header.stamp = timestamp; // Timestamp for the marker
    msg.ns = ns;                  // Namespace for grouping markers
    msg.id = id;                  // Unique ID for this marker
    msg.type = visualization_msgs::msg::Marker::ARROW; // Marker type: ARROW
    msg.action = action;          // Action: Add, modify, or delete the marker
    // Set marker scale
    msg.scale.x = marker_scale;         // Shaft diameter
    msg.scale.y = marker_scale * 0.35; // Head diameter
    msg.scale.z = 0.0;                  // Head length (not used for ARROW type)
    // Set marker color
    msg.color.a = 1.0;          // Alpha (opacity)
    msg.color.r = color[0];     // Red
    msg.color.g = color[1];     // Green
    msg.color.b = color[2];     // Blue
    // Define the start and end points of the arrow
    msg.points.resize(2);       // ARROW type requires exactly two points
    msg.points[0].x = pos[0];   // Start point X
    msg.points[0].y = pos[1];   // Start point Y
    msg.points[0].z = pos[2];   // Start point Z
    msg.points[1].x = pos[0] + scale * dir[0]; // End point X (based on direction and scale)
    msg.points[1].y = pos[1] + scale * dir[1]; // End point Y
    msg.points[1].z = pos[2] + scale * dir[2]; // End point Z
    // Publish the Arrow Marker
    pub->publish(msg);
}

void 
publishHexacopterMarker(
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub,
    const string &frame_id,
    const string &ns,
    const rclcpp::Time &timestamp,
    int id,
    int action,
    double marker_scale,
    const Vector3d &color)
{
    const double sqrt2_2 = std::sqrt(2) / 2;

    visualization_msgs::msg::Marker marker;

    // 设置 marker 的基本信息
    marker.header.frame_id = frame_id;
    marker.header.stamp = timestamp;
    marker.ns = ns;
    marker.action = action; // 设置 marker 的动作
    marker.id = id;

    // 设置 rotor 的形状和颜色
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.scale.x = 0.2 * marker_scale;
    marker.scale.y = 0.2 * marker_scale;
    marker.scale.z = 0.01 * marker_scale;
    marker.color.r = 0.4;
    marker.color.g = 0.4;
    marker.color.b = 0.4;
    marker.color.a = 0.8;
    marker.pose.position.z = 0;

    // 前左/右 rotor
    marker.pose.position.x = 0.19 * marker_scale;
    marker.pose.position.y = 0.11 * marker_scale;
    marker.id--;
    pub->publish(marker);

    marker.pose.position.x = 0.19 * marker_scale;
    marker.pose.position.y = -0.11 * marker_scale;
    marker.id--;
    pub->publish(marker);

    // 左/右 rotor
    marker.pose.position.x = 0;
    marker.pose.position.y = 0.22 * marker_scale;
    marker.id--;
    pub->publish(marker);

    marker.pose.position.x = 0;
    marker.pose.position.y = -0.22 * marker_scale;
    marker.id--;
    pub->publish(marker);

    // 后左/右 rotor
    marker.pose.position.x = -0.19 * marker_scale;
    marker.pose.position.y = 0.11 * marker_scale;
    marker.id--;
    pub->publish(marker);

    marker.pose.position.x = -0.19 * marker_scale;
    marker.pose.position.y = -0.11 * marker_scale;
    marker.id--;
    pub->publish(marker);

    // 设置臂的形状
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.scale.x = 0.44 * marker_scale;
    marker.scale.y = 0.02 * marker_scale;
    marker.scale.z = 0.01 * marker_scale;
    marker.color.r = color[0];
    marker.color.g = color[1];
    marker.color.b = color[2];
    marker.color.a = 1;

    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = -0.015 * marker_scale;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;

    // 设置臂的方向和旋转（45度）
    marker.pose.orientation.w = sqrt2_2;
    marker.pose.orientation.z = sqrt2_2;
    marker.id--;
    pub->publish(marker);

    // 设置臂的方向（30度旋转）
    marker.pose.orientation.w = 0.9659;
    marker.pose.orientation.z = 0.2588;
    marker.id--;
    pub->publish(marker);

    marker.pose.orientation.w = 0.9659;
    marker.pose.orientation.z = -0.2588;
    marker.id--;
    pub->publish(marker);
}

void 
publishCameraMarker(
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr& pub,
    const std::string& frame_id,
    const std::string& ns,
    const rclcpp::Time& timestamp,
    int id,
    double marker_scale,
    const Vector3d& color)
{
    /*
     * Draw a pyramid as the camera marker
     */
    const double sqrt2_2 = std::sqrt(2) / 2;

    visualization_msgs::msg::Marker marker;

    // The marker will be displayed in frame_id
    marker.header.frame_id = frame_id;
    marker.header.stamp = timestamp;
    marker.ns = ns;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.id = id;

    // Make rectangles as frame
    double r_w = 1.0;
    double z_plane = (r_w / 2.0) * marker_scale;
    marker.pose.position.x = 0;
    marker.pose.position.y = (r_w / 4.0) * marker_scale;
    marker.pose.position.z = z_plane;

    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.scale.x = r_w * marker_scale;
    marker.scale.y = 0.04 * marker_scale;
    marker.scale.z = 0.04 * marker_scale;
    marker.color.r = color[0];
    marker.color.g = color[1];
    marker.color.b = color[2];
    marker.color.a = 1.0;

    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;

    // Publish the first rectangle
    marker.id--;
    pub->publish(marker);

    // Publish the second rectangle
    marker.pose.position.y = -(r_w / 4.0) * marker_scale;
    marker.id--;
    pub->publish(marker);

    // Publish the third rectangle
    marker.scale.x = (r_w / 2.0) * marker_scale;
    marker.pose.position.x = (r_w / 2.0) * marker_scale;
    marker.pose.position.y = 0;
    marker.pose.orientation.w = sqrt2_2;
    marker.pose.orientation.z = sqrt2_2;
    marker.id--;
    pub->publish(marker);

    // Publish the fourth rectangle
    marker.pose.position.x = -(r_w / 2.0) * marker_scale;
    marker.id--;
    pub->publish(marker);

    // Make pyramid edges
    marker.scale.x = (3.0 * r_w / 4.0) * marker_scale;
    marker.pose.position.z = 0.5 * z_plane;

    marker.pose.position.x = (r_w / 4.0) * marker_scale;
    marker.pose.position.y = (r_w / 8.0) * marker_scale;
    marker.pose.orientation.x = 0.08198092;
    marker.pose.orientation.y = -0.34727674;
    marker.pose.orientation.z = 0.21462883;
    marker.pose.orientation.w = 0.9091823;
    marker.id--;
    pub->publish(marker);

    marker.pose.position.x = -(r_w / 4.0) * marker_scale;
    marker.pose.position.y = (r_w / 8.0) * marker_scale;
    marker.pose.orientation.x = 0.08198092;
    marker.pose.orientation.y = 0.34727674;
    marker.pose.orientation.z = -0.21462883;
    marker.pose.orientation.w = 0.9091823;
    marker.id--;
    pub->publish(marker);

    marker.pose.position.x = -(r_w / 4.0) * marker_scale;
    marker.pose.position.y = -(r_w / 8.0) * marker_scale;
    marker.pose.orientation.x = -0.08198092;
    marker.pose.orientation.y = 0.34727674;
    marker.pose.orientation.z = 0.21462883;
    marker.pose.orientation.w = 0.9091823;
    marker.id--;
    pub->publish(marker);

    marker.pose.position.x = (r_w / 4.0) * marker_scale;
    marker.pose.position.y = -(r_w / 8.0) * marker_scale;
    marker.pose.orientation.x = -0.08198092;
    marker.pose.orientation.y = -0.34727674;
    marker.pose.orientation.z = -0.21462883;
    marker.pose.orientation.w = 0.9091823;
    marker.id--;
    pub->publish(marker);
}

void 
publishFrameMarker(
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr& pub,
    const Matrix3d& rot,
    const Vector3d& pos,
    const std::string& ns,
    const rclcpp::Time& timestamp,
    int id,
    int action,
    double marker_scale,
    rclcpp::Duration lifetime)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "/world";  // Frame ID
    marker.header.stamp = timestamp;   // Timestamp
    marker.ns = ns;                    // Namespace for the marker
    marker.type = visualization_msgs::msg::Marker::ARROW;  // Marker type
    marker.action = action;            // Action (e.g., ADD or MODIFY)
    marker.scale.x = 0.5 * marker_scale;  // Shaft diameter
    marker.scale.y = 0.5 * marker_scale;  // Head diameter
    marker.color.a = 1.0;              // Alpha value (opacity)

    geometry_msgs::msg::Point point;

    // Draw Z-axis arrow (blue)
    marker.id = id++;
    marker.points.clear();
    point.x = static_cast<float>(pos.x());
    point.y = static_cast<float>(pos.y());
    point.z = static_cast<float>(pos.z());
    marker.points.push_back(point);
    point.x = static_cast<float>(pos.x() + marker_scale * rot(0, 2));
    point.y = static_cast<float>(pos.y() + marker_scale * rot(1, 2));
    point.z = static_cast<float>(pos.z() + marker_scale * rot(2, 2));
    marker.points.push_back(point);
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.lifetime = lifetime;  // Marker lifetime
    pub->publish(marker);

    // Draw X-axis arrow (red)
    marker.id = id++;
    marker.points.clear();
    point.x = static_cast<float>(pos.x());
    point.y = static_cast<float>(pos.y());
    point.z = static_cast<float>(pos.z());
    marker.points.push_back(point);
    point.x = static_cast<float>(pos.x() + marker_scale * rot(0, 0));
    point.y = static_cast<float>(pos.y() + marker_scale * rot(1, 0));
    point.z = static_cast<float>(pos.z() + marker_scale * rot(2, 0));
    marker.points.push_back(point);
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.lifetime = lifetime;
    pub->publish(marker);

    // Draw Y-axis arrow (green)
    marker.id = id++;
    marker.points.clear();
    point.x = static_cast<float>(pos.x());
    point.y = static_cast<float>(pos.y());
    point.z = static_cast<float>(pos.z());
    marker.points.push_back(point);
    point.x = static_cast<float>(pos.x() + marker_scale * rot(0, 1));
    point.y = static_cast<float>(pos.y() + marker_scale * rot(1, 1));
    point.z = static_cast<float>(pos.z() + marker_scale * rot(2, 1));
    marker.points.push_back(point);
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.lifetime = lifetime;
    pub->publish(marker);
}

} // namespace output_helper
} // namespace vk


