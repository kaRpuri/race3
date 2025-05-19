#include <memory>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "nanoflann.hpp"
#include "Eigen/Dense"
#include "Eigen/StdVector"  

class PurePursuit : public rclcpp::Node
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  

    PurePursuit() : Node("pure_pursuit_node"), 
                    sim(true), 
                    L(2.5), // 1.72 for simulation, 2.7 on car considering slip
                    P(0.435), 
                    kd_tree_adaptor(waypoints)
    {
        // Load waypoints FIRST before anything else
        loadWaypoints();
        
        // Subscribers
        if (sim) {
            odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
                "/ego_racecar/odom", 10,
                std::bind(&PurePursuit::odom_callback, this, std::placeholders::_1));
        } else {
            pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/pf/viz/inferred_pose", 10,
                std::bind(&PurePursuit::pose_callback, this, std::placeholders::_1));
        }

        laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&PurePursuit::lidar_callback, this, std::placeholders::_1));

        drive_pub = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
        marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/waypoints_markers", 10);

        // Build KDTree only after waypoints are loaded
        if (!waypoints.empty()) {
            kd_tree = std::make_unique<KDTree>(2, kd_tree_adaptor, nanoflann::KDTreeSingleIndexAdaptorParams(10));
            kd_tree->buildIndex();
            RCLCPP_INFO(this->get_logger(), "KDTree built with %ld waypoints", waypoints.size());
        } else {
            RCLCPP_ERROR(this->get_logger(), "No waypoints loaded! KDTree not built.");
        }

        // Timer for marker publishing
        timer = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&PurePursuit::publish_waypoints_markers, this));
    }

private:
    // ROS interfaces
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub;
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub;

    // Parameters
    bool sim;
    double L, P;

    // use the alligned allocator for waypoint and KD tree
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> waypoints;
    
    struct KDTreeAdaptor {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // Alignment macro
        const std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>& data;
        
        KDTreeAdaptor(const std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>& data) : data(data) {}
        
        inline size_t kdtree_get_point_count() const { return data.size(); }
        inline double kdtree_get_pt(const size_t idx, const size_t dim) const { return data[idx][dim]; }
        template <class BBOX> bool kdtree_get_bbox(BBOX&) const { return false; }
    };
    
    KDTreeAdaptor kd_tree_adaptor;
    
    using KDTree = nanoflann::KDTreeSingleIndexAdaptor<
        nanoflann::L2_Simple_Adaptor<double, KDTreeAdaptor>,
        KDTreeAdaptor, 2, unsigned int>;
        
    std::unique_ptr<KDTree> kd_tree;
    Eigen::Matrix<double, 2,3> candidate_goals;
    double car_x, car_y;
    double car_yaw;
    Eigen::Vector2d car_pos;


  

    // Rotation matrix
    Eigen::Matrix3d rot;

    // Member Functions
    void loadWaypoints();
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void process_pose(const geometry_msgs::msg::Quaternion& quat);
    Eigen::Vector2d translate_point(const Eigen::Vector2d& curr, const Eigen::Vector2d& target);
    void publish_waypoints_markers();
    void publish_candidate_markers(const Eigen::Matrix<double, 2,3> candidate_goals);
    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
};

void PurePursuit::loadWaypoints() {
    std::string csv_path = "/home/kabirpuri/f1ws/src/race3/src/final_points.csv";
    std::ifstream file(csv_path);
    
    if (!file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open waypoints file: %s", csv_path.c_str());
        return;
    }
    
    RCLCPP_INFO(this->get_logger(), "Successfully opened waypoints file: %s", csv_path.c_str());
    
    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string x_str, y_str;
        std::getline(ss, x_str, ',');
        std::getline(ss, y_str, ',');
        
        try {
            double x = std::stod(x_str);
            double y = std::stod(y_str);
            waypoints.emplace_back(x, y);
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Failed to parse waypoint: %s", line.c_str());
        }
    }
    
    RCLCPP_INFO(this->get_logger(), "Loaded %ld waypoints", waypoints.size());
}




void PurePursuit::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    try {
        car_x = msg->pose.pose.position.x;
        car_y = msg->pose.pose.position.y;
        car_pos = Eigen::Vector2d(car_x, car_y);
        process_pose(msg->pose.pose.orientation);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error in odom_callback: %s", e.what()); // test odom callback
    }
}

void PurePursuit::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    try { // test the pose callback
        car_x = msg->pose.position.x;
        car_y = msg->pose.position.y;
        car_pos = Eigen::Vector2d(car_x, car_y);
        process_pose(msg->pose.orientation);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error in pose_callback: %s", e.what());
    }
}




void PurePursuit::process_pose(const geometry_msgs::msg::Quaternion& quat) {
    if (waypoints.empty() || !kd_tree) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                              "Waypoints or KDTree not initialized");
        return;
    }

    // Quaternion to rotation matrix
    Eigen::Quaterniond q(quat.w, quat.x, quat.y, quat.z);
    rot = q.normalized().toRotationMatrix().transpose();


    car_yaw = atan2(rot(1, 0), rot(0, 0));
    
    // Find closest waypoint
    std::vector<double> query{car_x, car_y};
    unsigned int idx;
    double dist;
    
    try {
        kd_tree->knnSearch(query.data(), 1, &idx, &dist);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "KDTree search error: %s", e.what());
        return;
    }

    // Vectorized lookahead search improved for faster computation
    
    size_t goal_idx = waypoints.size();
    for (size_t i = idx; i < waypoints.size(); ++i) {
        if ((waypoints[i] - car_pos).norm() >= L) {
            goal_idx = i;
            break;
        }
    }
    if (goal_idx == waypoints.size()) {
        for (size_t i = 0; i < idx; ++i) {
            if ((waypoints[i] - car_pos).norm() >= L) {
                goal_idx = i;
                break;
            }
        }
        if (goal_idx == waypoints.size()) return;
    }
    Eigen::Vector2d goal = waypoints[goal_idx];
    Eigen::Vector2d goalmo = waypoints[(goal_idx - 1)];

    double slope = (goal.y() - goalmo.y()) / (goal.x() - goalmo.x());
    double perpendicular_slope = -1 / slope;

    Eigen::Vector2d dir(1.0, perpendicular_slope);
    dir.normalize();
    
    idx = 0;
    for (double d = -0.6; d<0.6+1e-6; d+=0.6) {
        candidate_goals.col(idx) = goal + d * dir;
        idx++;
    }
    publish_candidate_markers(candidate_goals);
       
}















void PurePursuit::lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    const std::vector<float>& ranges = msg->ranges;
    size_t num_ranges = ranges.size();
    double angle_min = msg->angle_min;
    double angle_increment = msg->angle_increment;

    // ----- 1. LIDAR DATA SMOOTHING -----
    // Apply moving average filter for smoother readings
    std::vector<float> smoothed_ranges(ranges.size());
    const int filter_window = 5; // Must be odd
    const int half_window = filter_window / 2;
    
    for (size_t i = 0; i < ranges.size(); ++i) {
        float sum = 0.0f;
        int count = 0;
        for (int j = -half_window; j <= half_window; ++j) {
            int idx = static_cast<int>(i) + j;
            if (idx >= 0 && idx < static_cast<int>(ranges.size())) {
                sum += ranges[idx];
                count++;
            }
        }
        smoothed_ranges[i] = sum / count;
    }

    // ----- 2. CLIP RANGES TO 2.5M -----
    std::vector<float> clipped_ranges(smoothed_ranges.size());
    for (size_t i = 0; i < smoothed_ranges.size(); ++i) {
        clipped_ranges[i] = std::min(smoothed_ranges[i], 2.5f);
    }

    // ----- 3. FIND WIDEST GAP -----
    // Static variables for persistence across function calls
    static int prev_gap_mid_idx = -1;
    static double gap_direction_bias = 0.0;
    
    int max_gap_start = -1, max_gap_end = -1;
    int curr_gap_start = -1;
    int max_gap_size = 0;
    
    // Find all gaps
    std::vector<std::pair<int, int>> gaps;
    
    for (size_t i = 0; i < clipped_ranges.size(); ++i) {
        if (clipped_ranges[i] >= 2.49f) { // Almost 2.5m (epsilon for float comparison)
            if (curr_gap_start == -1) curr_gap_start = i;
            if (i == clipped_ranges.size() - 1 && curr_gap_start != -1) {
                gaps.push_back(std::make_pair(curr_gap_start, i));
                if ((i - curr_gap_start + 1) > max_gap_size) {
                    max_gap_size = i - curr_gap_start + 1;
                    max_gap_start = curr_gap_start;
                    max_gap_end = i;
                }
            }
        } else {
            if (curr_gap_start != -1) {
                gaps.push_back(std::make_pair(curr_gap_start, i - 1));
                if ((i - curr_gap_start) > max_gap_size) {
                    max_gap_size = i - curr_gap_start;
                    max_gap_start = curr_gap_start;
                    max_gap_end = i - 1;
                }
                curr_gap_start = -1;
            }
        }
    }
    
    // Check if we have multiple similar-sized gaps (the oscillation case)
    bool has_ambiguous_gaps = false;
    int second_best_gap_idx = -1;
    
    if (gaps.size() >= 2) {
        // Sort gaps by size (largest first)
        std::sort(gaps.begin(), gaps.end(), 
            [](const auto& a, const auto& b) { 
                return (a.second - a.first) > (b.second - b.first); 
            });
        
        // If top two gaps are similar in size (within 15%)
        int best_gap_size = gaps[0].second - gaps[0].first;
        int second_gap_size = gaps[1].second - gaps[1].first;
        
        if (second_gap_size > 0.85 * best_gap_size) {
            has_ambiguous_gaps = true;
            
            // Calculate midpoints
            int best_mid = (gaps[0].first + gaps[0].second) / 2;
            int second_mid = (gaps[1].first + gaps[1].second) / 2;
            second_best_gap_idx = second_mid;
            
            // If we had a previous choice, stick with similar direction
            if (prev_gap_mid_idx != -1) {
                // Determine if we should maintain direction
                int center_idx = num_ranges / 2;
                bool prev_was_left = prev_gap_mid_idx < center_idx;
                bool best_is_left = best_mid < center_idx;
                bool second_is_left = second_mid < center_idx;
                
                // If previous direction matches second best, use that instead
                if (prev_was_left == second_is_left && prev_was_left != best_is_left) {
                    // Swap the gaps
                    max_gap_start = gaps[1].first;
                    max_gap_end = gaps[1].second;
                    gap_direction_bias = 0.8; // Strong bias to maintain direction
                } else {
                    gap_direction_bias *= 0.9; // Decay bias when continuing same direction
                }
            }
        } else {
            gap_direction_bias = 0.0; // Reset bias when gap choice is clear
        }
    }
    
    if (max_gap_start == -1 || max_gap_end == -1) {
        RCLCPP_WARN(this->get_logger(), "No valid gap found!");
        return;
    }
    
    // ----- 4. CALCULATE MIDPOINT OF BEST GAP -----
    int gap_mid_idx = (max_gap_start + max_gap_end) / 2;
    double gap_angle = angle_min + gap_mid_idx * angle_increment;
    
    // Update for next iteration
    prev_gap_mid_idx = gap_mid_idx;
    
    // Convert the gap midpoint to Cartesian coordinates (in vehicle frame)
    double best_x = clipped_ranges[gap_mid_idx] * cos(gap_angle);
    double best_y = clipped_ranges[gap_mid_idx] * sin(gap_angle);
    Eigen::Vector2d best_point(best_x, best_y);
    
    // ----- 5. FIND CLOSEST CANDIDATE GOAL -----
    int closest_goal_idx = -1;
    double min_dist = std::numeric_limits<double>::max();
    
    for (int i = 0; i < candidate_goals.cols(); ++i) {
        Eigen::Vector2d translated = translate_point(car_pos, candidate_goals.col(i));
        double dist = (translated - best_point).norm();
        
        // Apply directional bias if we have ambiguous gaps
        if (has_ambiguous_gaps && second_best_gap_idx != -1) {
            // Calculate angle to second best gap
            double second_angle = angle_min + second_best_gap_idx * angle_increment;
            double second_x = clipped_ranges[second_best_gap_idx] * cos(second_angle);
            double second_y = clipped_ranges[second_best_gap_idx] * sin(second_angle);
            Eigen::Vector2d second_point(second_x, second_y);
            
            // Calculate distance to second best point
            double second_dist = (translated - second_point).norm();
            
            // If we're close to the second best but with bias considered it's better,
            // reduce the distance to make it more attractive
            if (second_dist < dist * 1.2) {
                dist = dist * (1.0 + gap_direction_bias);
            }
        }
        
        if (dist < min_dist) {
            min_dist = dist;
            closest_goal_idx = i;
        }
    }
    
    if (closest_goal_idx == -1) {
        RCLCPP_WARN(this->get_logger(), "No valid candidate goal found!");
        return;
    }
    
    // ----- 6. APPLY ADAPTIVE SMOOTHING TO INDEX -----
    static int prev_idx = -1;
    static double smoothed_idx = closest_goal_idx;
    
    if (prev_idx == -1) {
        // First run, initialize
        smoothed_idx = closest_goal_idx;
    } else {
        // Calculate index change magnitude
        int idx_change = std::abs(closest_goal_idx - prev_idx);
        
        // Determine adaptive smoothing factor based on change magnitude
        double alpha;
        if (idx_change >= 4) {
            // Large change - respond quickly
            alpha = 0.8;
        } else if (idx_change >= 2) {
            // Medium change
            alpha = 0.5;
        } else if (idx_change == 1) {
            // Small change - apply hysteresis to prevent oscillation
            // If we're oscillating between adjacent indices, stick with previous
            static int oscillation_count = 0;
            static int last_idx_change_dir = 0;
            
            int curr_idx_change_dir = closest_goal_idx - prev_idx;
            
            // If direction changed and we're just toggling between two values
            if (curr_idx_change_dir * last_idx_change_dir < 0) {
                oscillation_count++;
                
                // If oscillating, maintain previous index
                if (oscillation_count >= 3) {
                    closest_goal_idx = prev_idx;
                    alpha = 0.1; // Very slow change
                } else {
                    alpha = 0.3; // Moderate change
                }
            } else {
                oscillation_count = 0;
                alpha = 0.4; // Normal small change
            }
            
            last_idx_change_dir = curr_idx_change_dir;
        } else {
            // No change
            alpha = 0.5;
        }
        
        // Apply exponential smoothing
        smoothed_idx = alpha * closest_goal_idx + (1.0 - alpha) * smoothed_idx;
    }
    
    // Convert smoothed index to integer
    closest_goal_idx = static_cast<int>(std::round(smoothed_idx));
    closest_goal_idx = std::max(0, std::min(closest_goal_idx, static_cast<int>(candidate_goals.cols()) - 1));
    
    // Store for next iteration
    prev_idx = closest_goal_idx;
    printf("Closest goal index: %d\n", closest_goal_idx);
    printf("Smoothed goal index: %d\n", static_cast<int>(smoothed_idx));
    
    // ----- 7. PURSUE SELECTED GOAL -----
    Eigen::Vector2d selected_goal = candidate_goals.col(closest_goal_idx);
    Eigen::Vector2d translated = translate_point(car_pos, selected_goal);
    
    // Compute curvature and steering angle
    double curvature = 2 * translated.y() / (L * L);
    double steering_angle = P * curvature;
    
    // Compute speed with dynamic adjustment
    ackermann_msgs::msg::AckermannDriveStamped drive_msg;
    const double max_speed = 4.5;
    const double min_speed = 2.0;
    double speed_curvature = 2 * translated.y() / 1;
    double speed = max_speed - ((max_speed - min_speed) * speed_curvature * 2);
    speed = std::clamp(speed, min_speed, max_speed);
    
    drive_msg.drive.speed = speed;
    drive_msg.drive.steering_angle = steering_angle;
    drive_pub->publish(drive_msg);
}





















Eigen::Vector2d PurePursuit::translate_point(const Eigen::Vector2d& curr, const Eigen::Vector2d& target) {
    Eigen::Matrix4d H = Eigen::Matrix4d::Identity();
    H.block<3, 3>(0, 0) = rot;
    H(0, 3) = curr.x();
    H(1, 3) = curr.y();
    Eigen::Vector4d dir(target.x() - curr.x(), target.y() - curr.y(), 0, 0);
    Eigen::Vector4d translated = H * dir;
    return Eigen::Vector2d(translated.x(), translated.y());
}






void PurePursuit::publish_candidate_markers(const Eigen::Matrix<double, 2,3> candidate_goals){
    visualization_msgs::msg::MarkerArray candidate_markers;
    for (int i = 0; i < 3; ++i) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "candidate_goals";
        marker.id = i;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = candidate_goals(0, i);
        marker.pose.position.y = candidate_goals(1, i);
        marker.pose.position.z = 0.1;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        candidate_markers.markers.push_back(marker);
    }
    marker_pub->publish(candidate_markers);
}


void PurePursuit::publish_waypoints_markers() {
    if (waypoints.empty()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                            "No waypoints to visualize");
        return;
    }

    visualization_msgs::msg::MarkerArray marker_array;
    for (size_t i = 0; i < waypoints.size(); ++i) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "waypoints";
        marker.id = i;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = waypoints[i][0];
        marker.pose.position.y = waypoints[i][1];
        marker.pose.position.z = 0.1;
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker_array.markers.push_back(marker);
    }
    marker_pub->publish(marker_array); // dont publish in the race, reduces computation time slightly
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<PurePursuit>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception in main: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}