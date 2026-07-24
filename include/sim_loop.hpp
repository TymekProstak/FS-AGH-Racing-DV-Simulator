// sim_loop.hpp

#pragma once

#include "double_track.hpp"
#include "cone_detector.hpp"
#include "cone_track.hpp"
#include "centerline_metrics.hpp"
#include "runtime_helpers.hpp"
#include "imu_simulator.hpp"
#include "torque_allocation.hpp"
#include "visualization_helpers.hpp"

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>

#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/TransformStamped.h>

#include "dv_interfaces/Control.h"
#include "dv_interfaces/DV_board.h"
#include "dv_interfaces/Imu.h"
#include "dv_interfaces/Cone.h"
#include "dv_interfaces/Cones.h"
#include "dv_interfaces/full_state.h"

#include <string>
#include <algorithm>
#include <cmath>
#include <random>
#include <utility>
#include <vector>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

namespace lem_dynamics_sim_
{

struct StateEstimate
{
    double x{};
    double y{};
    double yaw{};
    double vx{};
    double vy{};
    double yaw_rate{};
};

struct StatePoseSample
{
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double yaw_rate = 0.0;
};

struct StateSpeedSample
{
    double vx = 0.0;
    double vy = 0.0;
};

class Simulation_lem_ros_node
{
    std::mt19937 phase_rng_{std::random_device{}()};

public:
    Simulation_lem_ros_node(ros::NodeHandle& nh,
                            const std::string& param_file,
                            const std::string& track_file,
                            const std::string& centerline_file,
                            const std::string& log_file);

    ~Simulation_lem_ros_node();

    void step();

    const State& get_state() const { return state_; }
    const ParamBank& get_parameters() const { return P_; }
    int get_step_number() const { return step_number_; }

    void control_command_callback(const dv_interfaces::Control::ConstPtr& msg);

private:
    std::string metrics_log_file_path_;
    void log_metric_of_ride_data_();

    CenterlineMetrics centerline_metrics_;
    double absolute_lateral_error_sum_m_ = 0.0;
    double absolute_heading_error_sum_rad_ = 0.0;
    double path_speed_sum_mps_ = 0.0;
    std::size_t ride_metric_samples_ = 0;

    std::vector<double> ten_biggest_slip_ratio_;
    std::vector<double> ten_biggest_beta_angle_;
    std::vector<double> ten_biggest_ey_;
    std::vector<double> ten_biggest_epsi_;

    double percentage_time_sideslip_over_threshold_ = 0.0;
    double time_sideslip_over_threshold_s_ = 0.0;
    double time_sideslip_evaluated_s_ = 0.0;

    ros::Subscriber sub_control_;

    ros::Publisher pub_state_estimate_;
    ros::Publisher pub_imu_;
    ros::Publisher pub_dv_board_data_;
    ros::Publisher pub_steer_;

    ros::Publisher pub_cones_;
    ros::Publisher pub_lidar_cones_;

    ros::Publisher pub_markers_cones_gt_;
    ros::Publisher pub_markers_cones_vis_;
    ros::Publisher pub_markers_cones_lidar_;

    ros::Publisher pub_log_full_;
    ros::Publisher pub_marker_bolid_;
    ros::Publisher pub_gg_sphere_marker_;

    ros::Publisher pub_steer_status_;

    tf2_ros::StaticTransformBroadcaster tf_static_br_;
    tf2_ros::TransformBroadcaster tf_br_;

    ParamBank P_;
    State state_;
    Track track_global_;

    double wheel_speed_read_fl = 0.0;
    double wheel_speed_read_fr = 0.0;
    double wheel_speed_read_rl = 0.0;
    double wheel_speed_read_rr = 0.0;

   

    WheelTorqueCommand dv_board_to_main_torque_command_;
    WheelTorqueCommand active_main_torque_command_;
    DelayedQueue<WheelTorqueCommand> pending_main_outputs_;
    bool has_dv_board_to_main_command_ = false;
    int dv_board_to_main_move_type_ = dv_interfaces::Control::FOUR_WHEEL;

    double rack_angle_command_ = 0.0;
    ValueCache<double> measured_steering_cache_;

    ValueCache<dv_interfaces::Control> control_callback_cache_;
    dv_interfaces::Control last_input_read_by_dv_board;

    bool first_control_input_received_ = false;

    StateEstimate state_estimate_;
    ValueCache<StatePoseSample> ready_estimated_pose_;
    ValueCache<StateSpeedSample> ready_estimated_speed_;
    DelayedQueue<StatePoseSample> pending_estimated_poses_;
    DelayedQueue<StateSpeedSample> pending_estimated_speeds_;
    ValueCache<StateEstimate> vcu_state_estimate_cache_;

    double estimator_bias_x_ = 0.0;
    double estimator_bias_y_ = 0.0;
    double estimator_bias_yaw_ = 0.0;
    double estimator_bias_vx_ = 0.0;
    double estimator_bias_vy_ = 0.0;
    double estimator_bias_yaw_rate_ = 0.0;

    int step_number_ = 0;

    PeriodicTimer camera_timer_;
    PeriodicTimer lidar_timer_;
    PeriodicTimer wheel_encoder_timer_;
    PeriodicTimer state_estimator_timer_;
    PeriodicTimer control_input_timer_;
    PeriodicTimer steering_command_timer_;
    PeriodicTimer dv_board_to_main_timer_;
    PeriodicTimer main_loop_timer_;
    PeriodicTimer main_imu_timer_;
    PeriodicTimer dv_board_publish_timer_;
    PeriodicTimer steering_publish_timer_;

    int main_computation_delay_steps_ = 0;
    int estimator_pose_yaw_delay_steps_ = 0;
    int estimator_speed_delay_steps_ = 0;

    int last_frame_size_ = 0;
    int last_lidar_frame_size_ = 0;

    ImuSimulator imu_;
    ValueCache<dv_interfaces::Imu> published_imu_cache_;
    ValueCache<dv_interfaces::Imu> main_imu_cache_;

    bool use_only_lidar = true;
    bool use_only_camera = false;
    bool use_fusion = false;

    int sim_time = -1;

    struct CameraTask
    {
        int ready_step;
        Track frame;
    };

    std::deque<CameraTask> camera_queue_;
    std::deque<ros::Time> timestamp_queue_;

    struct LidarTask
    {
        int ready_step;
        Track frame;
    };

    std::deque<LidarTask> lidar_queue_;
    std::deque<ros::Time> lidar_timestamp_queue_;

    double random_noise_generator_() const;
    void configure_timers_();
    double sample_vision_exec_time_() const;
    double sample_lidar_exec_time_() const;

    void update_sensor_pipeline_();
    void update_control_pipeline_();
    void integrate_vehicle_dynamics_();
    void update_ride_metrics_();
    void publish_outputs_();
    void flush_perception_queues_();
    void stop_when_time_limit_reached_();

    void publish_state_estimate_(const StateEstimate& estimate);

    void publish_cones_(const Track& cones, ros::Time timestamp);
    void publish_lidar_cones_(const Track& cones, ros::Time timestamp);

    void publish_cones_vision_markers_(const Track& det, const ros::Time& acquisition_stamp);
    void publish_cones_lidar_markers_(const Track& det, const ros::Time& acquisition_stamp);
    void publish_cones_gt_markers_();

    void publish_ready_camera_frames_from_queue_();
    void publish_ready_lidar_frames_from_queue_();

    void update_state_estimator_();
    void read_control_by_dv_board_if_due();
    void read_steer_by_orin_if_due_();

    void update_imu_();
    void main_read_imu_if_due_();

    void shoot_camera_or_enqueue_if_due_();
    void shoot_lidar_or_enqueue_if_due_();
    void send_dv_board_to_main_if_due_();
    void run_main_if_due_();
    void apply_ready_main_outputs_();
    WheelTorqueCommand compute_main_torque_command_();

    void publish_dv_board_data_if_due_();
    void publish_steer_if_due_();

    void publish_estimated_vehicle_tf_(
        const StateEstimate& estimate);
    void publish_static_vehicle_transforms_();
    void publish_bolid_tf_true();
    void pub_full_state_();
    void publish_bolid_marker_();

    void publish_steer_actuator_status_();

    void read_wheel_encoder_if_due_();
};

} // namespace lem_dynamics_sim_
