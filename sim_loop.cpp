// sim_loop.cpp

#include "sim_loop.hpp"
#include <array>
#include <iostream>
#include <exception>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <limits>
#include <vector>

namespace lem_dynamics_sim_ {

    static inline void update_top_abs(std::vector<double>& v, double x, std::size_t N = 10)
    {
        const double ax = std::abs(x);

        // jeśli jeszcze nie ma N elementów -> wrzuć i posortuj
        if (v.size() < N) {
            v.push_back(x);
            std::sort(v.begin(), v.end(),
                    [](double a, double b){ return std::abs(a) > std::abs(b); });
            return;
        }

        // jeśli x nie jest większe niż najmniejsze z TOP -> ignore
        double smallest_abs = std::abs(v.back());
        if (ax <= smallest_abs) return;

        // podmień najmniejsze i ponownie posortuj
        v.back() = x;
        std::sort(v.begin(), v.end(),
                [](double a, double b){ return std::abs(a) > std::abs(b); });
    }

using json = nlohmann::json;

// ====== Konstruktor / inicjalizacja ======
Simulation_lem_ros_node::Simulation_lem_ros_node(ros::NodeHandle& nh,
                                                 const std::string& param_file,
                                                 const std::string& cones_file,
                                                 const std::string& centerline_file,
                                                 const std::string& log_file )
{
    // --- DIAG: wczytywanie parametrów ---
    std::cout << "[INIT] Opening param file: " << param_file << std::endl;
    {
        try {
            std::ifstream f(param_file);
            if (!f.is_open()) {
                std::cerr << "[INIT][FAIL] Cannot open param file." << std::endl;
                throw std::runtime_error("Nie mogę otworzyć pliku parametrów: " + param_file);
            }
            std::cout << "[INIT] Param file opened. Parsing JSON..." << std::endl;

            nlohmann::json J;
            f >> J;
            std::cout << "[INIT] JSON parsed. Building ParamBank..." << std::endl;

            P_ = build_param_bank(J);
            std::cout << "[INIT][OK] ParamBank built. Count=" << P_.size() << std::endl;
        }
        catch (const std::exception& e) {
            std::cerr << "[INIT][FAIL] Parameter loading failed. what(): " << e.what() << std::endl;
            throw; // nie zmieniamy logiki — dalej rzucamy wyjątek
        }
    }

    // Initial pose belongs to the selected track and is supplied by its launch.
    state_.setZero();
    nh.param("initial_x_m", state_.x, 0.0);
    nh.param("initial_y_m", state_.y, 0.0);
    nh.param("initial_yaw_rad", state_.yaw, 0.0);

    // --- DIAG: wczytanie pachołków ---
    std::cout << "[INIT] Loading cones file: " << cones_file << std::endl;
    try {
        track_global_ = load_track_from_csv(cones_file);
        std::cout << "[INIT][OK] Cones loaded. cones=" << track_global_.cones.size() << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "[INIT][FAIL] Cones load failed. what(): " << e.what() << std::endl;
        throw;
    }

    std::cout << "[INIT] Loading centerline file: "
              << centerline_file << std::endl;
    centerline_metrics_.load_csv(centerline_file);
    std::cout << "[INIT][OK] Centerline loaded. points="
              << centerline_metrics_.point_count() << std::endl;

    // Every asynchronous simulated device owns a named timer.
    try {
        configure_timers_();
    } catch (const std::exception& e) {
        std::cerr << "[INIT][FAIL] Timer configuration failed: "
                  << e.what() << std::endl;
        throw;
    }

    // 4) ROS I/O
    std::cout << "[INIT] Initializing ROS I/O (subscribers/publishers)..." << std::endl;
    try {
        sub_control_ = nh.subscribe<dv_interfaces::Control>(
            "/dv_board/control", 1, &Simulation_lem_ros_node::dv_control_callback, this, ros::TransportHints().tcpNoDelay());
        pub_state_estimate_ =
            nh.advertise<nav_msgs::Odometry>("/ins/pose", 1);
        pub_imu_   = nh.advertise<dv_interfaces::Imu>("/dv_board/imu", 1);
        pub_dv_board_data_ = nh.advertise<dv_interfaces::DV_board>("/dv_board/data", 1);
        pub_steer_ = nh.advertise<std_msgs::Float64>("/servo_node/cubemars/encoder_absolute", 1);
        pub_cones_ = nh.advertise<dv_interfaces::Cones>("/dv_cone_detector/cones", 1);
        pub_lidar_cones_ = nh.advertise<dv_interfaces::Cones>("/dv_cone_detector/cones", 1);
        pub_markers_cones_gt_  = nh.advertise<visualization_msgs::MarkerArray>("/viz/cones_gt", 1,true);
        pub_markers_cones_vis_ = nh.advertise<visualization_msgs::MarkerArray>("/viz/cones_vis", 1);
        pub_markers_cones_lidar_ = nh.advertise<visualization_msgs::MarkerArray>("/viz/cones_lidar", 1);
        pub_log_full_ = nh.advertise<dv_interfaces::full_state>("/debug/full_log_info", 1);
        // One vehicle body marker plus four wheel-load arrows are published
        // together on this topic.
        pub_marker_bolid_ = nh.advertise<visualization_msgs::Marker>("/viz/bolide_model", 10);
        pub_gg_sphere_marker_ = nh.advertise<visualization_msgs::Marker>("/simulation/gg_sphere", 1);
        pub_steer_status_ = nh.advertise<std_msgs::Bool>("/servo_node/cubemars/initialization_complete", 1, true);
        std::cout << "[INIT][OK] ROS I/O ready." << std::endl;

        auto check_pub = [&](const char* name, const ros::Publisher& p){
            if (!p) {
                std::cerr << "[INIT][FAIL] Publisher invalid right after advertise(): " << name << std::endl;
                ROS_ERROR("Publisher invalid right after advertise(): %s", name);
            }
        };
        check_pub("pub_state_estimate_",    pub_state_estimate_);
        check_pub("pub_imu_",               pub_imu_);
        check_pub("pub_dv_board_data_",     pub_dv_board_data_);
        check_pub("pub_steer_",             pub_steer_);
        check_pub("pub_cones_",             pub_cones_);
        check_pub("pub_lidar_cones_",       pub_lidar_cones_);
        check_pub("pub_markers_cones_gt_",  pub_markers_cones_gt_);
        check_pub("pub_markers_cones_vis_", pub_markers_cones_vis_);
        check_pub("pub_markers_cones_lidar_", pub_markers_cones_lidar_);

    } catch (const std::exception& e) {
        std::cerr << "[INIT][FAIL] ROS I/O init failed. what(): " << e.what() << std::endl;
        throw;
    }

    // 5) publikacja conów z toru (ground truth) o wiecznym life-time
    std::cout << "[INIT] Publishing GT cones markers..." << std::endl;
    try {
        publish_cones_gt_markers_();
        std::cout << "[INIT][OK] GT cones published." << std::endl;
    } catch (const std::exception& e) {
        // nie przerywamy — to tylko markery
        std::cerr << "[INIT][WARN] publish_cones_gt_markers_ failed. what(): " << e.what() << std::endl;
    }

    // 6) Inicjalizacja logowania metryk jazdy
    if (!log_file.empty()) {
        metrics_log_file_path_ = log_file;
        const auto pos = metrics_log_file_path_.rfind(".csv");
        if (pos != std::string::npos) {
            metrics_log_file_path_.replace(pos, 4, "_metrics.csv");
        } else {
            metrics_log_file_path_ += "_metrics.csv";
        }
    } else {
        metrics_log_file_path_.clear(); // metryki wyłączone jeśli log_file pusty
    }

    // 7) Kolejki puste na start
    camera_queue_.clear();
    timestamp_queue_.clear();
    lidar_queue_.clear();
    lidar_timestamp_queue_.clear();

    std::cout << "[INIT][DONE] Node constructed successfully." << std::endl;

    ROS_WARN_STREAM("Init yaw=" << state_.yaw << " vy=" << state_.vy);

    {
        bool launch_use_lidar  = false;
        bool launch_use_camera = false;
        bool launch_use_fusion = false;

        nh.param<bool>("use_lidar",  launch_use_lidar,  false);
        nh.param<bool>("use_camera", launch_use_camera, false);
        nh.param<bool>("use_fusion", launch_use_fusion, false);

        // ============================================================
        // Resolve perception mode.
        // Priorytet:
        // 1) fusion
        // 2) lidar only
        // 3) camera only
        // 4) fallback: lidar only
        //
        // Dzięki temu tylko jeden pipeline publikuje detekcje na
        // /dv_cone_detector/cones.
        // ============================================================
        if (launch_use_fusion)
        {
            use_fusion      = true;
            use_only_lidar  = false;
            use_only_camera = false;
        }
        else if (launch_use_lidar)
        {
            use_fusion      = false;
            use_only_lidar  = true;
            use_only_camera = false;
        }
        else if (launch_use_camera)
        {
            use_fusion      = false;
            use_only_lidar  = false;
            use_only_camera = true;
        }
        else
        {
            ROS_WARN_STREAM(
                "[INIT][PERCEPTION] No perception mode enabled in launch. "
                "Falling back to lidar-only."
            );

            use_fusion      = false;
            use_only_lidar  = true;
            use_only_camera = false;
        }

        ROS_WARN_STREAM(
            "[INIT][PERCEPTION] mode:"
            << " use_only_lidar="  << use_only_lidar
            << " use_only_camera=" << use_only_camera
            << " use_fusion="      << use_fusion
        );
    }
    nh.param("sim_time", sim_time, -1);
    std::cout << "[INIT] sim_time = " << sim_time
            << "  ( <0 => infinite , >=0 => stop after sim_time seconds )"
            << std::endl;

    ROS_WARN_STREAM("[INIT] sim_time=" << sim_time);
}

// ====== Destruktor ======
Simulation_lem_ros_node::~Simulation_lem_ros_node() {
    std::cout << "[DTOR] Saving ride metrics..." << std::endl;
    log_metric_of_ride_data_();

}

// ====== Interfejs publiczny ======
void Simulation_lem_ros_node::step()
{
    update_sensor_pipeline_();
    update_control_pipeline_();
    integrate_vehicle_dynamics_();
    ++step_number_;
    publish_outputs_();
    flush_perception_queues_();
    stop_when_time_limit_reached_();
}

void Simulation_lem_ros_node::update_sensor_pipeline_()
{
    read_wheel_encoder_if_due_();
    update_state_estimator_();
    update_imu_();
    main_read_imu_if_due_();
    shoot_camera_or_enqueue_if_due_();
    shoot_lidar_or_enqueue_if_due_();
}

void Simulation_lem_ros_node::update_control_pipeline_()
{
    read_control_by_dv_board_if_due();

    if (first_control_input_received_ && last_input_read_by_dv_board.finished &&
        state_.vx < 0.05) {
        state_.vx = 0.0;
        state_.omega_fl = 0.0;
        state_.omega_fr = 0.0;
        state_.omega_rl = 0.0;
        state_.omega_rr = 0.0;
    }

    send_dv_board_to_main_if_due_();
    run_main_if_due_();
    apply_ready_main_outputs_();
    read_steer_by_orin_if_due_();
}

void Simulation_lem_ros_node::integrate_vehicle_dynamics_()
{
    euler_sim_timestep(
        state_,
        Input(
            active_main_torque_command_.fl,
            active_main_torque_command_.fr,
            active_main_torque_command_.rl,
            active_main_torque_command_.rr,
            rack_angle_command_),
        P_);
    update_ride_metrics_();
}

void Simulation_lem_ros_node::update_ride_metrics_()
{
    const CenterlineTrackingError error = centerline_metrics_.evaluate(
        state_.x,
        state_.y,
        state_.yaw,
        state_.vx,
        state_.vy);

    absolute_lateral_error_sum_m_ +=
        std::abs(error.lateral_error_m);
    absolute_heading_error_sum_rad_ +=
        std::abs(error.heading_error_rad);
    path_speed_sum_mps_ += error.path_speed_mps;
    ++ride_metric_samples_;

    update_top_abs(
        ten_biggest_ey_, error.lateral_error_m, 10);
    update_top_abs(
        ten_biggest_epsi_, error.heading_error_rad, 10);
}

void Simulation_lem_ros_node::publish_outputs_()
{
    pub_full_state_();
    publish_bolid_marker_();
    publish_bolid_tf_true();
    publish_dv_board_data_if_due_();
    publish_steer_if_due_();
    publish_steer_actuator_status_();
}

void Simulation_lem_ros_node::flush_perception_queues_()
{
    publish_ready_camera_frames_from_queue_();
    publish_ready_lidar_frames_from_queue_();
}

void Simulation_lem_ros_node::stop_when_time_limit_reached_()
{
    if (sim_time < 0) {
        return;
    }

    const double simulated_time_s =
        step_number_ * P_.get("simulation_time_step");
    if (simulated_time_s >= static_cast<double>(sim_time)) {
        ROS_INFO_STREAM("Simulation time limit reached at "
                        << simulated_time_s << " s");
        ros::shutdown();
    }
}

// dv board reads control topic at fixed cadence
void Simulation_lem_ros_node::read_control_by_dv_board_if_due()
{
    if (control_input_timer_.due(step_number_) &&
        control_callback_cache_.has_value()) {
        last_input_read_by_dv_board = control_callback_cache_.value();
    }
}

void Simulation_lem_ros_node::read_wheel_encoder_if_due_()
{
    if (!wheel_encoder_timer_.due(step_number_)) return;

    const double R = P_.get("R");

    wheel_speed_read_fl = state_.omega_fl * R;
    wheel_speed_read_fr = state_.omega_fr * R;
    wheel_speed_read_rl = state_.omega_rl * R;
    wheel_speed_read_rr = state_.omega_rr * R;
}

void Simulation_lem_ros_node::read_steer_by_orin_if_due_()
{
    if (!steering_command_timer_.due(step_number_) ||
        !control_callback_cache_.has_value()) {
        return;
    }

    rack_angle_command_ =
        control_callback_cache_.value().steeringAngle_rad +
        P_.get("steering_bias_rad") +
        P_.get("steering_actuator_noise_std_rad") *
            random_noise_generator_();
}

// ====== ROS callback ======
// caching last requested input from control
void Simulation_lem_ros_node::dv_control_callback(const dv_interfaces::Control::ConstPtr& msg)
{
    first_control_input_received_ = true;
    control_callback_cache_.write(*msg);
}

void Simulation_lem_ros_node::configure_timers_()
{
    const double dt = P_.get("simulation_time_step");
    camera_timer_.configure(
        1.0 / P_.get("frames_per_second"), dt, phase_rng_);
    lidar_timer_.configure(
        1.0 / P_.get("lidar_frames_per_second"), dt, phase_rng_);
    wheel_encoder_timer_.configure(
        P_.get("wheel_encoder_reading_time_step"), dt, phase_rng_);
    state_estimator_timer_.configure(
        1.0 / P_.get("state_estimator_frequency_hz"), dt, phase_rng_);
    control_input_timer_.configure(
        P_.get("control_to_dv_board_read_time_step"), dt, phase_rng_);
    steering_command_timer_.configure(
        P_.get("steering_command_read_time_step"), dt, phase_rng_);
    dv_board_to_main_timer_.configure(
        P_.get("dv_board_to_main_time_step"), dt, phase_rng_);
    main_loop_timer_.configure(
        P_.get("main_loop_time_step"), dt, phase_rng_);
    main_imu_timer_.configure(
        P_.get("main_imu_read_time_step"), dt, phase_rng_);
    dv_board_publish_timer_.configure(
        P_.get("dv_board_data_publishing_time_step"), dt, phase_rng_);
    steering_publish_timer_.configure(
        P_.get("steer_publishing_time_step"), dt, phase_rng_);
    imu_.configure(P_, dt, phase_rng_);

    main_computation_delay_steps_ =
        std::max(0, static_cast<int>(std::round(P_.get("main_computation_delay_s") / dt)));

    estimator_pose_yaw_delay_steps_ =
        std::max(0, static_cast<int>(std::round(
            P_.get("state_estimator_pose_yaw_delay_s") / dt)));

    estimator_speed_delay_steps_ =
        std::max(0, static_cast<int>(std::round(
            P_.get("state_estimator_speed_delay_s") / dt)));
}

// dv board reads control topic at fixed cadence
void Simulation_lem_ros_node::update_state_estimator_()
{
    const bool estimator_due =
        state_estimator_timer_.due(step_number_);

    const double dt =
        P_.get("simulation_time_step") *
        state_estimator_timer_.period_steps();

    const double sqrt_dt = std::sqrt(dt);

    if (estimator_due)
    {
        estimator_bias_x_ += P_.get("state_estimator_position_bias_rw") *
            sqrt_dt * random_noise_generator_();
        estimator_bias_y_ += P_.get("state_estimator_position_bias_rw") *
            sqrt_dt * random_noise_generator_();
        estimator_bias_yaw_ += P_.get("state_estimator_yaw_bias_rw") *
            sqrt_dt * random_noise_generator_();
        estimator_bias_vx_ += P_.get("state_estimator_speed_bias_rw") *
            sqrt_dt * random_noise_generator_();
        estimator_bias_vy_ += P_.get("state_estimator_speed_bias_rw") *
            sqrt_dt * random_noise_generator_();
        estimator_bias_yaw_rate_ +=
            P_.get("state_estimator_yaw_rate_bias_rw") *
            sqrt_dt * random_noise_generator_();

        StatePoseSample pose_out;
        pose_out.x = state_.x + estimator_bias_x_ +
            P_.get("state_estimator_position_noise_std") *
                random_noise_generator_();
        pose_out.y = state_.y + estimator_bias_y_ +
            P_.get("state_estimator_position_noise_std") *
                random_noise_generator_();
        pose_out.yaw = state_.yaw + estimator_bias_yaw_ +
            P_.get("state_estimator_yaw_noise_std") *
                random_noise_generator_();
        pose_out.yaw_rate =
            state_.yaw_rate + estimator_bias_yaw_rate_ +
            P_.get("state_estimator_yaw_rate_noise_std") *
                random_noise_generator_();
        unwrap_angle(pose_out.yaw);
        pending_estimated_poses_.push(
            step_number_ + estimator_pose_yaw_delay_steps_, pose_out);

        StateSpeedSample speed_out;
        speed_out.vx = state_.vx + estimator_bias_vx_ +
            P_.get("state_estimator_speed_noise_std") *
                random_noise_generator_();
        speed_out.vy = state_.vy + estimator_bias_vy_ +
            P_.get("state_estimator_speed_noise_std") *
                random_noise_generator_();
        pending_estimated_speeds_.push(
            step_number_ + estimator_speed_delay_steps_, speed_out);
    }

    if (auto pose =
            pending_estimated_poses_.take_latest_ready(step_number_)) {
        ready_estimated_pose_.write(std::move(*pose));
    }
    if (auto speed =
            pending_estimated_speeds_.take_latest_ready(step_number_)) {
        ready_estimated_speed_.write(std::move(*speed));
    }

    const int calib_steps = std::max(
        1,
        static_cast<int>(P_.get("calibration_time") / P_.get("simulation_time_step"))
    );
    const bool calibrated = (step_number_ > calib_steps);

    if (!estimator_due || !calibrated) return;
    if (!ready_estimated_pose_.has_value() ||
        !ready_estimated_speed_.has_value()) return;

    const auto& pose = ready_estimated_pose_.value();
    const auto& speed = ready_estimated_speed_.value();
    state_estimate_.x = pose.x;
    state_estimate_.y = pose.y;
    state_estimate_.yaw = pose.yaw;
    state_estimate_.yaw_rate = pose.yaw_rate;
    state_estimate_.vx = speed.vx;
    state_estimate_.vy = speed.vy;

    // The VCU/main-board unit receives the same sampled estimate that is
    // published to the autonomous stack on /ins/pose.
    vcu_state_estimate_cache_.write(state_estimate_);
    publish_state_estimate_(state_estimate_);
    publish_estimated_vehicle_tf_(state_estimate_);
}

void Simulation_lem_ros_node::update_imu_()
{
    auto message = imu_.update(step_number_, state_, P_);
    if (!message) {
        return;
    }

    message->header.stamp = ros::Time::now();
    message->header.frame_id = "bolide_CoG";
    pub_imu_.publish(*message);
    published_imu_cache_.write(std::move(*message));
}

void Simulation_lem_ros_node::shoot_camera_or_enqueue_if_due_()
{
    if (!use_only_camera) return;

    if (!camera_timer_.due(step_number_)) return;

    Track detection = shoot_a_frame(track_global_, P_, state_);

    const double dt = P_.get("simulation_time_step");
    const double vision_exec_time = sample_vision_exec_time_();
    const int processing_steps =
        std::max(0, static_cast<int>(std::round(vision_exec_time / dt)));

    CameraTask task;
    task.ready_step = step_number_ + processing_steps;
    task.frame      = std::move(detection);

    if (static_cast<int>(camera_queue_.size()) >= 3)
    {
        camera_queue_.pop_front();
        timestamp_queue_.pop_front();
    }

    camera_queue_.push_back(std::move(task));
    timestamp_queue_.push_back(ros::Time::now());
}

void Simulation_lem_ros_node::shoot_lidar_or_enqueue_if_due_()
{
    if (!use_only_lidar && !use_fusion) return;

    if (!lidar_timer_.due(step_number_)) return;

    Track detection;

    if (use_fusion)
    {
        detection = shoot_a_frame_fusion(track_global_, P_, state_);
    }
    else
    {
        detection = shoot_a_frame_lidar(track_global_, P_, state_);
    }

    const double dt = P_.get("simulation_time_step");
    const double lidar_exec_time = sample_lidar_exec_time_();
    const int processing_steps =
        std::max(0, static_cast<int>(std::round(lidar_exec_time / dt)));

    LidarTask task;
    task.ready_step = step_number_ + processing_steps;
    task.frame      = std::move(detection);

    if (static_cast<int>(lidar_queue_.size()) >= 3)
    {
        lidar_queue_.pop_front();
        lidar_timestamp_queue_.pop_front();
    }

    lidar_queue_.push_back(std::move(task));
    lidar_timestamp_queue_.push_back(ros::Time::now());
}

void Simulation_lem_ros_node::publish_ready_camera_frames_from_queue_()
{
    if (!use_only_camera) return;

    while (!camera_queue_.empty() &&
           camera_queue_.front().ready_step <= step_number_)
    {
        const auto& task = camera_queue_.front();
        const auto& timestamp = timestamp_queue_.front();

        publish_cones_(task.frame, timestamp);
        publish_cones_vision_markers_(task.frame, timestamp);

        camera_queue_.pop_front();
        timestamp_queue_.pop_front();
    }
}

void Simulation_lem_ros_node::publish_ready_lidar_frames_from_queue_()
{
    if (!use_only_lidar && !use_fusion) return;

    while (!lidar_queue_.empty() &&
           lidar_queue_.front().ready_step <= step_number_)
    {
        const auto& task = lidar_queue_.front();
        const auto& timestamp = lidar_timestamp_queue_.front();

        publish_lidar_cones_(task.frame, timestamp);
        publish_cones_lidar_markers_(task.frame, timestamp);

        lidar_queue_.pop_front();
        lidar_timestamp_queue_.pop_front();
    }
}

void Simulation_lem_ros_node::send_dv_board_to_main_if_due_()
{
    if (!dv_board_to_main_timer_.due(step_number_)) {
        return;
    }

    dv_board_to_main_move_type_ =
        last_input_read_by_dv_board.move_type;

    if (dv_board_to_main_move_type_ == dv_interfaces::Control::ONE_WHEEL) {
        const double total_motor_torque_nm =
            static_cast<double>(last_input_read_by_dv_board.torque_FL) +
            static_cast<double>(last_input_read_by_dv_board.torque_FR) +
            static_cast<double>(last_input_read_by_dv_board.torque_RL) +
            static_cast<double>(last_input_read_by_dv_board.torque_RR);
        // Baseline front/rear allocation belongs exclusively to ONE_WHEEL.
        // FOUR_WHEEL requests are copied independently in the branch below.
        dv_board_to_main_torque_command_ =
            allocate_one_wheel_baseline_torque(
                P_, total_motor_torque_nm);
        has_dv_board_to_main_command_ = true;
        return;
    }

    if (dv_board_to_main_move_type_ == dv_interfaces::Control::FOUR_WHEEL) {
        dv_board_to_main_torque_command_.fl =
            control_torque_to_wheel_torque(
                P_, last_input_read_by_dv_board.torque_FL);
        dv_board_to_main_torque_command_.fr =
            control_torque_to_wheel_torque(
                P_, last_input_read_by_dv_board.torque_FR);
        dv_board_to_main_torque_command_.rl =
            control_torque_to_wheel_torque(
                P_, last_input_read_by_dv_board.torque_RL);
        dv_board_to_main_torque_command_.rr =
            control_torque_to_wheel_torque(
                P_, last_input_read_by_dv_board.torque_RR);
        has_dv_board_to_main_command_ = true;
        return;
    }

    dv_board_to_main_torque_command_ = WheelTorqueCommand{};
    has_dv_board_to_main_command_ = true;
    ROS_ERROR_STREAM_THROTTLE(
        1.0,
        "[SIM][DV_BOARD] Unsupported Control.move_type="
        << static_cast<int>(dv_board_to_main_move_type_)
        << ". Publishing zero wheel torque."
    );
}


WheelTorqueCommand
Simulation_lem_ros_node::compute_main_torque_command_()
{
    if (!has_dv_board_to_main_command_) {
        return WheelTorqueCommand{};
    }

    WheelTorqueCommand out = dv_board_to_main_torque_command_;
    if (dv_board_to_main_move_type_ == dv_interfaces::Control::FOUR_WHEEL) {
        return out;
    }

    if (dv_board_to_main_move_type_ != dv_interfaces::Control::ONE_WHEEL) {
        return WheelTorqueCommand{};
    }

    if (main_imu_cache_.has_value()) {
        const double speed_mps = 0.25 * (
            wheel_speed_read_fl + wheel_speed_read_fr +
            wheel_speed_read_rl + wheel_speed_read_rr);
        const double measured_steer_rad =
            measured_steering_cache_.value_or(state_.rack_angle);
        const double measured_yaw_rate_radps =
            -main_imu_cache_.value().gyro.z;
        const double raw_wheel_delta_nm =
            compute_left_torque_vectoring_delta(
                P_, speed_mps, measured_steer_rad, measured_yaw_rate_radps);

        const double gear_ratio = P_.get("gear_ratio");
        const double limited_motor_delta_nm = std::clamp(
            raw_wheel_delta_nm / gear_ratio,
            -P_.get("torque_vectoring_max_motor_delta_nm"),
            P_.get("torque_vectoring_max_motor_delta_nm"));
        apply_symmetric_torque_vectoring(
            out, limited_motor_delta_nm * gear_ratio);
    }

    return out;
}


void Simulation_lem_ros_node::run_main_if_due_()
{
    if (!main_loop_timer_.due(step_number_)) {
        return;
    }

    pending_main_outputs_.push(
        step_number_ + main_computation_delay_steps_,
        compute_main_torque_command_());
}

void Simulation_lem_ros_node::apply_ready_main_outputs_()
{
    if (auto command = pending_main_outputs_.take_latest_ready(step_number_)) {
        active_main_torque_command_ = std::move(*command);
    }
}


double Simulation_lem_ros_node::random_noise_generator_() const {
    static thread_local std::mt19937 rng{std::random_device{}()};
    static thread_local std::normal_distribution<double> N01(0.0, 1.0);
    return N01(rng);
}

void Simulation_lem_ros_node::publish_state_estimate_(
    const StateEstimate& estimate)
{
    nav_msgs::Odometry odom_msg{};
    odom_msg.header.stamp = ros::Time::now();
    odom_msg.header.frame_id = "map";
    odom_msg.child_frame_id  = "bolide_CoG";

    odom_msg.pose.pose.position.x = estimate.x;
    odom_msg.pose.pose.position.y = estimate.y;
    odom_msg.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, estimate.yaw);
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    odom_msg.twist.twist.linear.x  = estimate.vx;
    odom_msg.twist.twist.linear.y  = estimate.vy;
    odom_msg.twist.twist.angular.z = estimate.yaw_rate;

    pub_state_estimate_.publish(odom_msg);
}

void Simulation_lem_ros_node::publish_cones_(const Track& cones, ros::Time timestamp)
{
    if (!use_only_camera) return;

    if (!pub_cones_)
    {
        std::cerr << "[PUB][ERROR] pub_cones_ invalid; NOT publishing cones." << std::endl;
        ROS_ERROR("pub_cones_ invalid; not publishing cones");
        return;
    }

    dv_interfaces::Cones cones_msg;
    cones_msg.header.stamp = timestamp;
    cones_msg.header.frame_id = "camera_base";

    for (const auto& cone : cones.cones)
    {
        dv_interfaces::Cone cone_msg;
        cone_msg.confidence = 1.0;
        cone_msg.x = static_cast<float>(cone.x);
        cone_msg.y = static_cast<float>(cone.y);
        cone_msg.z = static_cast<float>(cone.z);
        cone_msg.distance_uncertainty = 0.0f;
        cone_msg.class_name = cone.color;
        cones_msg.cones.push_back(cone_msg);
    }

    pub_cones_.publish(cones_msg);
}

void Simulation_lem_ros_node::publish_lidar_cones_(const Track& cones, ros::Time timestamp)
{
    if (!use_only_lidar && !use_fusion) return;

    if (!pub_lidar_cones_)
    {
        std::cerr << "[PUB][ERROR] pub_lidar_cones_ invalid; NOT publishing lidar/fusion cones." << std::endl;
        ROS_ERROR("pub_lidar_cones_ invalid; not publishing lidar/fusion cones");
        return;
    }

    dv_interfaces::Cones cones_msg;
    cones_msg.header.stamp = timestamp;
    cones_msg.header.frame_id = "os_sensor";

    for (const auto& cone : cones.cones)
    {
        dv_interfaces::Cone cone_msg;
        cone_msg.confidence = 1.0;
        cone_msg.x = static_cast<float>(cone.x);
        cone_msg.y = static_cast<float>(cone.y);
        cone_msg.z = static_cast<float>(cone.z);
        cone_msg.distance_uncertainty = 0.0f;
        cone_msg.class_name = cone.color;
        cones_msg.cones.push_back(cone_msg);
    }

    pub_lidar_cones_.publish(cones_msg);
}

double Simulation_lem_ros_node::sample_vision_exec_time_() const {
    const double mu  = P_.get("mean_time_of_vision_execuction");
    const double var = P_.get("var_of_vision_time_execution");

    static thread_local std::mt19937 rng{std::random_device{}()};
    std::normal_distribution<double> normal(mu, std::sqrt(std::max(0.0, var)));

    const double exec_time = normal(rng);
    return std::abs(exec_time); // czas wykonania nie może być ujemny
}

double Simulation_lem_ros_node::sample_lidar_exec_time_() const {
    const double mu  = P_.get("lidar_execution_time_mean");
    const double var = P_.get("lidar_execution_time_var");

    static thread_local std::mt19937 rng{std::random_device{}()};
    std::normal_distribution<double> normal(mu, std::sqrt(std::max(0.0, var)));

    const double exec_time = normal(rng);
    return std::abs(exec_time); // czas wykonania nie może być ujemny
}

void Simulation_lem_ros_node::publish_cones_vision_markers_(
    const Track& det, const ros::Time& acquisition_stamp)
{
    if (!use_only_camera) return;

    if (!pub_markers_cones_vis_) {
        ROS_ERROR("pub_markers_cones_vis_ invalid; not publishing vision markers");
        return;
    }

    visualization_msgs::MarkerArray arr;

    // Używam dokładnie tego samego timestampu dla całej ramki.
    // To jest czas akwizycji detekcji, a nie czas publikacji.
    const ros::Time stamp = acquisition_stamp;

    // ======================================================
    // 1) Usuwam markery z poprzedniej ramki
    // ======================================================
    for (int id = 200; id < 200 + last_frame_size_; ++id)
    {
        visualization_msgs::Marker del;
        del.header.frame_id = "camera_base";
        del.header.stamp    = stamp;
        del.ns              = "cones_vis";
        del.id              = id;
        del.action          = visualization_msgs::Marker::DELETE;
        arr.markers.push_back(del);
    }

    // ======================================================
    // 2) Dodaję nowe markery
    // ======================================================
    const double fps = std::max(1e-3, P_.get("frames_per_second"));

    // Daję niewielki zapas ponad 1 okres ramki.
    // Dzięki temu marker nie miga przy małych jitterach czasowych.
    const ros::Duration lifetime(1.5 / fps);

    int id = 200;
    for (const auto& c : det.cones)
    {
        std_msgs::ColorRGBA col = camera_cone_color(c.color, 0.7f);

        visualization_msgs::Marker m;
        m.header.frame_id = "camera_base";
        m.header.stamp    = stamp;   // <-- najważniejsza poprawka
        m.ns              = "cones_vis";
        m.id              = id++;
        m.type            = visualization_msgs::Marker::CUBE;
        m.action          = visualization_msgs::Marker::ADD;

        m.pose.position.x = c.x;
        m.pose.position.y = c.y;
        m.pose.position.z = c.z + 0.15;
        m.pose.orientation.w = 1.0;

        m.scale.x = 0.30;
        m.scale.y = 0.30;
        m.scale.z = 0.30;

        m.color        = col;
        m.lifetime     = lifetime;
        m.frame_locked = false;

        arr.markers.push_back(std::move(m));
    }

    // ======================================================
    // 3) Zapamiętuję rozmiar bieżącej ramki
    // ======================================================
    last_frame_size_ = static_cast<int>(det.cones.size());

    pub_markers_cones_vis_.publish(arr);
}

void Simulation_lem_ros_node::publish_cones_lidar_markers_(
    const Track& det, const ros::Time& acquisition_stamp)
{
    if (!use_only_lidar && !use_fusion) return;

    if (!pub_markers_cones_lidar_) {
        ROS_ERROR("pub_markers_cones_lidar_ invalid; not publishing lidar/fusion markers");
        return;
    }

    visualization_msgs::MarkerArray arr;

    // Używam czasu akwizycji całej ramki lidaru.
    const ros::Time stamp = acquisition_stamp;

    // ======================================================
    // 1) Usuwam markery z poprzedniej ramki
    // ======================================================
    for (int id = 400; id < 400 + last_lidar_frame_size_; ++id)
    {
        visualization_msgs::Marker del;
        del.header.frame_id = "os_sensor";
        del.header.stamp    = stamp;
        del.ns              = "cones_lidar";
        del.id              = id;
        del.action          = visualization_msgs::Marker::DELETE;
        arr.markers.push_back(del);
    }

    // ======================================================
    // 2) Dodaję nowe markery
    // ======================================================
    const double fps = std::max(1e-3, P_.get("lidar_frames_per_second"));

    // Daję niewielki zapas ponad 1 okres ramki.
    const ros::Duration lifetime(1.5 / fps);

    int id = 400;
    for (const auto& c : det.cones)
    {
        std_msgs::ColorRGBA col = lidar_cone_color(c.color, 0.75f);

        visualization_msgs::Marker m;
        m.header.frame_id = "os_sensor";
        m.header.stamp    = stamp;   // <-- najważniejsza poprawka
        m.ns              = "cones_lidar";
        m.id              = id++;
        m.type            = visualization_msgs::Marker::SPHERE;
        m.action          = visualization_msgs::Marker::ADD;

        m.pose.position.x = c.x;
        m.pose.position.y = c.y;
        m.pose.position.z = c.z;
        m.pose.orientation.w = 1.0;

        m.scale.x = 0.22;
        m.scale.y = 0.22;
        m.scale.z = 0.22;

        m.color        = col;
        m.lifetime     = lifetime;
        m.frame_locked = false;

        arr.markers.push_back(std::move(m));
    }

    // ======================================================
    // 3) Zapamiętuję rozmiar bieżącej ramki
    // ======================================================
    last_lidar_frame_size_ = static_cast<int>(det.cones.size());

    pub_markers_cones_lidar_.publish(arr);
}

void Simulation_lem_ros_node::publish_bolid_tf_true() {
    geometry_msgs::TransformStamped tf_true;
    tf_true.header.stamp = ros::Time::now();
    tf_true.header.frame_id = "map";
    tf_true.child_frame_id  = "bolide_true";
    tf_true.transform.translation.x = state_.x;
    tf_true.transform.translation.y = state_.y;
    tf_true.transform.translation.z = 0.0;
    tf2::Quaternion q1; q1.setRPY(0, 0, state_.yaw);
    tf_true.transform.rotation.x = q1.x();
    tf_true.transform.rotation.y = q1.y();
    tf_true.transform.rotation.z = q1.z();
    tf_true.transform.rotation.w = q1.w();
    tf_br_.sendTransform(tf_true);
}

void Simulation_lem_ros_node::publish_estimated_vehicle_tf_(
    const StateEstimate& estimate)
{

    if (!pub_state_estimate_) {
        ROS_ERROR("State-estimate publisher is invalid");
        return;
    }

    geometry_msgs::TransformStamped estimate_tf;
    estimate_tf.header.stamp = ros::Time::now();
    estimate_tf.header.frame_id = "map";
    estimate_tf.child_frame_id  = "bolide_CoG";
    estimate_tf.transform.translation.x = estimate.x;
    estimate_tf.transform.translation.y = estimate.y;
    estimate_tf.transform.translation.z = 0.0;
    tf2::Quaternion q2;
    q2.setRPY(0, 0, estimate.yaw);
    estimate_tf.transform.rotation.x = q2.x();
    estimate_tf.transform.rotation.y = q2.y();
    estimate_tf.transform.rotation.z = q2.z();
    estimate_tf.transform.rotation.w = q2.w();
    tf_br_.sendTransform(estimate_tf);

}

void Simulation_lem_ros_node::publish_cones_gt_markers_()
{
    visualization_msgs::MarkerArray arr;

    if (!pub_markers_cones_gt_) {
        std::cerr << "[PUB][ERROR] pub_markers_cones_gt_ invalid; NOT publishing GT markers." << std::endl;
        ROS_ERROR("pub_markers_cones_gt_ invalid; not publishing GT markers");
        return;
    }

    // (opcjonalnie) wyczyść poprzednie markery od tego publishera
    {
        visualization_msgs::Marker del;
        del.header.frame_id = "map";
        del.header.stamp    = ros::Time::now();
        del.action = visualization_msgs::Marker::DELETEALL;
        arr.markers.push_back(del);
    }

    // lifetime = 0 → wieczny
    const ros::Duration kForever(0.0);

    int id = 300;
    for (const auto& c : track_global_.cones)
    {
        // kolor wg klasy (yellow/blue/orange/…)
        std_msgs::ColorRGBA col = ground_truth_cone_color(c.color, 0.95f);

        // bazowy “stożek” jako cylinder; funkcja ustawia ns="cones",
        // za chwilę nadpiszemy na "cones_gt" i frame na "map"
        visualization_msgs::Marker m = make_cone_marker(
            id++, /*frame*/ "map", c.x, c.y, c.z, col, kForever
        );

        // doprecyzowanie nagłówka/namespacu dla GT
        m.header.frame_id = "map";
        m.header.stamp    = ros::Time::now();
        m.ns = "cones_gt";                  // osobna przestrzeń nazw dla GT
        m.action = visualization_msgs::Marker::ADD;

        // (opcjonalnie) możesz różnić GT od wizji np. większą przezroczystością:
        // m.color.a = 0.7f;

        arr.markers.push_back(std::move(m));
    }

    pub_markers_cones_gt_.publish(arr);
}

void Simulation_lem_ros_node::pub_full_state_(){
    dv_interfaces::full_state msg;
    Log_Info_full info = log_info_full(
        state_,
        Input(
            active_main_torque_command_.fl,
            active_main_torque_command_.fr,
            active_main_torque_command_.rl,
            active_main_torque_command_.rr,
            rack_angle_command_
        ),
        P_,
        step_number_
    );

    msg.time = info.time;
    msg.step_number = step_number_;

    msg.x = info.x;
    msg.y = info.y;
    msg.yaw = info.yaw;
    msg.yaw_rate = info.yaw_rate;
    msg.vx = info.vx;

    msg.vy = info.vy;
    msg.ax = info.ax;
    msg.ay = info.ay;

    msg.torque = info.torque_total;
    msg.torque_left = info.torque_fl + info.torque_rl;
    msg.torque_right = info.torque_fr + info.torque_rr;
    msg.omega_rl = info.omega_rl;
    msg.omega_rr = info.omega_rr;

    msg.rack_angle = info.rack_angle;
    msg.delta_left = info.delta_left;
    msg.delta_rigth = info.delta_right;
    msg.rack_angle_request = info.rack_angle_request;

    msg.fx_fl = info.fx_fl;
    msg.fx_fr = info.fx_fr;
    msg.fx_rl = info.fx_rl;
    msg.fx_rr = info.fx_rr;

    msg.fy_fl = info.fy_fl;
    msg.fy_fr = info.fy_fr;
    msg.fy_rl = info.fy_rl;
    msg.fy_rr = info.fy_rr;

    msg.fz_fl = info.fz_fl;
    msg.fz_fr = info.fz_fr;
    msg.fz_rl = info.fz_rl;
    msg.fz_rr = info.fz_rr;

    msg.slip_angle_fl = info.slip_angle_fl;
    msg.slip_angle_fr = info.slip_angle_fr;
    msg.slip_angle_rl = info.slip_angle_rl;
    msg.slip_angle_rr = info.slip_angle_rr;
    msg.slip_angle_body = info.slip_angle_body;

    msg.kappa_fl = info.kappa_fl;
    msg.kappa_fr = info.kappa_fr;
    msg.kappa_rl = info.kappa_rl;
    msg.kappa_rr = info.kappa_rr;

    msg.total_drag = info.total_drag;
    msg.total_downforce = info.total_downforce;
    msg.Power_total = info.power_total;
    msg.torque_request = info.torque_request;

    msg.step_dt = P_.get("simulation_time_step");

    const double kappa_max_abs_signed =
    (std::abs(msg.kappa_rr) > std::abs(msg.kappa_rl)) ? msg.kappa_rr : msg.kappa_rl;

    update_top_abs(ten_biggest_slip_ratio_, kappa_max_abs_signed, 10);
    const double vehicle_speed_mps = std::hypot(msg.vx, msg.vy);
    if (vehicle_speed_mps >=
        P_.get("metrics_minimum_speed_mps_for_sideslip")) {
        time_sideslip_evaluated_s_ += msg.step_dt;
        update_top_abs(
            ten_biggest_beta_angle_, msg.slip_angle_body, 10);
        if (std::abs(msg.slip_angle_body) >
            P_.get("metrics_sideslip_threshold_rad")) {
            time_sideslip_over_threshold_s_ += msg.step_dt;
        }
    }

    pub_log_full_.publish(msg);

     // ==========================================================
    // [NOWE] RYSOWANIE KROPLI G-G (MARKER)
    // ==========================================================
    visualization_msgs::Marker gg_sphere;
    gg_sphere.header.frame_id = "gg_dashboard"; //
    gg_sphere.header.stamp = ros::Time::now();
    gg_sphere.ns = "gg_current_accel";
    gg_sphere.id = 1; // Musi być inny niż ID obwiedni!
    gg_sphere.type = visualization_msgs::Marker::SPHERE;
    gg_sphere.action = visualization_msgs::Marker::ADD;

    // Współrzędne na wykresie:
    // Oś X wykresu = Przyspieszenie boczne (ay)
    // Oś Y wykresu = Przyspieszenie wzdłużne (ax)
    gg_sphere.pose.position.x = info.ay;
    gg_sphere.pose.position.y = info.ax;
    gg_sphere.pose.position.z = 0.0;

    // Brak rotacji
    gg_sphere.pose.orientation.w = 1.0;

    // Rozmiar "kropli" (np. 15 cm średnicy na wykresie)
    gg_sphere.scale.x = 1.15;
    gg_sphere.scale.y = 1.15;
    gg_sphere.scale.z = 1.15; // Byłaby to kula, ale patrzymy z góry

    // Kolor - np. jaskrawy czerwony, żeby odcinał się od szarej obwiedni
    gg_sphere.color.r = 1.0;
    gg_sphere.color.g = 0.0;
    gg_sphere.color.b = 0.0;
    gg_sphere.color.a = 1.0;

    pub_gg_sphere_marker_.publish(gg_sphere);

}

void Simulation_lem_ros_node::publish_bolid_marker_()
{
    if (!pub_marker_bolid_) {
        ROS_ERROR("pub_marker_bolid_ invalid; not publishing bolid marker");
        return;
    }

    constexpr double kCarLengthM = 2.925;
    constexpr double kCarWidthM = 1.452;
    constexpr double kCarHeightM = 1.452;
    constexpr double kArrowBaseClearanceM = 0.05;

    const ros::Time stamp = ros::Time::now();

    visualization_msgs::Marker car;
    car.header.frame_id = "bolide_true";        // auto porusza się z TF pojazdu
    car.header.stamp    = stamp;
    car.ns   = "bolide";
    car.id   = 0;
    car.type = visualization_msgs::Marker::CUBE;
    car.action = visualization_msgs::Marker::ADD;

    // --- rozmiar pojazdu FS (około) ---
    car.scale.x = kCarLengthM;    // długość (m)
    car.scale.y = kCarWidthM;     // szerokość (m)
    car.scale.z = kCarHeightM;    // wysokość (m)

    // --- pozycja ---
    car.pose.position.x = 0.0;      // środek ciężkości = TF origin
    car.pose.position.y = 0.0;
    car.pose.position.z = 0.75;     // połowa wysokości, żeby stał na ziemi
    car.pose.orientation.w = 1.0;

    car.color.r = 0.0f;
    car.color.g = 0.9f;
    car.color.b = 0.2f;
    car.color.a = 1.0f;

    // --- lifetime krótkie, auto się odświeża ---
    car.lifetime = ros::Duration(0.1);

    pub_marker_bolid_.publish(car);

    // Vertical arrows visualize the instantaneous normal load at every wheel.
    // A load equal to the mean static wheel load produces a one-metre arrow.
    const double mean_static_wheel_load_n =
        P_.get("m") * P_.get("g") / 4.0;
    const double arrow_base_z_m =
        kCarHeightM + kArrowBaseClearanceM;

    struct WheelLoadMarker
    {
        int id;
        double x_m;
        double y_m;
        double normal_load_n;
    };

    const std::array<WheelLoadMarker, 4> wheel_loads{{
        {0,  P_.get("l_f"),  P_.get("t_front") / 2.0, state_.N_fl},
        {1,  P_.get("l_f"), -P_.get("t_front") / 2.0, state_.N_fr},
        {2, -P_.get("l_r"),  P_.get("t_rear") / 2.0, state_.N_rl},
        {3, -P_.get("l_r"), -P_.get("t_rear") / 2.0, state_.N_rr},
    }};

    for (const WheelLoadMarker& wheel : wheel_loads) {
        const double finite_load_n =
            std::isfinite(wheel.normal_load_n)
                ? std::max(0.0, wheel.normal_load_n)
                : 0.0;
        const double arrow_length_m =
            finite_load_n / mean_static_wheel_load_n;

        visualization_msgs::Marker arrow;
        arrow.header.frame_id = "bolide_true";
        arrow.header.stamp = stamp;
        arrow.ns = "wheel_normal_load";
        arrow.id = wheel.id;
        arrow.type = visualization_msgs::Marker::ARROW;
        arrow.action = visualization_msgs::Marker::ADD;

        geometry_msgs::Point start;
        start.x = wheel.x_m;
        start.y = wheel.y_m;
        start.z = arrow_base_z_m;

        geometry_msgs::Point end = start;
        end.z += arrow_length_m;

        arrow.points.push_back(start);
        arrow.points.push_back(end);

        // For a two-point ARROW marker these are shaft diameter, head
        // diameter and head length, respectively.
        arrow.scale.x = 0.06;
        arrow.scale.y = 0.14;
        arrow.scale.z = std::min(0.18, 0.4 * arrow_length_m);

        arrow.color.r = 1.0f;
        arrow.color.g = 0.25f;
        arrow.color.b = 0.05f;
        arrow.color.a = 1.0f;

        arrow.lifetime = ros::Duration(0.1);
        arrow.frame_locked = true;

        pub_marker_bolid_.publish(arrow);
    }
}

void Simulation_lem_ros_node::publish_dv_board_data_if_due_()
{
    if (!dv_board_publish_timer_.due(step_number_)) {
        return;
    }

    dv_interfaces::DV_board msg;

    /*
        wheel_speed_read_* is already the linear wheel velocity:

            velocity = omega * R   [m/s]

        The controller computes vehicle encoder speed as the average of the
        four fields, so do not multiply the values by 10 and do not average
        them here.
    */
    msg.velocity_RL = static_cast<float>(wheel_speed_read_rl);
    msg.velocity_RR = static_cast<float>(wheel_speed_read_rr);
    msg.velocity_FL = static_cast<float>(wheel_speed_read_fl);
    msg.velocity_FR = static_cast<float>(wheel_speed_read_fr);

    // Simulator does not currently model the DV mission/state machine here.
    msg.mission = 0U;
    msg.state = 0U;
    msg.cubemars_enable = false;

    pub_dv_board_data_.publish(msg);
}

void Simulation_lem_ros_node::main_read_imu_if_due_()
{
    if (!main_imu_timer_.due(step_number_) ||
        !published_imu_cache_.has_value()) {
        return;
    }
    main_imu_cache_.write(published_imu_cache_.value());
}

void Simulation_lem_ros_node::publish_steer_if_due_()
{
    if (!steering_publish_timer_.due(step_number_)) {
        return;
    }

    const double measured_steering_rad =
        state_.rack_angle +
        P_.get("steering_encoder_noise_std_rad") *
            random_noise_generator_();
    measured_steering_cache_.write(measured_steering_rad);

    std_msgs::Float64 steer_msg;
    steer_msg.data = measured_steering_rad;
    pub_steer_.publish(steer_msg);
}

void Simulation_lem_ros_node::log_metric_of_ride_data_()
{
    // Jeśli nie ma ścieżki, to nic nie zapisuję
    if (metrics_log_file_path_.empty()) return;

    std::ofstream f(metrics_log_file_path_, std::ios::out);
    if (!f.is_open())
    {
        ROS_WARN_STREAM("[METRICS] Cannot open metrics file: " << metrics_log_file_path_);
        return;
    }

    const double dt = P_.get("simulation_time_step");
    const double total_time = static_cast<double>(step_number_) * dt;
    const double sample_count =
        static_cast<double>(ride_metric_samples_);
    const double mean_abs_lateral_error_m =
        sample_count > 0.0
            ? absolute_lateral_error_sum_m_ / sample_count
            : 0.0;
    const double mean_abs_heading_error_rad =
        sample_count > 0.0
            ? absolute_heading_error_sum_rad_ / sample_count
            : 0.0;
    const double mean_path_speed_mps =
        sample_count > 0.0
            ? path_speed_sum_mps_ / sample_count
            : 0.0;

    if (time_sideslip_evaluated_s_ > 1.0e-9) {
        percentage_time_sideslip_over_threshold_ =
            100.0 * time_sideslip_over_threshold_s_ /
            time_sideslip_evaluated_s_;
    } else {
        percentage_time_sideslip_over_threshold_ = 0.0;
    }

    // helper do wektorów -> "a;b;c;d"
    auto join_vec = [](const std::vector<double>& v) -> std::string
    {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(6);
        for (size_t i = 0; i < v.size(); ++i)
        {
            if (i) oss << ";";
            oss << v[i];
        }
        return oss.str();
    };

    // CSV: proste "metric,value" - łatwe do parsowania i czytania
    f << "metric,value\n";

    f << "total_time_s," << total_time << "\n";
    f << "mean_abs_lateral_error_m,"
      << mean_abs_lateral_error_m << "\n";
    f << "mean_abs_heading_error_rad,"
      << mean_abs_heading_error_rad << "\n";
    f << "mean_path_speed_mps,"
      << mean_path_speed_mps << "\n";
    f << "ride_metric_samples,"
      << ride_metric_samples_ << "\n";

    f << "sideslip_threshold_rad,"
      << P_.get("metrics_sideslip_threshold_rad") << "\n";
    f << "sideslip_minimum_speed_mps,"
      << P_.get("metrics_minimum_speed_mps_for_sideslip") << "\n";
    f << "sideslip_evaluated_time_s,"
      << time_sideslip_evaluated_s_ << "\n";
    f << "sideslip_over_threshold_time_s,"
      << time_sideslip_over_threshold_s_ << "\n";
    f << "sideslip_over_threshold_percent,"
      << percentage_time_sideslip_over_threshold_ << "\n";

    // Top 10 listy jako jedna komórka (bezpieczne i wygodne)
    f << "top_10_driven_wheel_slip_ratio,"
      << "\"" << join_vec(ten_biggest_slip_ratio_) << "\"\n";
    f << "top_10_body_sideslip_rad,"
      << "\"" << join_vec(ten_biggest_beta_angle_) << "\"\n";
    f << "top_10_lateral_error_m,"
      << "\"" << join_vec(ten_biggest_ey_) << "\"\n";
    f << "top_10_heading_error_rad,"
      << "\"" << join_vec(ten_biggest_epsi_) << "\"\n";

    f.flush();
    f.close();

    ROS_WARN_STREAM("[METRICS] Saved ride metrics to: " << metrics_log_file_path_);
}

void Simulation_lem_ros_node::publish_steer_actuator_status_()
{
    if (!steering_publish_timer_.due(step_number_)) {
        return;
    }

    std_msgs::Bool status_msg;
    status_msg.data = true;
    pub_steer_status_.publish(status_msg);
}
} // namespace lem_dynamics_sim_
