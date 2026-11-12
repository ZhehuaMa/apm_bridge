#include "custom_link_vehicle_node.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include "custom_link/sitl_transport.h"

using namespace custom_link_bridge;
using namespace std::chrono_literals;

namespace
{
    constexpr double kDegToRad = M_PI / 180.0;
    constexpr double kRadToDeg = 180.0 / M_PI;
    constexpr int kAbsoluteZeroCdeg = -27315;
    // 视觉锁定迟滞宽限: tracker "Target lost -> Re-initializing" 空档实测 ~0.3-0.7 s
    // (nanotrack 5 帧惯性耗尽到重初始化), 留 1.0 s 覆盖重捕周期。任务监督层另有
    // "丢失 2 s 结束交战" 兜底, 短宽限不会放大陈旧锁定的风险。
    constexpr double TRACKING_HOLDOVER_S = 1.0;

    static double normalizeAngle(double a)
    {
        while (a > M_PI)
            a -= 2.0 * M_PI;
        while (a <= -M_PI)
            a += 2.0 * M_PI;
        return a;
    }

    // ATTITUDE 指令 → 角度模式设定点（FLU）：roll/pitch 取姿态四元数的欧拉角（rad），
    // yaw 沿用 bodyrates.z 角速度（rad/s，mode_req=3 的 yaw 通道仍是 deg/s）。
    // 四元数未归一化或含非有限值时返回 false，整条指令丢弃（不回落角速度通道）。
    static bool attitudeSetpoints(const quadrotor_msgs::msg::ControlCommand &command, std::array<double, 3> &setpoints)
    {
        const auto &q = command.orientation;
        const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
        if (!std::isfinite(norm) || std::abs(norm - 1.0) > 0.01)
            return false;
        double roll, pitch, yaw;
        tf2::Matrix3x3(tf2::Quaternion(q.x, q.y, q.z, q.w)).getRPY(roll, pitch, yaw);
        (void)yaw; // 偏航走 bodyrates.z 角速度，yaw 欧拉角不参与设定点
        setpoints = {roll, pitch, command.bodyrates.z};
        return std::all_of(setpoints.begin(), setpoints.end(), [](double v) { return std::isfinite(v); });
    }

} // namespace

CustomLinkVehicleNode::CustomLinkVehicleNode()
    : Node("custom_link_bridge")
{
    // ---------------- 参数 ----------------
    this->declare_parameter<double>("gravity_const", 9.80665);
    this->declare_parameter<double>("mass", 2.0);
    this->declare_parameter<int>("lipo_cells", 6);
    this->declare_parameter<double>("battery.capacity_mah", 13000.0); // 整包容量
    this->declare_parameter<int>("channel_trigger", 6);
    this->declare_parameter<bool>("voltage_compensation", false);
    this->declare_parameter<bool>("baro_compensation", true);
    // fpv_test.md 模型 F_std(u) = a3·u³ + a2·u² + a1·u (u = DShot 占空比), thrust_test_node 标定
    this->declare_parameter<double>("motor_parameters.a3", 0.0);
    this->declare_parameter<double>("motor_parameters.a2", 0.0);
    this->declare_parameter<double>("motor_parameters.a1", 0.0);
    this->declare_parameter<double>("motor_parameters.r_eq", 0.0);
    this->declare_parameter<double>("motor_parameters.k_i", 0.0);
    // I_std(F_std) 三项电流模型的二次/三次项 (2026-11-12 修订; 0 = 线性, 兼容旧 YAML)
    this->declare_parameter<double>("motor_parameters.k_i2", 0.0);
    this->declare_parameter<double>("motor_parameters.k_i3", 0.0);
    this->declare_parameter<double>("voltage_lpf_hz", 3.0);
    airsim_enabled = this->declare_parameter<bool>("airsim.enabled", false);

    this->declare_parameter<std::string>("link.transport", "serial"); // serial | tcp
    this->declare_parameter<std::string>("link.serial_port", "/dev/ttyACM0");
    this->declare_parameter<int>("link.baudrate", 921600);
    this->declare_parameter<std::string>("link.tcp_host", "127.0.0.1");
    this->declare_parameter<int>("link.tcp_port", 5763); // SITL: UART3 = 5760 + uart_index
    this->declare_parameter<double>("link.control_rate", 100.0);
    this->declare_parameter<double>("link.sync_rate", 1.0);
    this->declare_parameter<double>("link.diag_rate", 0.2);
    this->declare_parameter<double>("link.rate_limit_dps", 0.0);
    this->declare_parameter<double>("link.angle_limit_deg", 0.0);
    this->declare_parameter<double>("cli.timeout_s", 3.0); // /fpv/cli 响应超时
    this->declare_parameter<bool>("cli.bind_rx_on_connect", false);

    this->declare_parameter<std::string>("imu.frame_id", "base_link");
    this->declare_parameter<double>("imu.orientation_stddev", 0.0);
    this->declare_parameter<double>("imu.angular_velocity_stddev", 0.0);
    this->declare_parameter<double>("imu.linear_acceleration_stddev", 0.0);

    this->declare_parameter<double>("rc.control_timeout", 0.5);
    this->declare_parameter<double>("landed.airborne_altitude", 0.5);
    this->declare_parameter<double>("landed.landing_altitude", 0.2);
    this->declare_parameter<double>("landed.takeoff_confirm_time", 0.5);
    this->declare_parameter<double>("landed.landing_confirm_time", 1.0);

    this->get_parameter("gravity_const", gravity);
    this->get_parameter("mass", mass);
    this->get_parameter("lipo_cells", n_lipo_cells);
    this->get_parameter("battery.capacity_mah", battery_capacity_mah);
    this->get_parameter("channel_trigger", channel_trigger);
    this->get_parameter("voltage_compensation", voltage_compensation);
    this->get_parameter("baro_compensation", baro_compensation);
    this->get_parameter("cli.timeout_s", cli_timeout);
    this->get_parameter("cli.timeout_s", cli_timeout);
    this->get_parameter("cli.bind_rx_on_connect", cli_bind_rx_on_connect);

    // 旧版 A..D/spin_k/vbat 模型已按 fpv_test.md 重建, 键名不兼容: 显式拒绝, 避免
    // 旧参数文件被静默忽略后以未标定模型运行。u_idle 不再是参数: 链路建立后由飞控
    // CLI "get motor_idle" 读取 (与飞控实际怠速一致)。
    static const std::set<std::string> kModelKeys{
        "motor_parameters.a3", "motor_parameters.a2", "motor_parameters.a1",
        "motor_parameters.r_eq", "motor_parameters.k_i", "motor_parameters.k_i2",
        "motor_parameters.k_i3"};
    for (const auto &[name, value] : this->get_node_parameters_interface()->get_parameter_overrides())
    {
        (void)value;
        if (name == "motor_parameters.u_idle" || name == "motor_parameters.motor_idle")
            throw std::invalid_argument("Unsupported parameter '" + name +
                                        "': u_idle is read from the FC (CLI get motor_idle); remove it");
        if (name.rfind("motor_parameters.", 0) == 0 && !kModelKeys.count(name))
            throw std::invalid_argument("Unsupported parameter '" + name +
                                        "': motor_parameters now follow fpv_test.md "
                                        "(a3, a2, a1, r_eq, k_i, k_i2, k_i3); "
                                        "recalibrate with thrust_test_node");
    }
    this->get_parameter("motor_parameters.a3", propulsion.a3);
    this->get_parameter("motor_parameters.a2", propulsion.a2);
    this->get_parameter("motor_parameters.a1", propulsion.a1);
    this->get_parameter("motor_parameters.r_eq", propulsion.r_eq);
    this->get_parameter("motor_parameters.k_i", propulsion.k_i);
    this->get_parameter("motor_parameters.k_i2", propulsion.k_i2);
    this->get_parameter("motor_parameters.k_i3", propulsion.k_i3);
    this->get_parameter("voltage_lpf_hz", voltage_lpf_hz);
    propulsion.v_ref = n_lipo_cells * kBatteryNominalVoltagePerCell;     // 6S: 22.20 V
    propulsion.v_min = n_lipo_cells * kBatteryUndervoltagePerCell;       // 6S: 12.0 V
    // §5: 滤波器初值取基准电压; 电流在首个遥测样本处初始化
    voltage_lpf.cutoff_hz = voltage_lpf_hz;
    voltage_lpf.value = propulsion.v_ref;
    current_lpf.cutoff_hz = voltage_lpf_hz;

    double ori_stddev = 0.0, gyro_stddev = 0.0, acc_stddev = 0.0;
    this->get_parameter("imu.orientation_stddev", ori_stddev);
    this->get_parameter("imu.angular_velocity_stddev", gyro_stddev);
    this->get_parameter("imu.linear_acceleration_stddev", acc_stddev);
    orientation_variance = ori_stddev * ori_stddev;
    angular_velocity_variance = gyro_stddev * gyro_stddev;
    linear_acceleration_variance = acc_stddev * acc_stddev;
    this->get_parameter("imu.frame_id", imu_frame_id);
    this->get_parameter("rc.control_timeout", control_timeout);
    this->get_parameter("landed.airborne_altitude", airborne_altitude);
    this->get_parameter("landed.landing_altitude", landing_altitude);
    this->get_parameter("landed.takeoff_confirm_time", takeoff_confirm_time_s);
    this->get_parameter("landed.landing_confirm_time", landing_confirm_time_s);
    if (!std::isfinite(airborne_altitude) || !std::isfinite(landing_altitude) ||
        !std::isfinite(takeoff_confirm_time_s) || !std::isfinite(landing_confirm_time_s) ||
        airborne_altitude <= 0.0 || landing_altitude < 0.0 || landing_altitude >= airborne_altitude ||
        takeoff_confirm_time_s <= 0.0 || landing_confirm_time_s <= 0.0)
        throw std::invalid_argument("Invalid landed-state detection parameters");
    if (airsim_enabled && (voltage_compensation || baro_compensation))
        throw std::invalid_argument("AirSim SITL requires voltage_compensation=false and baro_compensation=false");
    if (airsim_enabled &&
        (!std::isfinite(mass) || mass <= 0 || !std::isfinite(gravity) || gravity <= 0 ||
         !std::isfinite(control_timeout) || control_timeout <= 0))
        throw std::invalid_argument("Invalid SITL mass, gravity or timeout");
    if (!std::isfinite(voltage_lpf_hz) || voltage_lpf_hz < 2.0 || voltage_lpf_hz > 5.0)
        throw std::invalid_argument("voltage_lpf_hz must be within 2..5 Hz (fpv_test.md section 4)");
    // a3..a1 全零 = 未标定: 只允许 raw 油门通道 (台架标定工具即走 raw); 已标定模型须
    // 通过 §3.2 合法性校验。曲线判据 (dF/du > 0 on [u_idle,1]) 的窗口依赖 FC
    // motor_idle, 此刻 u_idle 只是占位值: 只硬性校验与 u_idle 无关的部分, 曲线判据
    // 延后到 queryMotorIdle 用真实 u_idle 复检 (通过前力指令被 propulsion_valid 拒绝)。
    if (quadratic_thrust_model::configured(propulsion))
    {
        const std::string param_error = quadratic_thrust_model::validateParameters(propulsion);
        if (!param_error.empty())
            throw std::invalid_argument("Invalid motor_parameters: " + param_error);
        const std::string curve_error = quadratic_thrust_model::validateModel(propulsion);
        if (curve_error.empty())
            RCLCPP_INFO(this->get_logger(),
                        "Propulsion model: F_std(u) = %.6g u^3 + %.6g u^2 + %.6g u N (F(1)=%.3f N), "
                        "V_ref=%.2f V, R_eq=%.4g ohm, I_std(F) = ((%.4g F + %.4g) F + %.4g) F A, "
                        "V_min=%.1f V, LPF %.1f Hz; u_idle pending FC motor_idle (CLI)",
                        propulsion.a3, propulsion.a2, propulsion.a1,
                        quadratic_thrust_model::standardThrust(propulsion, 1.0), propulsion.v_ref,
                        propulsion.r_eq, propulsion.k_i3, propulsion.k_i2, propulsion.k_i,
                        propulsion.v_min, voltage_lpf_hz);
        else
            RCLCPP_WARN(this->get_logger(),
                        "motor_parameters: \"%s\" at the provisional u_idle=%.4f; re-validated "
                        "once the FC motor_idle is read (force commands rejected until it passes)",
                        curve_error.c_str(), propulsion.u_idle);
    }
    else
        RCLCPP_WARN(this->get_logger(),
                    "motor_parameters a3..a1 are all zero (uncalibrated): ~/control_command force commands are "
                    "rejected; only ~/control_command_raw works until thrust_test_node calibration");

    // ---------------- ROS 接口 ----------------
    imu_pub = this->create_publisher<sensor_msgs::msg::Imu>(
        "/mavros/imu/data", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort());
    odom_pub = this->create_publisher<nav_msgs::msg::Odometry>(
        "/fpv/odom", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort());
    rc_in_pub = this->create_publisher<mavros_msgs::msg::RCIn>(
        "/mavros/rc/in", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort());
    state_pub = this->create_publisher<mavros_msgs::msg::State>("/mavros/state", 10);
    ext_state_pub = this->create_publisher<mavros_msgs::msg::ExtendedState>("/mavros/extended_state", 10);
    battery_pub = this->create_publisher<sensor_msgs::msg::BatteryState>("/mavros/battery", 10);
    // 环境气压/温度 (0x11 携带)：对外统一 /fpv 话题，供台架标定工具做 fpv_test.md §3.1 环境归一化
    pressure_pub = this->create_publisher<sensor_msgs::msg::FluidPressure>(
        "/fpv/static_pressure", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort());
    temperature_pub = this->create_publisher<sensor_msgs::msg::Temperature>(
        "/fpv/temperature_baro", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort());
    ap_feedback_pub = this->create_publisher<quadrotor_msgs::msg::LowLevelFeedback>("~/low_level_feedback", 10);
    gimbal_imu_pub = this->create_publisher<sensor_msgs::msg::Imu>("/fpv/gimbal", 10);
    gimbal_mode_pub = this->create_publisher<std_msgs::msg::String>(
        "/fpv/gimbal_mode", rclcpp::QoS(1).transient_local());
    trigger_pub = this->create_publisher<std_msgs::msg::Header>("/fpv/tracker_trigger", 10);
    altitude_pub = this->create_publisher<std_msgs::msg::Float64>("/fpv/altitude", rclcpp::SensorDataQoS());
    // GPS / 失效保护 (0x12 PayloadSlow, 10 Hz)：沿用 MAVROS 话题名，OSD 与
    // 其他节点可直接复用 mavros 消费习惯
    gpsraw_pub = this->create_publisher<mavros_msgs::msg::GPSRAW>(
        "/mavros/gpsstatus/gpsraw", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort());
    navsat_pub = this->create_publisher<sensor_msgs::msg::NavSatFix>(
        "/mavros/global_position/global", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort());
    failsafe_pub = this->create_publisher<std_msgs::msg::Bool>("/fpv/failsafe", 10);
    // ESC 转速 (0x16 PayloadEscTelemetry, 100 Hz)：data[i] = 电机 i 机械 rpm
    motor_rpm_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>("/fpv/motor_rpm", 10);

    control_command_sub = this->create_subscription<quadrotor_msgs::msg::ControlCommand>(
        "~/control_command", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
        std::bind(&CustomLinkVehicleNode::ctrlCommandCallback, this, std::placeholders::_1));
    control_command_raw_sub = this->create_subscription<quadrotor_msgs::msg::ControlCommand>(
        "~/control_command_raw", rclcpp::QoS(rclcpp::KeepLast(5)).best_effort(),
        std::bind(&CustomLinkVehicleNode::ctrlCommandRawCallback, this, std::placeholders::_1));
    arm_interface_sub = this->create_subscription<std_msgs::msg::Bool>(
        "~/arm", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
        std::bind(&CustomLinkVehicleNode::armCallback, this, std::placeholders::_1));
    offboard_interface_sub = this->create_subscription<std_msgs::msg::Bool>(
        "~/offboard", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
        std::bind(&CustomLinkVehicleNode::offboardCallback, this, std::placeholders::_1));

    tracker_state_sub = this->create_subscription<std_msgs::msg::Int8>(
        "/fpv/tracker_state", 1,
        [&](const std_msgs::msg::Int8::SharedPtr msg) {
            const bool locked = (msg->data == 2); // TRACKING
            in_tracking = locked;
            // 迟滞锚点: 记住最近一次锁定时刻, 供 MISSION 脉冲判定的宽限窗口使用
            // (见 rcTriggerUpdate 的 TRACKING_HOLDOVER_S)。
            if (locked)
                tracking_hold_s = this->now().seconds();
        });
    radar_state_sub = this->create_subscription<std_msgs::msg::Int8>(
        "/fpv/radar_state", 1,
        [&](const std_msgs::msg::Int8::SharedPtr msg) {
            radar_tracking = (msg->data == 2); // TRACKING
        });
    mission_state_sub = this->create_subscription<std_msgs::msg::Bool>(
        "/fpv/mission_state", 1,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            // 拦截器结束任务（True→False）同样联动退出 OFFBOARD（与 OFF 触发帧双保险）。
            if (in_mission.exchange(msg->data) && !msg->data && client)
            {
                client->setOffboardRequest(false);
                RCLCPP_INFO(this->get_logger(), "Mission ended: requesting custom-link MANUAL");
            }
        });

    mission_gimbal_on_ground = this->declare_parameter<bool>("gimbal.mission_on_ground", false);
    gimbal = apm_bridge::createGimbal(*this, [publisher = gimbal_imu_pub](auto msg) {
        publisher->publish(std::move(msg));
    });
    // 初始模式在 gimbal->start() 之前发布, 确保排在回中完成之前: 回中极快
    // (云台已在零点) 时首帧也必须是 "HOMING" -> "AIM", 而不是直接 "AIM"。
    gimbal_homing = gimbal && !gimbal->homed();
    {
        std_msgs::msg::String initial_mode;
        initial_mode.data = gimbal_homing ? "HOMING" : "AIM";
        gimbal_mode_pub->publish(initial_mode);
    }
    if (gimbal)
    {
        // 启动即连云台: home_on_connect 后端 (串口云台) 先回中机械零点,
        // homed() 置位后才由 updateGimbalMode 进 AIM; 无回中语义的后端
        // (捷联/AirSim) homed() 恒真, 维持启动即 AIM。
        if (gimbal->homed())
            gimbal->setMode(apm_bridge::GimbalMode::AIM);
        gimbal->start();
    }
    if (!this->has_parameter("gimbal.los_timeout"))
        this->declare_parameter<double>("gimbal.los_timeout", 0.3);
    target_los_sub = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "/fpv/target_los", rclcpp::QoS(1).best_effort(),
        [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
            if (!gimbal)
                return;
            apm_bridge::TargetLos target;
            const auto &angles = msg->twist.linear;
            const auto &rates = msg->twist.angular;
            const int64_t stamp = rclcpp::Time(msg->header.stamp).nanoseconds();
            const int64_t now = this->now().nanoseconds();
            if (now < last_los_clock)
            {
                last_los_stamp = 0;
                RCLCPP_WARN(this->get_logger(), "ROS clock moved backwards; resetting target LOS sequence");
            }
            last_los_clock = now;
            const double age = (now - stamp) * 1e-9;
            target.valid = msg->header.frame_id == "camera_flu" &&
                std::isfinite(angles.y) && std::isfinite(angles.z) &&
                std::isfinite(rates.y) && std::isfinite(rates.z) &&
                std::abs(angles.y) < M_PI_2 - 1e-6 && std::abs(angles.z) < M_PI_2 - 1e-6 &&
                age >= 0 && age <= this->get_parameter("gimbal.los_timeout").as_double() &&
                stamp > last_los_stamp;
            if (target.valid)
            {
                last_los_stamp = stamp;
                target.pitch = angles.y;
                target.yaw = angles.z;
                target.pitch_rate = rates.y;
                target.yaw_rate = rates.z;
                target.age = age;
            }
            else
            {
                if (!msg->header.frame_id.empty())
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                        "Rejecting target LOS: invalid frame, values, stale or non-increasing timestamp");
            }
            gimbal->updateTargetLos(target);
        });
    gimbal_trigger_sub = this->create_subscription<std_msgs::msg::Header>(
        "/fpv/tracker_trigger", 10, [this](const std_msgs::msg::Header::SharedPtr msg) {
            if (msg->frame_id == "ON")
                gimbal_aim_requested = true;
            else if (msg->frame_id == "OFF" || msg->frame_id == "DISMISS")
            {
                gimbal_abort_requested = true;
                // 退出 MISSION 联动：退出 OFFBOARD（modeReq=0），杆量权限即刻交还飞手。
                if (client)
                {
                    client->setOffboardRequest(false);
                    RCLCPP_INFO(this->get_logger(), "Tracker OFF: requesting custom-link MANUAL");
                }
            }
            else if (msg->frame_id == "MISSION")
            {
                // 进入 MISSION 联动：请求飞控进入 OFFBOARD（modeReq=2，HOST_CONTROL
                // 帧随控制流携带），拦截弹控制帧即刻获得权限，不再依赖 AUX2 开关。
                if (client)
                {
                    client->setOffboardRequest(true);
                    RCLCPP_INFO(this->get_logger(), "MISSION: requesting custom-link OFFBOARD");
                }
            }
        });
    gimbal_mode_timer = this->create_wall_timer(20ms, [this]() { updateGimbalMode(); });

    // ---------------- 链路 ----------------
    using custom_link::Client;
    using custom_link::SerialTransport;
    using custom_link::TcpTransport;

    std::string transport_kind, serial_port, tcp_host;
    int baudrate = 921600, tcp_port = 5763;
    double control_rate = 100.0, sync_rate = 1.0, diag_rate = 0.2;
    this->get_parameter("link.transport", transport_kind);
    this->get_parameter("link.serial_port", serial_port);
    this->get_parameter("link.baudrate", baudrate);
    this->get_parameter("link.tcp_host", tcp_host);
    this->get_parameter("link.tcp_port", tcp_port);
    this->get_parameter("link.control_rate", control_rate);
    this->get_parameter("link.sync_rate", sync_rate);
    this->get_parameter("link.diag_rate", diag_rate);
    this->get_parameter("link.rate_limit_dps", rate_limit_dps);
    this->get_parameter("link.angle_limit_deg", angle_limit_deg);
    const auto control_period = custom_link::periodFromRate(control_rate);

    std::unique_ptr<custom_link::Transport> transport;
    if (airsim_enabled)
    {
        if (transport_kind != "tcp")
            throw std::invalid_argument("AirSim SITL requires link.transport=tcp");
        const auto bind_host = this->declare_parameter<std::string>("airsim.link_bind_host", "127.0.0.1");
        const int bind_port = this->declare_parameter<int>("airsim.link_port", 5764);
        if (bind_port <= 0 || bind_port > 65535 || tcp_port <= 0 || tcp_port > 65535 || bind_port == tcp_port)
            throw std::invalid_argument("AirSim relay and FC TCP ports must be valid and distinct");
        transport = std::make_unique<custom_link::SitlTransport>(
            tcp_host, static_cast<uint16_t>(tcp_port), bind_host, static_cast<uint16_t>(bind_port), this->get_logger());
    }
    else if (transport_kind == "tcp")
    {
        RCLCPP_INFO(this->get_logger(), "Custom link over TCP %s:%d", tcp_host.c_str(), tcp_port);
        transport = std::make_unique<TcpTransport>(tcp_host, static_cast<uint16_t>(tcp_port));
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Custom link over serial %s @ %d", serial_port.c_str(), baudrate);
        transport = std::make_unique<SerialTransport>(serial_port, static_cast<uint32_t>(baudrate));
    }

    client = std::make_unique<Client>(std::move(transport));
    client->setRates(control_rate, sync_rate);

    client->setLinkStateCallback([this](bool up) {
        if (up)
        {
            RCLCPP_INFO(this->get_logger(), "Custom link up");
            // cli.bind_rx_on_connect: 每次链路建立成功后让接收机进配对模式
            //（0x24 CL_CLI_CMD_BIND_RX，同 CLI bind_rx）。请求由 CLI 工作线程
            // 执行，这里只置标志并唤醒。持锁置位避免丢唤醒。
            // 同时 (重新) 读取飞控 motor_idle 作为 u_idle: 飞控可能已重启/改过配置。
            {
                std::lock_guard<std::mutex> lk(cli_job_mtx);
                if (cli_bind_rx_on_connect)
                    rx_bind_pending = true;
                scheduleIdleQuery(std::chrono::steady_clock::now());
            }
            cli_job_cv.notify_one();
        }
        else
            RCLCPP_WARN(this->get_logger(), "Custom link down");
    });

    client->setTelemetryCallback(
        [this](const custom_link::protocol::Frame &frame, uint32_t fcUs, int64_t hostUs) {
            using namespace custom_link::protocol;
            switch (frame.msgId)
            {
            case MSG_FC_FAST:
            {
                PayloadFast p;
                if (frame.as(p))
                    handleFast(p, hostUs);
                break;
            }
            case MSG_FC_MEDIUM:
            {
                PayloadMedium p;
                if (frame.as(p))
                    handleMedium(p, hostUs);
                break;
            }
            case MSG_FC_SLOW:
            {
                // 0x12 允许尾部扩展 (固件在 36B 基础字段后追加了诊断计数器,
                // 同 0x11 追加 rssi_pct 的向前兼容手法): 按前缀解包, 忽略尾部。
                PayloadSlow p;
                if (frame.len >= sizeof(PayloadSlow)) {
                    memcpy(&p, frame.payload.data(), sizeof(PayloadSlow));
                    handleSlow(p, hostUs);
                }
                break;
            }
            case MSG_FC_ODOMETRY:
            {
                PayloadOdometry p;
                if (frame.as(p))
                    handleOdometry(p, hostUs);
                else
                    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                         "Invalid custom-link odometry payload size: %u", frame.len);
                break;
            }
            case MSG_FC_ESC_TELEMETRY:
            {
                PayloadEscTelemetry p;
                if (frame.as(p))
                    handleEscTelemetry(p, hostUs);
                else
                    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                         "Invalid custom-link ESC telemetry payload size: %u", frame.len);
                break;
            }
            default:
                break;
            }
            (void)fcUs;
        });

    client->start();

    // CLI 设置读写服务：延迟响应回调（header+request），工作线程阻塞等
    // 链路响应后 send_response 回包。必须在 client 就绪后创建（工作线程
    // 直接使用 client），也保证析构时先停工作线程再停链路。
    cli_service = this->create_service<quadrotor_msgs::srv::CliCommand>(
        "/fpv/cli",
        [this](const std::shared_ptr<rmw_request_id_t> header,
               const std::shared_ptr<quadrotor_msgs::srv::CliCommand::Request> request) {
            cliServiceCallback(header, request);
        });
    cli_worker_running = true;
    cli_worker = std::thread(&CustomLinkVehicleNode::cliWorkerLoop, this);

    // 控制量推送：回调只写缓存，这里统一判新鲜度并转换体轴后下发。
    // 超时后只作废控制量，arm 请求不动（链路恢复后自动重试解锁边沿）。
    cmd_push_timer = this->create_wall_timer(control_period, [this]() { pushControl(); });

    diag_timer = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / std::max(0.05, diag_rate))), [this]() {
            const auto sync = client->timeSync();
            const auto stats = client->stats();
            if (!sync.valid)
                return;
            RCLCPP_INFO(this->get_logger(),
                        "Custom link: rtt %.2f ms (jitter %.2f), skew %+.0f ppm, FC offset %+.2f ms, "
                        "rx %lu (F:%u M:%u S:%u O:%u), tx %lu, reconnects %lu",
                        sync.rtt_ms, sync.jitter_ms, sync.skew_ppm, sync.offset_ms,
                        static_cast<unsigned long>(stats.rx_frames),
                        stats.rx_per_msg[custom_link::protocol::MSG_FC_FAST & 7u],
                        stats.rx_per_msg[custom_link::protocol::MSG_FC_MEDIUM & 7u],
                        stats.rx_per_msg[custom_link::protocol::MSG_FC_SLOW & 7u],
                        stats.rx_per_msg[custom_link::protocol::MSG_FC_ODOMETRY & 7u],
                        static_cast<unsigned long>(stats.tx_frames),
                        static_cast<unsigned long>(stats.reconnects));
        });

}

CustomLinkVehicleNode::~CustomLinkVehicleNode()
{
    {
        std::lock_guard<std::mutex> lk(cli_job_mtx);
        cli_worker_running = false;
    }
    cli_job_cv.notify_all();
    if (cli_worker.joinable())
        cli_worker.join();
    if (gimbal)
        gimbal->stop();
    if (client)
        client->stop();
}

// ======================================================================
// 遥测（读取线程上下文）
// ======================================================================

void CustomLinkVehicleNode::handleFast(const custom_link::protocol::PayloadFast &p, int64_t hostUs)
{
    // Betaflight 原生传感器与 ROS FLU 完全同轴同号，全部直通
    //（SITL 刚体闭环实证 src/test/sitl/sitl_custom_link_frames_test.py，
    // 固件三源互证 mavlink.c/sitl_gyro.h/pid.c pidLevel）：
    //   acc   = FLU（x 前、y 左、z 上，水平 z=+1g）
    //   gyro  = FLU 角速度（roll 右滚+、pitch 低头+ 即 FLU +y、yaw 逆时针+）
    //   姿态  = 四元数 w,x,y,z ×16384，FLU 体 -> ENU 世（固件端已完成
    //           +pi/2 参考旋转），解码归一化后直通发布为 Imu orientation。
    //           四元数在 ±90° 俯仰（垂直发射/过顶）连续；旧欧拉传输在该处
    //           折返（pitch 变号、roll 翻 180°）——2026-11 姿态符号偶发
    //           反向的根因，协议已彻底移除欧拉姿态。
    const double ax = p.acc[0] * 0.001 * gravity;
    const double ay = p.acc[1] * 0.001 * gravity;
    const double az = p.acc[2] * 0.001 * gravity;
    const double gx = p.gyro[0] * 0.1 * kDegToRad;
    const double gy = p.gyro[1] * 0.1 * kDegToRad;
    const double gz = p.gyro[2] * 0.1 * kDegToRad;

    // CRC 之后仍做一次廉价的四元数健全性检查（非有限/范数偏离即拒帧，
    // 保留上一帧姿态）。×16384 量化误差 ~1e-4，0.1 门限极宽松。
    std::array<double, 4> q;
    double norm2 = 0.0;
    bool finite = true;
    for (int i = 0; i < 4; ++i)
    {
        q[i] = p.quat[i] / 16384.0;
        finite = finite && std::isfinite(q[i]);
        norm2 += q[i] * q[i];
    }
    if (!finite || std::fabs(norm2 - 1.0) > 0.1)
    {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Invalid custom-link attitude quaternion (norm2=%.4f)", norm2);
        return;
    }
    const double inv_norm = 1.0 / std::sqrt(norm2);
    for (double &v : q)
        v *= inv_norm;

    {
        std::lock_guard<std::mutex> lk(mtx_att);
        accel_flu = {ax, ay, az};
        gyro_flu = {gx, gy, gz};
        quat_flu_enu = q;   // [w, x, y, z]，FLU->ENU，已归一化
        att_valid = true;
    }
    publishImu(hostUs);
}

void CustomLinkVehicleNode::publishImu(int64_t hostUs)
{
    sensor_msgs::msg::Imu msg;
    tf2::Quaternion q;
    {
        std::lock_guard<std::mutex> lk(mtx_att);
        if (!att_valid)
            return;
        // tf2 构造参数序为 (x, y, z, w)；内部缓存为 [w, x, y, z]
        q = tf2::Quaternion(quat_flu_enu[1], quat_flu_enu[2], quat_flu_enu[3], quat_flu_enu[0]);
        msg.angular_velocity.x = gyro_flu[0];
        msg.angular_velocity.y = gyro_flu[1];
        msg.angular_velocity.z = gyro_flu[2];
        msg.linear_acceleration.x = accel_flu[0];
        msg.linear_acceleration.y = accel_flu[1];
        msg.linear_acceleration.z = accel_flu[2];
    }
    msg.header.stamp = rclcpp::Time(static_cast<int64_t>(hostUs) * 1000, RCL_ROS_TIME);
    msg.header.frame_id = imu_frame_id;
    msg.orientation = tf2::toMsg(q);
    if (orientation_variance > 0.0)
        msg.orientation_covariance[0] = msg.orientation_covariance[4] =
            msg.orientation_covariance[8] = orientation_variance;
    if (angular_velocity_variance > 0.0)
        msg.angular_velocity_covariance[0] = msg.angular_velocity_covariance[4] =
            msg.angular_velocity_covariance[8] = angular_velocity_variance;
    if (linear_acceleration_variance > 0.0)
        msg.linear_acceleration_covariance[0] = msg.linear_acceleration_covariance[4] =
            msg.linear_acceleration_covariance[8] = linear_acceleration_variance;
    if (gimbal)
        gimbal->updateVehicleState(msg, gravity);
    imu_pub->publish(msg);

    // 地面且未解锁时估计当地重力，逻辑同 apm_bridge::VehicleNode::imuCallback。
    static uint8_t last_state = mavros_msgs::msg::ExtendedState::LANDED_STATE_UNDEFINED;
    const uint8_t current_state = landed_state.load();
    if (last_state != current_state)
    {
        last_state = current_state;
        ema.reset();
    }
    if (current_state == mavros_msgs::msg::ExtendedState::LANDED_STATE_ON_GROUND && !armed.load())
    {
        const double acc = std::sqrt(msg.linear_acceleration.x * msg.linear_acceleration.x +
                                     msg.linear_acceleration.y * msg.linear_acceleration.y +
                                     msg.linear_acceleration.z * msg.linear_acceleration.z);
        ema.filter(acc, 0);
    }
}

void CustomLinkVehicleNode::handleMedium(const custom_link::protocol::PayloadMedium &p, int64_t hostUs)
{
    mavros_msgs::msg::RCIn msg;
    msg.header.stamp = rclcpp::Time(static_cast<int64_t>(hostUs) * 1000, RCL_ROS_TIME);
    msg.rssi = p.rssi_pct; // RC 链路质量 0..100（真机 rx RSSI 源 / SITL 帧新鲜度）
    msg.channels.assign(p.rc, p.rc + 16);
    rc_in_pub->publish(msg);

    rcTriggerUpdate(p.rc, 16);

    const double alt = p.baro_alt_cm * 0.01; // cm -> m
    if (airsim_enabled)
    {
        // SITL reports absolute barometric altitude, not height above the launch surface.
        const double filtered = sitl_altitude_ema.filter(alt);
        if (armed_feedback_valid && !armed.load())
        {
            sitl_ground_altitude = filtered;
            sitl_ground_valid = true;
        }
        altitude_m.store(filtered - sitl_ground_altitude);
    }
    else
    {
        altitude_m.store(alt);
    }
    const double relative_altitude = altitude_m.load();
    if (previous_altitude_us != 0 && hostUs > previous_altitude_us) {
        const double dt = static_cast<double>(hostUs - previous_altitude_us) * 1.0e-6;
        const double velocity = (relative_altitude - previous_altitude_m) / dt;
        if (std::isfinite(velocity) && dt <= 1.0)
            vertical_velocity_mps.store(std::clamp(velocity, -10.0, 10.0));
    }
    previous_altitude_m = relative_altitude;
    previous_altitude_us = hostUs;
    std_msgs::msg::Float64 altitude;
    altitude.data = alt;
    altitude_pub->publish(altitude);
    if (baro_compensation)
    {
        // §4 步骤 1: F_std_target = F_req·(P0/P)·(T/T0); 气压/温度缺失时保持上一次系数
        // (启动时为 1, 即标准状态)。
        std::lock_guard<std::mutex> lk(mtx_kp);
        if (p.temp_cdeg > kAbsoluteZeroCdeg)
        {
            baro_temperature_c = p.temp_cdeg * 0.01;
        }
        else
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                "Ignoring invalid barometric temperature %.2f C; retaining the previous temperature",
                                p.temp_cdeg * 0.01);
        }
        const double factor = quadratic_thrust_model::environmentFactor(static_cast<double>(p.baro_pa),
                                                                        baro_temperature_c);
        if (std::isfinite(factor))
        {
            env_factor = factor;
        }
        else
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                "Ignoring zero barometric pressure; retaining the previous environment factor");
        }
    }

    const rclcpp::Time stamp(static_cast<int64_t>(hostUs) * 1000, RCL_ROS_TIME);
    sensor_msgs::msg::FluidPressure pressure;
    pressure.header.stamp = stamp;
    pressure.fluid_pressure = static_cast<double>(p.baro_pa);
    pressure.variance = 0.0;
    pressure_pub->publish(std::move(pressure));

    sensor_msgs::msg::Temperature temperature;
    temperature.header.stamp = stamp;
    temperature.temperature = p.temp_cdeg * 0.01; // 0.01 degC -> degC
    temperature.variance = 0.0;
    temperature_pub->publish(std::move(temperature));
}

void CustomLinkVehicleNode::handleSlow(const custom_link::protocol::PayloadSlow &p, int64_t hostUs)
{
    const double voltage = p.vbat_mv * 0.001;
    const double amperage = p.current_ma * 0.001;

    {
        // §4: 母线电压 (及总电流) 经 2~5 Hz 一阶低通后再参与补偿, 防止大油门突加时
        // 电压瞬降形成正反馈。按遥测时间戳计算 α, 截止频率与遥测速率无关。
        std::lock_guard<std::mutex> lk(mtx_kp);
        // 首个样本按 0x12 名义周期 (10 Hz) 推进, 滤波器从 V_ref 初值收敛 (§5)
        const double dt = last_battery_us > 0 ? (hostUs - last_battery_us) * 1e-6 : 0.1;
        last_battery_us = hostUs;
        if (voltage > 0.0)
            voltage_lpf.update(voltage, dt);
        if (amperage >= 0.0)
            current_lpf.update(amperage, dt);
    }

    const bool now_armed = (p.status & custom_link::protocol::STATUS_ARMED) != 0;
    const bool was_armed = armed.exchange(now_armed);
    armed_feedback_valid = true;
    client->setArmedFeedback(now_armed);
    // OFFBOARD 实际生效状态反馈给客户端：已生效时上层 2 Hz 重复的
    // /fpv/offboard=true 保活不再触发 mode_req 0->2 重试边沿（那会让 FC
    // 随每次保活退出再重进 OFFBOARD，油门在 RC 与主机间来回切换，转速抖动）。
    client->setOffboardFeedback(
        (p.mode_flags & custom_link::protocol::mode::OFFBOARD) != 0);

    // 起桨/落桨边沿驱动下游：从未起桨到起桨发布 ON 触发 Tracker，从起桨到
    // 未起桨发布 OFF 停止 Tracker 与 Interceptor（0x12 10 Hz，边沿单次发布；
    // 自身的 gimbal_trigger_sub 订阅会同步产生云台 AIM/中止请求）。
    if (now_armed != was_armed)
    {
        std_msgs::msg::Header trigger;
        trigger.stamp = this->now();
        trigger.frame_id = now_armed ? "ON" : "OFF";
        trigger_pub->publish(std::move(trigger));
        RCLCPP_INFO(this->get_logger(), "Props %s: tracker trigger %s",
                    now_armed ? "spinning (armed)" : "stopped (disarmed)",
                    trigger.frame_id.c_str());
    }

    publishState(p.mode_flags, now_armed);

    sensor_msgs::msg::BatteryState bat;
    bat.header.stamp = rclcpp::Time(static_cast<int64_t>(hostUs) * 1000, RCL_ROS_TIME);
    bat.voltage = static_cast<float>(voltage);
    bat.current = static_cast<float>(-amperage); // ROS 约定放电为负
    // mAh 累计 (0x12 携带)。固件计数值单位为 mAh (int16)；
    // 换算隔离在此常量，若固件侧改为其他单位只需调整这一处。
    constexpr double kMahPerCount = 1.0;
    const double consumed_ah = p.mah * kMahPerCount * 0.001;
    bat.charge = static_cast<float>(consumed_ah); // 已耗 Ah (MAVROS sys_status 惯例)
    if (battery_capacity_mah > 0.0)
    {
        const double capacity_ah = battery_capacity_mah * 0.001;
        bat.capacity = static_cast<float>(capacity_ah);
        bat.design_capacity = bat.capacity;
        bat.percentage = static_cast<float>(
            std::clamp(1.0 - consumed_ah / capacity_ah, 0.0, 1.0));
    }
    else
    {
        bat.capacity = std::numeric_limits<float>::quiet_NaN();
        bat.design_capacity = std::numeric_limits<float>::quiet_NaN();
        bat.percentage = std::numeric_limits<float>::quiet_NaN();
    }
    bat.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
    bat.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
    bat.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIPO;
    bat.present = voltage > 1.0;
    // cell_voltage[0] 放整包电压，与 MAVROS 在本工程中的用法保持一致。
    bat.cell_voltage = {static_cast<float>(voltage)};
    battery_pub->publish(bat);

    auto fb = std::make_unique<quadrotor_msgs::msg::LowLevelFeedback>();
    fb->header.stamp = bat.header.stamp;
    fb->battery_voltage = voltage;
    fb->battery_state = quadrotor_msgs::msg::LowLevelFeedback::BAT_INVALID;
    if (voltage > n_lipo_cells * kBatteryLowVoltagePerCell)
        fb->battery_state = quadrotor_msgs::msg::LowLevelFeedback::BAT_GOOD;
    else if (voltage > n_lipo_cells * kBatteryCriticalVoltagePerCell)
        fb->battery_state = quadrotor_msgs::msg::LowLevelFeedback::BAT_LOW;
    else if (voltage > n_lipo_cells * kBatteryInvalidVoltagePerCell)
        fb->battery_state = quadrotor_msgs::msg::LowLevelFeedback::BAT_CRITICAL;

    if (rc_manual.load())
        fb->control_mode = quadrotor_msgs::msg::LowLevelFeedback::RC_MANUAL;
    else
        fb->control_mode = use_rate.load() ? quadrotor_msgs::msg::LowLevelFeedback::BODY_RATES
                                           : quadrotor_msgs::msg::LowLevelFeedback::ATTITUDE;
    ap_feedback_pub->publish(std::move(fb));

    // ---- GPS (0x12 PayloadSlow)：GPSRAW 全量 + NavSatFix 经纬度 ----
    mavros_msgs::msg::GPSRAW gps;
    gps.header.stamp = bat.header.stamp;
    gps.fix_type = p.fix ? mavros_msgs::msg::GPSRAW::GPS_FIX_TYPE_3D_FIX
                         : mavros_msgs::msg::GPSRAW::GPS_FIX_TYPE_NO_FIX;
    gps.satellites_visible = p.sats;
    gps.lat = p.lat_e7;   // degE7 直传
    gps.lon = p.lon_e7;
    gps.alt = p.alt_msl_cm * 10; // cm -> mm
    gps.eph = std::numeric_limits<uint16_t>::max(); // 固件未提供精度信息
    gps.epv = std::numeric_limits<uint16_t>::max();
    gps.vel = p.gspeed_cms;      // cm/s
    gps.cog = p.course_cdeg;     // 0.01 deg
    gpsraw_pub->publish(std::move(gps));

    sensor_msgs::msg::NavSatFix fix;
    fix.header.stamp = bat.header.stamp;
    fix.header.frame_id = "base_link";
    fix.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
    fix.status.status = p.fix ? sensor_msgs::msg::NavSatStatus::STATUS_FIX
                              : sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
    fix.latitude = p.lat_e7 * 1e-7;
    fix.longitude = p.lon_e7 * 1e-7;
    fix.altitude = p.alt_msl_cm * 0.01;
    fix.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
    navsat_pub->publish(std::move(fix));

    // ---- 失效保护标志 ----
    std_msgs::msg::Bool failsafe;
    failsafe.data = (p.status & custom_link::protocol::STATUS_FAILSAFE) != 0;
    failsafe_pub->publish(std::move(failsafe));
}

void CustomLinkVehicleNode::handleOdometry(const custom_link::protocol::PayloadOdometry &p, int64_t hostUs)
{
    if ((p.flags & custom_link::protocol::ODOM_VALID_FLAGS) != custom_link::protocol::ODOM_VALID_FLAGS)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Ignoring odometry without GPS 3D fix, ARM origin or valid armed estimate");
        return;
    }
    if (p.arm_yaw_cdeg < -18000 || p.arm_yaw_cdeg > 18000)
    {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Ignoring odometry with invalid ARM heading");
        return;
    }
    for (int axis = 0; axis < 3; ++axis)
    {
        if (!std::isfinite(p.position_variance[axis]) || p.position_variance[axis] < 0.0f ||
            !std::isfinite(p.velocity_variance[axis]) || p.velocity_variance[axis] < 0.0f)
        {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Ignoring odometry with invalid covariance");
            return;
        }
    }
    nav_msgs::msg::Odometry msg;
    msg.header.stamp = rclcpp::Time(hostUs * 1000, RCL_ROS_TIME);
    msg.header.frame_id = "odom";
    msg.child_frame_id = imu_frame_id;
    const tf2::Quaternion arm_heading(tf2::Vector3(0, 0, 1),
        normalizeAngle(p.arm_yaw_cdeg * 0.01 * kDegToRad + M_PI_2));
    const tf2::Matrix3x3 world_to_odom(arm_heading.inverse());
    const tf2::Vector3 position = world_to_odom *
        tf2::Vector3(p.position_cm[0] * 0.01, p.position_cm[1] * 0.01, p.position_cm[2] * 0.01);
    msg.pose.pose.position.x = position.x();
    msg.pose.pose.position.y = position.y();
    msg.pose.pose.position.z = position.z();
    // 姿态四元数：0x14 与 0x10 同约定（w,x,y,z ×16384，FLU->ENU），
    // 解码归一化后直用，无任何参考旋转/符号变换
    tf2::Quaternion q(p.quat[1] / 16384.0, p.quat[2] / 16384.0,
                      p.quat[3] / 16384.0, p.quat[0] / 16384.0);   // (x, y, z, w)
    {
        const double n = std::sqrt(q.x() * q.x() + q.y() * q.y() +
                                   q.z() * q.z() + q.w() * q.w());
        if (!std::isfinite(n) || std::fabs(n - 1.0) > 0.1)
        {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Ignoring odometry with invalid attitude quaternion");
            return;
        }
        q = tf2::Quaternion(q.x() / n, q.y() / n, q.z() / n, q.w() / n);
    }
    msg.pose.pose.orientation = tf2::toMsg(arm_heading.inverse() * q);
    // Twist stays in body FLU; only the pose frame changes from ENU to ARM heading.
    const tf2::Matrix3x3 world_to_body(q.inverse());
    const tf2::Vector3 velocity = world_to_body *
        tf2::Vector3(p.velocity_cms[0] * 0.01, p.velocity_cms[1] * 0.01, p.velocity_cms[2] * 0.01);
    msg.twist.twist.linear.x = velocity.x();
    msg.twist.twist.linear.y = velocity.y();
    msg.twist.twist.linear.z = velocity.z();
    msg.twist.twist.angular.x = p.gyro_dps_x10[0] * 0.1 * kDegToRad;
    msg.twist.twist.angular.y = p.gyro_dps_x10[1] * 0.1 * kDegToRad;
    msg.twist.twist.angular.z = p.gyro_dps_x10[2] * 0.1 * kDegToRad;
    for (int row = 0; row < 3; ++row)
    {
        // Conservative uncertainty when IMU covariance is not configured (zero means unknown).
        msg.pose.covariance[(row + 3) * 6 + row + 3] = orientation_variance > 0 ? orientation_variance : M_PI * M_PI;
        msg.twist.covariance[(row + 3) * 6 + row + 3] = angular_velocity_variance > 0 ? angular_velocity_variance : 1.0e6;
        for (int col = 0; col < 3; ++col)
        {
            for (int axis = 0; axis < 3; ++axis)
            {
                msg.pose.covariance[row * 6 + col] += world_to_odom[row][axis] *
                    p.position_variance[axis] * world_to_odom[col][axis];
                msg.twist.covariance[row * 6 + col] += world_to_body[row][axis] *
                    p.velocity_variance[axis] * world_to_body[col][axis];
            }
        }
    }
    odom_pub->publish(msg);
}

void CustomLinkVehicleNode::handleEscTelemetry(const custom_link::protocol::PayloadEscTelemetry &p, int64_t hostUs)
{
    // 机械 rpm 由固件按 motor_poles 换算后直接下发，桥只透传。valid 位缺失
    //（双向 DShot 未开/首个 eRPM 帧未到）的电机输出 NaN，下游用 isnan 判别
    // 而不是把 0 误读为停转。
    std_msgs::msg::Float32MultiArray msg;
    msg.layout.dim.resize(1);
    msg.layout.dim[0].label = "motor";
    msg.layout.dim[0].size = p.count;
    msg.layout.dim[0].stride = p.count;
    msg.data.reserve(p.count);
    for (uint8_t i = 0; i < p.count; ++i)
    {
        if (p.valid & (1u << i))
            msg.data.push_back(p.rpm[i]);
        else
            msg.data.push_back(std::numeric_limits<float>::quiet_NaN());
    }
    motor_rpm_pub->publish(msg);
}

void CustomLinkVehicleNode::publishState(uint32_t mode_flags, bool is_armed)
{
    namespace mode = custom_link::protocol::mode;

    // OFFBOARD 是自定义链路的接管模式，等价于 ArduPilot GUIDED / 旧 MSP_OVERRIDE。
    const bool offboard = mode_flags & mode::OFFBOARD;
    const bool was_offboard = !rc_manual.load();

    rc_manual.store(!offboard);

    if (in_mission.load() && was_offboard && !offboard)
    {
        // Send a trigger to indicate that offboard mode was lost during a mission
        std_msgs::msg::Header trigger;
        trigger.stamp = this->now();
        trigger.frame_id = "DISMISS";
        trigger_pub->publish(std::move(trigger));
    }

    const double alt = altitude_m.load();
    const double vertical_velocity = vertical_velocity_mps.load();
    const auto now_us = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    const uint8_t in_air = mavros_msgs::msg::ExtendedState::LANDED_STATE_IN_AIR;
    const uint8_t on_ground = mavros_msgs::msg::ExtendedState::LANDED_STATE_ON_GROUND;

    if (airsim_enabled && !sitl_ground_valid)
    {
        landed_state.store(mavros_msgs::msg::ExtendedState::LANDED_STATE_UNDEFINED);
        landed_candidate_state = mavros_msgs::msg::ExtendedState::LANDED_STATE_UNDEFINED;
        landed_candidate_since_us = 0;
        RCLCPP_WARN_ONCE(this->get_logger(), "SITL ground reference unavailable; landed state is undefined");
    }
    else if (!is_armed)
    {
        landed_state.store(on_ground);
        landed_candidate_state = on_ground;
        landed_candidate_since_us = 0;
    }
    else
    {
        uint8_t candidate = mavros_msgs::msg::ExtendedState::LANDED_STATE_UNDEFINED;
        if (landed_state.load() == in_air) {
            if (alt <= landing_altitude && std::abs(vertical_velocity) <= 0.35)
                candidate = on_ground;
        }
        else if (alt <= landing_altitude && std::abs(vertical_velocity) <= 0.35) {
            candidate = on_ground;
        }
        else if (alt >= airborne_altitude &&
                 (vertical_velocity >= 0.05 || alt >= airborne_altitude + 0.2)) {
            candidate = in_air;
        }

        if (candidate != landed_candidate_state) {
            landed_candidate_state = candidate;
            landed_candidate_since_us = candidate == mavros_msgs::msg::ExtendedState::LANDED_STATE_UNDEFINED
                ? 0 : now_us;
        }
        if (candidate != mavros_msgs::msg::ExtendedState::LANDED_STATE_UNDEFINED &&
            landed_candidate_since_us != 0) {
            const double confirmation_s = candidate == in_air ? takeoff_confirm_time_s : landing_confirm_time_s;
            if (now_us - landed_candidate_since_us >= static_cast<int64_t>(confirmation_s * 1.0e6))
                landed_state.store(candidate);
        }
    }

    mavros_msgs::msg::State state;
    state.header.stamp = this->now();
    state.connected = client->isUp();
    state.armed = is_armed;
    state.guided = offboard;
    state.manual_input = !offboard;

    if (offboard) {
        state.mode = (mode_flags & mode::ANGLE) ? "OFFBOARD|ANGLE" : "OFFBOARD";
    }
    else {
        bool plus = false;
        for(auto i = 0; i < mode_strings.size(); ++i) {
            if (mode_flags & (1u << i)) {
                if (plus)
                    state.mode += "|";
                else 
                    plus = true;
                state.mode += mode_strings[i];
            }
        }
        if (!plus)
            state.mode = "ACRO";
    }
    state_pub->publish(std::move(state));

    mavros_msgs::msg::ExtendedState ext;
    ext.header.stamp = this->now();
    ext.landed_state = landed_state.load();
    ext.vtol_state = mavros_msgs::msg::ExtendedState::VTOL_STATE_UNDEFINED;
    ext_state_pub->publish(std::move(ext));
}

// ======================================================================
// 控制
// ======================================================================

void CustomLinkVehicleNode::pushControl()
{
    custom_link::ControlInput input;
    bool fresh = false;
    {
        std::lock_guard<std::mutex> lk(mtx_cmd);
        if (cmd_valid)
        {
            if ((this->now() - cmd_stamp).seconds() < control_timeout)
            {
                // ROS FLU rad(/s) -> Betaflight deg 域，同轴同号直通换算：
                // 角速度模式为三轴 rad/s（BF 陀螺仪域即 FLU 角速度，见 handleFast
                // 注释）；角度模式 roll/pitch 为 rad、yaw 为 rad/s（mode_req=3）。
                input.rate_dps_or_angle_deg[0] = cmd_setpoints[0] * kRadToDeg;
                input.rate_dps_or_angle_deg[1] = cmd_setpoints[1] * kRadToDeg;
                input.rate_dps_or_angle_deg[2] = cmd_setpoints[2] * kRadToDeg;
                if (cmd_force_mode)
                {
                    // §4 在线反解 (每个控制周期用最新的环境系数与低通电压/电流):
                    // F_req -> 占空比 u ∈ [u_idle, 1] -> Betaflight 集体油门 (u_idle 对应 c = 0)。
                    std::lock_guard<std::mutex> lk(mtx_kp);
                    const double nan = std::numeric_limits<double>::quiet_NaN();
                    const double duty = quadratic_thrust_model::calculateThrottle(
                        propulsion, cmd_force, env_factor,
                        voltage_compensation ? voltage_lpf.value : nan,
                        voltage_compensation ? current_lpf.value : nan);
                    input.throttle = quadratic_thrust_model::dutyToCollective(duty, propulsion.u_idle);
                }
                else
                    input.throttle = cmd_thrust;
                input.angle = cmd_angle;
                input.motor_direct = cmd_motor_direct;
                fresh = true;
            }
            else
            {
                cmd_valid = false; // 只作废控制量，arm 请求独立保留
            }
        }
    }
    // 发送前限幅（0 = 不限制），客户端防御层，独立于固件侧限制：
    //   角速度通道 -> link.rate_limit_dps vs 固件 custom_link_rate_limit_dps，取更小；
    //   角度模式 roll/pitch -> link.angle_limit_deg vs 固件 pid profile 的
    //   angle_limit（CLI 可调，默认 60°），同样取更小。
    if (airsim_enabled && fresh &&
        (!std::isfinite(input.throttle) || !std::all_of(std::begin(input.rate_dps_or_angle_deg), std::end(input.rate_dps_or_angle_deg),
                                                      [](double rate) { return std::isfinite(rate); })))
    {
        fresh = false;
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Rejected non-finite SITL control");
    }
    if (input.angle) {
        if (fresh && angle_limit_deg > 0.01) {
            input.rate_dps_or_angle_deg[0] = std::clamp(input.rate_dps_or_angle_deg[0], -angle_limit_deg, angle_limit_deg);
            input.rate_dps_or_angle_deg[1] = std::clamp(input.rate_dps_or_angle_deg[1], -angle_limit_deg, angle_limit_deg);
        }
        
    } 
    else {
        if (fresh && rate_limit_dps > 0.01) {
            input.rate_dps_or_angle_deg[0] = std::clamp(input.rate_dps_or_angle_deg[0], -rate_limit_dps, rate_limit_dps);
            input.rate_dps_or_angle_deg[1] = std::clamp(input.rate_dps_or_angle_deg[1], -rate_limit_dps, rate_limit_dps);
        }
    }
    client->setControl(input, fresh);
}

void CustomLinkVehicleNode::updateGimbalMode()
{
    // 启动回中 (机械零点) 未完成: 云台由驱动回中状态机接管 (FPV+角度回零),
    // 模式选择器暂不介入; 期间到达的 AIM/中止边沿直接消费掉, 回中结束后
    // 由选择器按当前状态 (未解锁即 AIM) 重新决策, 不重放旧请求。
    if (gimbal && !gimbal->homed())
    {
        gimbal_aim_requested = false;
        gimbal_abort_requested = false;
        if (!gimbal_homing)
        {
            gimbal_homing = true;
            std_msgs::msg::String msg;
            msg.data = "HOMING";
            gimbal_mode_pub->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Gimbal mode -> HOMING (returning to mechanical zero)");
        }
        return;
    }
    const bool left_homing = gimbal_homing;
    gimbal_homing = false;
    const auto next = gimbal_mode_selector.update(armed.load(),
        landed_state.load() == mavros_msgs::msg::ExtendedState::LANDED_STATE_IN_AIR ||
            (mission_gimbal_on_ground && in_mission.load()),
        in_mission.load(), rc_manual.load(),
        gimbal_aim_requested.exchange(false), gimbal_abort_requested.exchange(false));
    // 回中刚结束也强制发布一次: gimbal_mode 可能未变 (AIM), 但对外状态
    // 需要从 "HOMING" 切换到实际模式并让驱动真正进入该模式。
    if (next == gimbal_mode && !left_homing)
        return;
    gimbal_mode = next;
    if (gimbal)
        gimbal->setMode(next);
    std_msgs::msg::String msg;
    msg.data = apm_bridge::gimbalModeName(next);
    gimbal_mode_pub->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Gimbal mode -> %s", msg.data.c_str());
}


void CustomLinkVehicleNode::ctrlCommandCallback(const quadrotor_msgs::msg::ControlCommand::SharedPtr command)
{
    if (set_unarm && armed.load() &&
        landed_state.load() == mavros_msgs::msg::ExtendedState::LANDED_STATE_ON_GROUND)
    {
        client->setArmRequest(false);
        set_unarm = false;
        return;
    }

    // 未解锁时不受理；解锁后的控制流是否接管由 FC 的 OFFBOARD 授权仲裁决定。
    if (command->armed == 0 || !armed.load())
        return;

    // 非有限推力会经推力模型落成 NaN 油门, 客户端 uint16 编码时为未定义行为
    // (x86 上通常落到最低油门); 与 raw 回调一致, 整帧丢弃。
    if (!std::isfinite(command->collective_thrust))
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Dropping control command: non-finite collective_thrust");
        return;
    }
    // 未标定 (a3..a1 全零) 时不存在 F_req -> 油门的映射, 拒绝力指令而不是输出怠速。
    if (!quadratic_thrust_model::configured(propulsion))
    {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                              "Dropping control command: motor_parameters uncalibrated (run thrust_test_node)");
        return;
    }
    // u_idle (飞控 motor_idle) 未读到前无法把占空比换算回集体油门。
    if (!motor_idle_known.load())
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Dropping control command: FC motor_idle (u_idle) not yet read via CLI");
        return;
    }
    // 模型未在真实 u_idle 下通过 §3.2 曲线判据 (非单调窗口/怠速点推力非正)。
    if (!propulsion_valid.load())
    {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                              "Dropping control command: thrust model failed the section 3.2 "
                              "gate at the FC motor_idle");
        return;
    }

    if (command->collective_thrust < 1e-6) // 准备落地上锁
        set_unarm = true;

    in_hover.store(command->is_hover_state != 0);

    // ATTITUDE = 固件原生角度模式（0x20 mode_req=3）：roll/pitch 取姿态四元数，
    // yaw 仍是角速度；BODY_RATES = 角速度模式（mode_req=2）。
    const bool angle = command->control_mode == quadrotor_msgs::msg::ControlCommand::ATTITUDE;
    std::array<double, 3> setpoints{command->bodyrates.x, command->bodyrates.y, command->bodyrates.z};
    if (angle && !attitudeSetpoints(*command, setpoints))
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Dropping ATTITUDE command: orientation quaternion not unit "
                             "(|norm-1| > 1%%) or non-finite setpoint");
        return;
    }
    if (!std::all_of(setpoints.begin(), setpoints.end(), [](double value) { return std::isfinite(value); }))
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Dropping control command: non-finite body rate");
        return;
    }
    use_rate.store(!angle);


    const double force = command->collective_thrust * mass;

    std::lock_guard<std::mutex> lk(mtx_cmd);
    cmd_angle = angle;
    cmd_setpoints = setpoints;
    cmd_force = force;
    cmd_force_mode = true;
    cmd_stamp = this->now();
    cmd_valid = true;
}

void CustomLinkVehicleNode::ctrlCommandRawCallback(const quadrotor_msgs::msg::ControlCommand::SharedPtr command)
{
    if (command->armed == 0 || !armed.load())
        return;

    // 与 ctrlCommandCallback 同一套 ATTITUDE -> 角度模式映射；区别仅在油门：
    // 本回调的 collective_thrust 已是归一化推力 0..1，直接透传。
    // MOTOR_DIRECT（mode_req=1，台架标定专用）：纯集体油门直通，姿态/角速度
    // 全部旁路 —— FC 端丢弃 PID 差动项，需 custom_link_motor_direct=ON 才受理。
    const bool angle = command->control_mode == quadrotor_msgs::msg::ControlCommand::ATTITUDE;
    const bool motor_direct = command->control_mode == quadrotor_msgs::msg::ControlCommand::MOTOR_DIRECT;
    std::array<double, 3> setpoints{command->bodyrates.x, command->bodyrates.y, command->bodyrates.z};
    if (angle && !attitudeSetpoints(*command, setpoints))
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Dropping ATTITUDE command: orientation quaternion not unit "
                             "(|norm-1| > 1%%) or non-finite setpoint");
        return;
    }
    if (!std::isfinite(command->collective_thrust) || !std::all_of(setpoints.begin(), setpoints.end(),
        [](double value) { return std::isfinite(value); }))
        return;
    use_rate.store(!angle);

    std::lock_guard<std::mutex> lk(mtx_cmd);
    cmd_angle = angle;
    cmd_motor_direct = motor_direct;
    cmd_setpoints = setpoints;
    cmd_thrust = std::clamp(command->collective_thrust, 0.0, 1.0);
    cmd_force_mode = false;
    cmd_stamp = this->now();
    cmd_valid = true;
}

void CustomLinkVehicleNode::armCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    // 解锁请求与模式请求独立；FC 仍执行 OFFBOARD 授权与 tryArm 安全检查。
    client->setArmRequest(msg->data);
}

void CustomLinkVehicleNode::offboardCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (!msg->data)
        gimbal_abort_requested = true;
    client->setOffboardRequest(msg->data);
    // 工具的 2 Hz level 保活会连续到达相同电平（客户端按 FC 反馈去重），
    // 日志同样只在电平变化时记一次。
    if (!last_offboard_log_level || *last_offboard_log_level != msg->data)
    {
        last_offboard_log_level = msg->data;
        RCLCPP_INFO(this->get_logger(), "Requested custom-link mode: %s",
                    msg->data ? "OFFBOARD" : "MANUAL");
    }
}

bool CustomLinkVehicleNode::visionLockHoldover() const
{
    if (in_tracking.load())
        return true;
    const double last_locked = tracking_hold_s.load();
    return last_locked > 0.0 && (now().seconds() - last_locked) < TRACKING_HOLDOVER_S;
}

void CustomLinkVehicleNode::rcTriggerUpdate(const uint16_t *channels, size_t count)
{
    // RC 触发脉冲（>0.5 s 保持判定、单次锁存、释放复位）只负责 MISSION 的
    // 启动与停止：任务中脉冲 = 停止（DISMISS，仅解除拦截控制，Tracker 保持
    // 运行，联动退出 OFFBOARD）；空闲且视觉/雷达锁定时脉冲 = 触发（MISSION，
    // 联动进入 OFFBOARD）。Tracker 的启停由起桨/落桨边沿自动发布的 ON/OFF
    // 负责（handleSlow），不再由脉冲翻转。AirSim 未解锁时的脉冲仅启动视觉预览。
    static rclcpp::Time last_triggered_time;
    static bool toggled = false;

    if (static_cast<int>(count) <= channel_trigger)
        return;

    if (!toggled)
    {
        if (channels[channel_trigger] > 1900)
        {
            auto current_time = this->now();
            if (last_triggered_time.nanoseconds() == 0)
            {
                last_triggered_time = current_time;
            }
            else if ((current_time - last_triggered_time).seconds() > 0.5)
            {
                last_triggered_time = rclcpp::Time(0);
                toggled = true;
                auto msg = std::make_unique<std_msgs::msg::Header>();
                msg->stamp = this->now();
                if (airsim_enabled && !armed.load())
                {
                    // RC-only, non-arming vision preview for AirSim aim checks.
                    // Even a locked tracker must not start MISSION while disarmed.
                    msg->frame_id = "ON";
                }
                else if (in_mission.load())
                {
                    msg->frame_id = "DISMISS";
                }
                else if (visionLockHoldover() || radar_tracking.load())
                {
                    msg->frame_id = "MISSION";
                }
                else
                {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                        "Trigger pulse ignored: no vision/radar lock and no mission running");
                    // 拒绝时不消耗脉冲锁存: 重置后在持续高电平下, 下一个 >=0.5 s 保持
                    // 窗口自动重试, 视觉锁定恢复 (tracker 丢失->重捕典型 0.3-0.7 s) 即补发,
                    // 无需飞行员/测试脚本重新完整打一个脉冲 (2026-11-09 发射失败根因之一)。
                    last_triggered_time = rclcpp::Time(0);
                    toggled = false;
                    return;
                }
                RCLCPP_INFO(this->get_logger(), "Tracker Triggered : %s (vision=%d radar=%d)",
                            msg->frame_id.c_str(), in_tracking.load(), radar_tracking.load());
                trigger_pub->publish(std::move(msg));
            }
        }
        else
        {
            last_triggered_time = rclcpp::Time(0);
        }
    }
    else if (channels[channel_trigger] < 1100)
    {
        toggled = false;
    }
}

// ======================================================================
// CLI 设置读写服务 (/fpv/cli)：延迟响应回调入队即返回，工作线程等链路
// 响应后 send_response 回包——单线程 executor 不会被慢 CLI 调用/超时
// 卡住控制与遥测。
// ======================================================================

void CustomLinkVehicleNode::cliRespond(const std::shared_ptr<rmw_request_id_t> header,
                                       std::shared_ptr<quadrotor_msgs::srv::CliCommand::Response> response)
{
    try
    {
        cli_service->send_response(*header, *response);
    }
    catch (const std::exception &e)
    {
        RCLCPP_WARN(this->get_logger(), "CLI service response failed: %s", e.what());
    }
}

void CustomLinkVehicleNode::cliServiceCallback(
    const std::shared_ptr<rmw_request_id_t> header,
    const std::shared_ptr<quadrotor_msgs::srv::CliCommand::Request> request)
{
    // 廉价校验在 executor 线程完成，坏请求立即回包不入队
    const char *bad = nullptr;
    if (request->cmd > quadrotor_msgs::srv::CliCommand::Request::CMD_APPLY_PID)
        bad = "bad request: unknown CLI command";
    else if (request->args.size() + 1 > custom_link::protocol::CLI_MAX_ARGS)
        bad = "bad request: args too long (max 59 chars)";
    if (bad != nullptr)
    {
        auto response = std::make_shared<quadrotor_msgs::srv::CliCommand::Response>();
        response->ok = false;
        response->error = bad;
        cliRespond(header, std::move(response));
        return;
    }

    bool queued = false;
    {
        std::lock_guard<std::mutex> lk(cli_job_mtx);
        // 队列有界：请求方堆积过多直接回忙，而不是无限排队
        constexpr size_t kMaxQueue = 8;
        if (cli_jobs.size() < kMaxQueue)
        {
            cli_jobs.push_back(CliJob{header, request});
            queued = true;
        }
    }
    if (queued)
    {
        cli_job_cv.notify_one();
        return; // 回包由工作线程延迟完成
    }
    auto response = std::make_shared<quadrotor_msgs::srv::CliCommand::Response>();
    response->ok = false;
    response->error = "busy: too many queued CLI requests";
    cliRespond(header, std::move(response));
}

void CustomLinkVehicleNode::maybeStartRxBind()
{
    if (!client)
        return;
    const auto result = client->cliCommand(custom_link::protocol::CLI_CMD_BIND_RX, "", cli_timeout);
    if (result.ok && result.status == custom_link::protocol::CLI_STATUS_OK)
        RCLCPP_INFO(this->get_logger(), "RX bind started on connect (bind_rx): %s", result.text.c_str());
    else if (result.ok)
        RCLCPP_WARN(this->get_logger(), "RX bind on connect rejected by FC: %s", result.text.c_str());
    else
        RCLCPP_WARN(this->get_logger(), "RX bind on connect failed: %s", result.error.c_str());
}

void CustomLinkVehicleNode::scheduleIdleQuery(std::chrono::steady_clock::time_point due)
{
    idle_query_pending = true;
    idle_query_due = due;
}

bool CustomLinkVehicleNode::queryMotorIdle()
{
    // fpv_test.md §4 步骤 4 的 u_idle 即飞控 motor_idle: 集体油门 0 对应的 DShot 占空比
    // u = u_idle + (1 - u_idle)·c, 只有读到它才能把反解出的占空比换回集体油门。
    if (!client)
        return false;
    const auto result = client->cliCommand(custom_link::protocol::CLI_CMD_GET, "motor_idle", cli_timeout);
    const double idle = result.ok && result.status == custom_link::protocol::CLI_STATUS_OK
                            ? quadratic_thrust_model::idleDutyFromCli(result.text)
                            : std::numeric_limits<double>::quiet_NaN();
    if (!std::isfinite(idle))
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
                             "Reading FC motor_idle via CLI failed (%s); force commands stay rejected, retrying",
                             result.ok ? result.text.c_str() : result.error.c_str());
        return false;
    }
    double inverse_error = -1.0;
    {
        std::lock_guard<std::mutex> lk(mtx_kp);
        propulsion.u_idle = idle;
        if (quadratic_thrust_model::configured(propulsion))
        {
            // §3.2 曲线判据此刻才有真实窗口: 不通过则拒绝力指令 (raw 通道不受影响),
            // 而不是静默退回 §5 原始初值在非单调窗口上跑牛顿迭代。
            const std::string curve_error = quadratic_thrust_model::validateModel(propulsion);
            propulsion_valid = curve_error.empty();
            if (curve_error.empty())
                inverse_error = quadratic_thrust_model::initInverse(propulsion);
            else
                RCLCPP_ERROR(this->get_logger(),
                             "motor_parameters fail the section 3.2 gate at the FC u_idle=%.4f: "
                             "%s; force commands rejected",
                             idle, curve_error.c_str());
        }
        motor_idle_known = true;
    }
    if (inverse_error >= 0.0)
        RCLCPP_INFO(this->get_logger(),
                    "FC %s -> u_idle = %.4f (collective 0); inverse-cubic initial guess max error %.2e",
                    result.text.c_str(), idle, inverse_error);
    else
        RCLCPP_INFO(this->get_logger(), "FC %s -> u_idle = %.4f (collective 0)", result.text.c_str(), idle);
    return true;
}

void CustomLinkVehicleNode::cliWorkerLoop()
{
    std::unique_lock<std::mutex> lk(cli_job_mtx);
    const auto idle_due = [this] {
        return idle_query_pending && std::chrono::steady_clock::now() >= idle_query_due;
    };
    const auto ready = [this, &idle_due] {
        return !cli_jobs.empty() || rx_bind_pending || idle_due() || !cli_worker_running.load();
    };
    while (cli_worker_running.load())
    {
        if (idle_query_pending)
            cli_job_cv.wait_until(lk, idle_query_due, ready);
        else
            cli_job_cv.wait(lk, ready);
        // 连接触发的接收机配对（cli.bind_rx_on_connect）：优先于排队请求
        if (rx_bind_pending)
        {
            rx_bind_pending = false;
            lk.unlock();
            maybeStartRxBind();
            lk.lock();
        }
        // 连接触发的 motor_idle 读取; 失败 (链路未通/飞控无 CLI) 周期重试
        if (idle_due())
        {
            idle_query_pending = false;
            lk.unlock();
            const bool ok = queryMotorIdle();
            lk.lock();
            if (!ok && !idle_query_pending)
                scheduleIdleQuery(std::chrono::steady_clock::now() + kIdleQueryRetry);
        }
        // 内层循环也要感知停止：FC 无响应时每个积压请求都吃满超时，
        // 析构 join 不能拖 kMaxQueue x cli_timeout；剩下的交给下面的
        // 关停排空快速回包
        while (!cli_jobs.empty() && cli_worker_running.load())
        {
            CliJob job = std::move(cli_jobs.front());
            cli_jobs.pop_front();
            lk.unlock();

            auto result = custom_link::Client::CliResult{};
            if (client)
                result = client->cliCommand(job.request->cmd, job.request->args, cli_timeout);
            else
                result.error = "link not initialized";
            // 经 /fpv/cli 改了 motor_idle: 立即重读, 保持 u_idle 与飞控一致
            const bool idle_changed = job.request->cmd == quadrotor_msgs::srv::CliCommand::Request::CMD_SET &&
                                      result.ok && result.status == custom_link::protocol::CLI_STATUS_OK &&
                                      job.request->args.rfind("motor_idle", 0) == 0;
            auto response = std::make_shared<quadrotor_msgs::srv::CliCommand::Response>();
            response->ok = result.ok;
            response->status = result.status;
            response->text = std::move(result.text);
            response->error = std::move(result.error);
            cliRespond(job.header, std::move(response));

            lk.lock();
            if (idle_changed)
                scheduleIdleQuery(std::chrono::steady_clock::now());
        }
    }
    // 关停排空：仍在队列里的请求明确回包，客户端不至于悬挂等超时
    while (!cli_jobs.empty())
    {
        CliJob job = std::move(cli_jobs.front());
        cli_jobs.pop_front();
        lk.unlock();
        auto response = std::make_shared<quadrotor_msgs::srv::CliCommand::Response>();
        response->ok = false;
        response->error = "node shutting down";
        cliRespond(job.header, std::move(response));
        lk.lock();
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<custom_link_bridge::CustomLinkVehicleNode>();
    rclcpp::spin(node);
    node.reset();
    rclcpp::shutdown();
    return 0;
}
