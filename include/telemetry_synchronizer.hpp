#pragma once

#include <ifc_tools/msg/vehicle_angular_velocity.hpp>
#include <ifc_tools/msg/vehicle_attitude.hpp>
#include <ifc_tools/msg/vehicle_global_position.hpp>
#include <ifc_tools/msg/vehicle_local_position.hpp>

#include <ifc_msgs/msg/autopilot_telemetry.hpp>
#include <ifc_msgs/msg/calibration_results.hpp>
#include <ifc_msgs/msg/stream_info.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace telemetry_synchronizer
{

class TelemetrySynchronizer : public rclcpp::Node
{
public:
    TelemetrySynchronizer() : rclcpp::Node("telemetry_synchronizer")
    {
        const double keep_pct = declare_parameter<double>("telemetry_synchronizer.keep_pct", 10.0);

        const std::size_t attitude_size = static_cast<std::size_t>(declare_parameter<int>("telemetry_synchronizer.vehicle_attitude_buffer_size", 1000));
        _attitude_keep = static_cast<std::size_t>(static_cast<double>(attitude_size) * keep_pct / 100.0);
        _attitude = std::make_shared<ifc_tools::VehicleAttitudeReceiver>(
            "telemetry_synchronizer_attitude",
            _attitude_utils,
            attitude_size,
            declare_parameter<std::string>("autopilot.vehicle_attitude", "/fmu/out/vehicle_attitude"));

        const std::size_t angular_velocity_size = static_cast<std::size_t>(declare_parameter<int>("telemetry_synchronizer.vehicle_angular_velocity_buffer_size", 1000));
        _angular_velocity_keep = static_cast<std::size_t>(static_cast<double>(angular_velocity_size) * keep_pct / 100.0);
        _angular_velocity = std::make_shared<ifc_tools::VehicleAngularVelocityReceiver>(
            "telemetry_synchronizer_angular_velocity",
            _angular_velocity_utils,
            angular_velocity_size,
            declare_parameter<std::string>("autopilot.vehicle_angular_velocity", "/fmu/out/vehicle_angular_velocity"));

        const std::size_t local_position_size = static_cast<std::size_t>(declare_parameter<int>("telemetry_synchronizer.vehicle_local_position_buffer_size", 500));
        _local_position_keep = static_cast<std::size_t>(static_cast<double>(local_position_size) * keep_pct / 100.0);
        _local_position = std::make_shared<ifc_tools::VehicleLocalPositionReceiver>(
            "telemetry_synchronizer_local_position",
            _local_position_utils,
            local_position_size,
            declare_parameter<std::string>("autopilot.vehicle_local_position", "/fmu/out/vehicle_local_position"));

        const std::size_t global_position_size = static_cast<std::size_t>(declare_parameter<int>("telemetry_synchronizer.vehicle_global_position_buffer_size", 500));
        _global_position_keep = static_cast<std::size_t>(static_cast<double>(global_position_size) * keep_pct / 100.0);
        _global_position = std::make_shared<ifc_tools::VehicleGlobalPositionReceiver>(
            "telemetry_synchronizer_global_position",
            _global_position_utils,
            global_position_size,
            declare_parameter<std::string>("autopilot.vehicle_global_position", "/fmu/out/vehicle_global_position"));

        _publisher = create_publisher<ifc_msgs::msg::AutopilotTelemetry>(
            declare_parameter<std::string>("telemetry_synchronizer.topic", "/telemetry_synchronizer/autopilot_telemetry"),
            rclcpp::ServicesQoS());

        _calibration_results_subscription = create_subscription<ifc_msgs::msg::CalibrationResults>(
            declare_parameter<std::string>("calibration_manager.topic", "/calibration_manager/calibration_results"),
            rclcpp::ServicesQoS(),
            [this](const ifc_msgs::msg::CalibrationResults& calibration_results)
            {
                _calibration_results = calibration_results;
            });

        _stream_info_subscription = create_subscription<ifc_msgs::msg::StreamInfo>(
            declare_parameter<std::string>("stream_telemetry.stream_info_topic", "/stream_telemetry/stream_info"),
            rclcpp::ServicesQoS(),
            [this](const ifc_msgs::msg::StreamInfo& stream_info)
            {
                receive(stream_info);
            });
    }

    std::vector<rclcpp::Node::SharedPtr> children() const
    {
        return {_attitude, _angular_velocity, _local_position, _global_position};
    }

private:
    const std::shared_ptr<const ifc_tools::VehicleAttitudeUtils> _attitude_utils = std::make_shared<ifc_tools::VehicleAttitudeUtils>();
    const std::shared_ptr<const ifc_tools::VehicleAngularVelocityUtils> _angular_velocity_utils = std::make_shared<ifc_tools::VehicleAngularVelocityUtils>();
    const std::shared_ptr<const ifc_tools::VehicleLocalPositionUtils> _local_position_utils = std::make_shared<ifc_tools::VehicleLocalPositionUtils>();
    const std::shared_ptr<const ifc_tools::VehicleGlobalPositionUtils> _global_position_utils = std::make_shared<ifc_tools::VehicleGlobalPositionUtils>();

    std::shared_ptr<ifc_tools::VehicleAttitudeReceiver> _attitude;
    std::shared_ptr<ifc_tools::VehicleAngularVelocityReceiver> _angular_velocity;
    std::shared_ptr<ifc_tools::VehicleLocalPositionReceiver> _local_position;
    std::shared_ptr<ifc_tools::VehicleGlobalPositionReceiver> _global_position;

    std::size_t _attitude_keep = 0;
    std::size_t _angular_velocity_keep = 0;
    std::size_t _local_position_keep = 0;
    std::size_t _global_position_keep = 0;

    rclcpp::Subscription<ifc_msgs::msg::CalibrationResults>::SharedPtr _calibration_results_subscription;
    rclcpp::Subscription<ifc_msgs::msg::StreamInfo>::SharedPtr _stream_info_subscription;

    rclcpp::Publisher<ifc_msgs::msg::AutopilotTelemetry>::SharedPtr _publisher;

    std::optional<ifc_msgs::msg::CalibrationResults> _calibration_results;

    void receive(const ifc_msgs::msg::StreamInfo& stream_info)
    {
        if (!_calibration_results || _calibration_results->delay.status.value != ifc_msgs::msg::Status::VALID)
        {
            return;
        }

        constexpr float nan = std::numeric_limits<float>::quiet_NaN();
        const int64_t timestamp = rclcpp::Time(stream_info.image_timestamp).nanoseconds() + _calibration_results->delay.delay;


        ifc_msgs::msg::AutopilotTelemetry telemetry;
        telemetry.image_timestamp = stream_info.image_timestamp;
        telemetry.attitude.fill(nan);
        telemetry.angular_velocity.fill(nan);
        telemetry.position.fill(nan);
        telemetry.velocity.fill(nan);
        telemetry.ref_lat = std::numeric_limits<double>::quiet_NaN();
        telemetry.ref_lon = std::numeric_limits<double>::quiet_NaN();
        telemetry.ref_alt = nan;
        telemetry.latitude = std::numeric_limits<double>::quiet_NaN();
        telemetry.longitude = std::numeric_limits<double>::quiet_NaN();
        telemetry.altitude = nan;

        if (const auto attitude = _attitude->interpolateAndPrune(timestamp, *_attitude_utils, _attitude_keep))
        {
            telemetry.attitude[0] = static_cast<float>(attitude->q.x());
            telemetry.attitude[1] = static_cast<float>(attitude->q.y());
            telemetry.attitude[2] = static_cast<float>(attitude->q.z());
            telemetry.attitude[3] = static_cast<float>(attitude->q.w());
        }

        if (const auto angular_velocity = _angular_velocity->interpolateAndPrune(timestamp, *_angular_velocity_utils, _angular_velocity_keep))
        {
            std::copy(angular_velocity->omega.begin(), angular_velocity->omega.end(), telemetry.angular_velocity.begin());
        }

        if (const auto local_position = _local_position->interpolateAndPrune(timestamp, *_local_position_utils, _local_position_keep))
        {
            std::copy(local_position->position.begin(), local_position->position.end(), telemetry.position.begin());
            std::copy(local_position->velocity.begin(), local_position->velocity.end(), telemetry.velocity.begin());
            telemetry.ref_lat = local_position->ref_lat;
            telemetry.ref_lon = local_position->ref_lon;
            telemetry.ref_alt = local_position->ref_alt;
        }

        if (const auto global_position = _global_position->interpolateAndPrune(timestamp, *_global_position_utils, _global_position_keep))
        {
            telemetry.latitude = global_position->latitude;
            telemetry.longitude = global_position->longitude;
            telemetry.altitude = global_position->altitude_amsl;
        }

        telemetry.header.stamp = now();
        _publisher->publish(telemetry);
    }
};

}
