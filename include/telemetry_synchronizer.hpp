#pragma once

#include <rclcpp/rclcpp.hpp>

namespace telemetry_synchronizer
{

class TelemetrySynchronizer : public rclcpp::Node
{
public:
    TelemetrySynchronizer() : rclcpp::Node("telemetry_synchronizer")
    {
    }
};

}
