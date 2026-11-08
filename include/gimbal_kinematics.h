#pragma once

#include <array>
#include <cmath>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

namespace apm_bridge
{
    using GimbalJointAngles = std::array<double, 3>; // FRD roll, pitch, yaw in radians.
    using GimbalAxes = std::array<bool, 3>;

    // Aligned frames at encoder zero. The ZXY chain (yaw * roll * pitch) matches
    // both the physical joint order and the vendor camera-frame euler order
    // Z -> Y -> X from protocol appendix 2: phi, theta and psi are right-hand
    // rotations about the LFD camera axes X (left, pitch), Y (front, roll) and
    // Z (down, yaw) respectively, and the same chain expressed in the carrier
    // FRD basis is Rz(yaw)*Rx(roll)*Ry(pitch)
    // (theta -> +roll, -phi -> pitch-up, psi -> yaw). See
    // doc/gimbal_protocol_verified.md section 3.1 (2026-11-08 revision).
    inline tf2::Quaternion gimbalCameraToWorld(const tf2::Quaternion &body_to_world,
                                              const GimbalJointAngles &joints, const GimbalAxes &axes)
    {
        const auto yaw = tf2::Quaternion(tf2::Vector3(0, 0, 1), axes[2] ? -joints[2] : 0);
        const auto roll = tf2::Quaternion(tf2::Vector3(1, 0, 0), axes[0] ? joints[0] : 0);
        const auto pitch = tf2::Quaternion(tf2::Vector3(0, 1, 0), axes[1] ? -joints[1] : 0);
        return (body_to_world * yaw * roll * pitch).normalized();
    }

    // ZXY virtual encoders avoid the +/-90-degree ZYX pitch fold (ZXY is also
    // the vendor euler order, see gimbalCameraToWorld above).
    inline GimbalJointAngles gimbalJointAngles(const tf2::Quaternion &camera_to_body, const GimbalAxes &axes)
    {
        const tf2::Matrix3x3 rotation(camera_to_body.normalized());
        if (!axes[2])
            return {axes[0] ? std::atan2(rotation[2][1], rotation[1][1]) : 0,
                    axes[1] ? std::atan2(-rotation[0][2], rotation[0][0]) : 0, 0};
        const double cos_roll = std::hypot(rotation[2][0], rotation[2][2]);
        const bool singular = cos_roll < 1e-6;
        const double roll = std::atan2(rotation[2][1], cos_roll);
        const double pitch = singular ? 0 : std::atan2(rotation[2][0], rotation[2][2]);
        const double yaw = singular ? -std::atan2(rotation[1][0], rotation[0][0]) :
            std::atan2(rotation[0][1], rotation[1][1]);
        return {axes[0] ? roll : 0, axes[1] ? pitch : 0, axes[2] ? yaw : 0};
    }
}
