#pragma once

namespace oms {

/// Rigid-body transform in 3D space.
/// Convention: ROS standard — xyz in metres, rpy (roll-pitch-yaw) in radians.
/// All values default to zero (identity transform).
struct Transform3D {
    // Translation (metres)
    double x{0.0};
    double y{0.0};
    double z{0.0};

    // Rotation — intrinsic Euler angles RPY (radians)
    double roll{0.0};
    double pitch{0.0};
    double yaw{0.0};

    /// Returns an identity transform (all zeros).
    static Transform3D identity() noexcept { return {}; }

    bool operator==(const Transform3D& o) const noexcept {
        return x == o.x && y == o.y && z == o.z &&
               roll == o.roll && pitch == o.pitch && yaw == o.yaw;
    }

    bool operator!=(const Transform3D& o) const noexcept { return !(*this == o); }
};

} // namespace oms
