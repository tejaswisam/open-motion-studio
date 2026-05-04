#pragma once

#include <optional>
#include <stdexcept>
#include <string>

#include "transform.hpp"

namespace oms {

// ---------------------------------------------------------------------------
// JointType — matches the URDF joint type spec.
// ---------------------------------------------------------------------------

enum class JointType {
    Fixed,       ///< No relative motion. Welds two links together.
    Revolute,    ///< Rotation about a single axis, with defined limits.
    Prismatic,   ///< Translation along a single axis, with defined limits.
    Continuous,  ///< Rotation about a single axis, unlimited (e.g. wheels).
    Floating,    ///< 6-DOF free motion (unconstrained).
    Planar,      ///< Motion in a plane perpendicular to the axis.
};

/// Convert JointType to its URDF string name.
const char* to_string(JointType type) noexcept;

/// Parse a URDF joint type string. Throws std::invalid_argument on unknown input.
JointType joint_type_from_string(const std::string& s);

// ---------------------------------------------------------------------------
// JointAxis — unit vector defining the axis of motion.
// Expressed in the joint's own frame.
// Default: z-axis [0, 0, 1] (ROS convention).
// ---------------------------------------------------------------------------

struct JointAxis {
    double x{0.0};
    double y{0.0};
    double z{1.0};
};

// ---------------------------------------------------------------------------
// JointLimits — motion bounds (required for Revolute and Prismatic).
// ---------------------------------------------------------------------------

struct JointLimits {
    double lower{0.0};      ///< Lower position limit (radians or metres)
    double upper{0.0};      ///< Upper position limit (radians or metres)
    double effort{0.0};     ///< Maximum force/torque (N or N·m)
    double velocity{0.0};   ///< Maximum speed (rad/s or m/s)
};

// ---------------------------------------------------------------------------
// JointDynamics — physical damping and static friction coefficients.
// ---------------------------------------------------------------------------

struct JointDynamics {
    double damping{0.0};    ///< Viscous damping coefficient (N·m·s/rad or N·s/m)
    double friction{0.0};   ///< Static friction value (N·m or N)
};

// ---------------------------------------------------------------------------
// Joint — a kinematic connection between two links.
//
// Every joint:
//   - Has a unique name
//   - Has exactly one parent link and one child link
//   - Has a pose (origin) expressed relative to the parent link frame
//   - Has a type that defines the degrees of freedom it allows
//
// Revolute / Prismatic joints MUST provide limits.
// Continuous joints MAY omit limits (effort + velocity still recommended).
// Fixed and Floating joints ignore axis and limits.
// ---------------------------------------------------------------------------

struct Joint {
    std::string name;                           ///< Unique identifier within the model

    JointType type{JointType::Fixed};           ///< Kinematic type

    std::string parent_link;                    ///< Name of the parent (upstream) link
    std::string child_link;                     ///< Name of the child (downstream) link

    Transform3D origin;                         ///< Joint frame pose relative to parent link frame

    JointAxis axis;                             ///< Axis of motion (ignored for Fixed/Floating)

    std::optional<JointLimits>   limits;        ///< Required for Revolute/Prismatic
    std::optional<JointDynamics> dynamics;      ///< Optional damping + friction
};

} // namespace oms
