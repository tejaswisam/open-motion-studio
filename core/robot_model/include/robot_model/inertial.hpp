#pragma once

#include "transform.hpp"

namespace oms {

// ---------------------------------------------------------------------------
// Inertial properties of a link.
// All values use SI units (kg, metres, kg·m²).
// ---------------------------------------------------------------------------

/// Symmetric 3×3 inertia tensor expressed in the link's inertial frame.
/// Only the upper-triangular elements are stored (tensor is symmetric).
///
///   | ixx  ixy  ixz |
///   | ixy  iyy  iyz |
///   | ixz  iyz  izz |
struct InertiaTensor {
    double ixx{0.0};
    double ixy{0.0};
    double ixz{0.0};
    double iyy{0.0};
    double iyz{0.0};
    double izz{0.0};
};

/// Full inertial description of a link:
///   - mass in kg
///   - inertia tensor in kg·m²
///   - origin: pose of the link's centre of mass relative to the link frame
struct Inertial {
    double mass{0.0};           ///< kg
    InertiaTensor inertia;      ///< kg·m²
    Transform3D origin;         ///< CoM pose relative to link frame
};

} // namespace oms
