#pragma once

#include <array>
#include <optional>
#include <string>

#include "geometry.hpp"
#include "inertial.hpp"
#include "transform.hpp"

namespace oms {

// ---------------------------------------------------------------------------
// Material — surface appearance used by a Visual element.
// ---------------------------------------------------------------------------

struct Material {
    std::string name;

    /// RGBA colour, each channel in [0.0, 1.0]. Default: opaque light grey.
    std::array<float, 4> rgba{0.8f, 0.8f, 0.8f, 1.0f};

    /// Optional path to a texture image (e.g. .png, .jpg).
    std::string texture_path;
};

// ---------------------------------------------------------------------------
// Visual — how a link looks in the renderer.
// ---------------------------------------------------------------------------

struct Visual {
    std::optional<std::string> name;    ///< Optional sub-element identifier
    Transform3D origin;                 ///< Pose relative to the link frame
    Geometry geometry;                  ///< Shape to render
    std::optional<Material> material;   ///< Surface appearance
};

// ---------------------------------------------------------------------------
// Collision — shape used for physics contact detection.
// Often simpler than the visual geometry for performance reasons.
// ---------------------------------------------------------------------------

struct Collision {
    std::optional<std::string> name;    ///< Optional sub-element identifier
    Transform3D origin;                 ///< Pose relative to the link frame
    Geometry geometry;                  ///< Collision shape
};

// ---------------------------------------------------------------------------
// Link — a rigid body in the kinematic tree.
//
// A link has:
//   - A unique name (used as key throughout the RobotModel)
//   - Optional inertial properties (required for dynamics simulation)
//   - Optional visual element (required for rendering)
//   - Optional collision element (required for physics)
//
// The root link has no parent joint. All other links have exactly one.
// ---------------------------------------------------------------------------

struct Link {
    std::string name;                       ///< Unique identifier within the model

    std::optional<Inertial>   inertial;     ///< Mass + inertia + CoM
    std::optional<Visual>     visual;       ///< Rendering geometry + material
    std::optional<Collision>  collision;    ///< Physics collision geometry
};

} // namespace oms
