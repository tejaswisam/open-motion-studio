#pragma once

#include <string>
#include <variant>

namespace oms {

// ---------------------------------------------------------------------------
// Geometry primitives — used by Visual and Collision elements of a Link.
// These mirror the URDF geometry spec but are decoupled from XML parsing.
// ---------------------------------------------------------------------------

/// Axis-aligned box defined by its full extents along each axis (metres).
struct BoxGeometry {
    double size_x{1.0};
    double size_y{1.0};
    double size_z{1.0};
};

/// Cylinder aligned with its local z-axis.
struct CylinderGeometry {
    double radius{0.5};   ///< metres
    double length{1.0};   ///< metres (full height)
};

/// Sphere centred at the geometry origin.
struct SphereGeometry {
    double radius{0.5};   ///< metres
};

/// External mesh file.
/// Supported formats (handled by downstream loaders): .stl, .dae, .gltf
struct MeshGeometry {
    std::string filepath;       ///< Absolute or project-relative path
    double scale_x{1.0};        ///< Uniform or per-axis scale applied at load time
    double scale_y{1.0};
    double scale_z{1.0};
};

/// Discriminated union of all geometry types.
/// Use std::visit / std::get to dispatch on the active type.
using Geometry = std::variant<BoxGeometry, CylinderGeometry, SphereGeometry, MeshGeometry>;

} // namespace oms
