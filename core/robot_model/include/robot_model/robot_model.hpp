#pragma once

#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "joint.hpp"
#include "link.hpp"

namespace oms {

// ---------------------------------------------------------------------------
// RobotModel — the core internal representation of a robot.
//
// Acts as the Single Source of Truth (SSOT) for the kinematic + dynamic
// structure of any robot in OMS, regardless of its origin (URDF, STEP, etc.).
//
// Responsibilities:
//   - Owns all Link and Joint instances (by value, keyed by name).
//   - Maintains the kinematic tree (parent/child relationships).
//   - Provides tree traversal and kinematic chain queries.
//   - Enforces structural validity (no orphans, unique names, root set).
//
// This class is intentionally decoupled from:
//   - XML / URDF parsing (handled upstream)
//   - Physics simulation (ROBOTICSCORE subscribes to this model)
//   - Rendering (RENDERER reads from this model via an adapter)
// ---------------------------------------------------------------------------

class RobotModel {
public:
    /// Construct an empty model with the given robot name.
    explicit RobotModel(std::string name);

    // -----------------------------------------------------------------------
    // Mutation API
    // -----------------------------------------------------------------------

    /// Add a link to the model. Throws std::invalid_argument if name already exists.
    void add_link(Link link);

    /// Add a joint to the model. Throws std::invalid_argument if:
    ///   - name already exists
    ///   - parent_link or child_link are not registered links
    void add_joint(Joint joint);

    /// Designate the root link (must already be added via add_link).
    /// Throws std::invalid_argument if the link is not registered.
    void set_root_link(const std::string& link_name);

    // -----------------------------------------------------------------------
    // Accessors
    // -----------------------------------------------------------------------

    const std::string& name() const noexcept;
    const std::string& root_link_name() const;

    /// Look up a link by name. Throws std::out_of_range if not found.
    const Link&  get_link(const std::string& name) const;
          Link&  get_link(const std::string& name);

    /// Look up a joint by name. Throws std::out_of_range if not found.
    const Joint& get_joint(const std::string& name) const;
          Joint& get_joint(const std::string& name);

    bool has_link(const std::string& name)  const noexcept;
    bool has_joint(const std::string& name) const noexcept;

    /// Returns all registered link names (insertion order not guaranteed).
    std::vector<std::string> link_names()  const;

    /// Returns all registered joint names (insertion order not guaranteed).
    std::vector<std::string> joint_names() const;

    std::size_t num_links()  const noexcept;
    std::size_t num_joints() const noexcept;

    // -----------------------------------------------------------------------
    // Tree Queries
    // -----------------------------------------------------------------------

    /// Returns names of all joints whose parent_link == link_name.
    std::vector<std::string> child_joints_of(const std::string& link_name) const;

    /// Returns name of the joint whose child_link == link_name, if any.
    /// (Each non-root link has exactly one parent joint.)
    std::optional<std::string> parent_joint_of(const std::string& link_name) const;

    /// Returns names of all child links of link_name (via their connecting joints).
    std::vector<std::string> child_links_of(const std::string& link_name) const;

    /// Returns the depth-first ordered list of link names starting from root.
    /// Useful for forward kinematics traversal.
    std::vector<std::string> kinematic_chain_dfs() const;

    // -----------------------------------------------------------------------
    // Validation
    // -----------------------------------------------------------------------

    /// Validates the model structure. Throws std::runtime_error describing the
    /// first violation found. Checks:
    ///   - Root link is set and registered
    ///   - Every non-root link is reachable from root
    ///   - No link is its own ancestor (cycle detection)
    ///   - Revolute / Prismatic joints have limits defined
    void validate() const;

private:
    std::string name_;
    std::string root_link_name_;

    std::unordered_map<std::string, Link>  links_;
    std::unordered_map<std::string, Joint> joints_;

    /// Internal DFS helper — appends link names to `out` in traversal order.
    void dfs_impl(const std::string& link_name,
                  std::vector<std::string>& out,
                  std::vector<std::string>& visited) const;
};

} // namespace oms
