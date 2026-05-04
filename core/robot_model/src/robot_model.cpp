#include "robot_model/robot_model.hpp"

#include <algorithm>
#include <sstream>
#include <stdexcept>

namespace oms {

// ---------------------------------------------------------------------------
// JointType helpers (defined here to avoid header-only linking issues)
// ---------------------------------------------------------------------------

const char* to_string(JointType type) noexcept {
    switch (type) {
        case JointType::Fixed:      return "fixed";
        case JointType::Revolute:   return "revolute";
        case JointType::Prismatic:  return "prismatic";
        case JointType::Continuous: return "continuous";
        case JointType::Floating:   return "floating";
        case JointType::Planar:     return "planar";
    }
    return "unknown";
}

JointType joint_type_from_string(const std::string& s) {
    if (s == "fixed")      return JointType::Fixed;
    if (s == "revolute")   return JointType::Revolute;
    if (s == "prismatic")  return JointType::Prismatic;
    if (s == "continuous") return JointType::Continuous;
    if (s == "floating")   return JointType::Floating;
    if (s == "planar")     return JointType::Planar;
    throw std::invalid_argument("Unknown joint type: '" + s + "'");
}

// ---------------------------------------------------------------------------
// RobotModel — construction
// ---------------------------------------------------------------------------

RobotModel::RobotModel(std::string name)
    : name_(std::move(name)) {}

// ---------------------------------------------------------------------------
// Mutation
// ---------------------------------------------------------------------------

void RobotModel::add_link(Link link) {
    if (links_.count(link.name)) {
        throw std::invalid_argument(
            "RobotModel: link '" + link.name + "' already exists.");
    }
    links_.emplace(link.name, std::move(link));
}

void RobotModel::add_joint(Joint joint) {
    if (joints_.count(joint.name)) {
        throw std::invalid_argument(
            "RobotModel: joint '" + joint.name + "' already exists.");
    }
    if (!links_.count(joint.parent_link)) {
        throw std::invalid_argument(
            "RobotModel: joint '" + joint.name +
            "' references unknown parent link '" + joint.parent_link + "'.");
    }
    if (!links_.count(joint.child_link)) {
        throw std::invalid_argument(
            "RobotModel: joint '" + joint.name +
            "' references unknown child link '" + joint.child_link + "'.");
    }
    joints_.emplace(joint.name, std::move(joint));
}

void RobotModel::set_root_link(const std::string& link_name) {
    if (!links_.count(link_name)) {
        throw std::invalid_argument(
            "RobotModel: cannot set root — link '" + link_name + "' not found.");
    }
    root_link_name_ = link_name;
}

// ---------------------------------------------------------------------------
// Accessors
// ---------------------------------------------------------------------------

const std::string& RobotModel::name() const noexcept { return name_; }

const std::string& RobotModel::root_link_name() const {
    if (root_link_name_.empty()) {
        throw std::runtime_error("RobotModel: root link has not been set.");
    }
    return root_link_name_;
}

const Link& RobotModel::get_link(const std::string& n) const {
    auto it = links_.find(n);
    if (it == links_.end())
        throw std::out_of_range("RobotModel: link '" + n + "' not found.");
    return it->second;
}

Link& RobotModel::get_link(const std::string& n) {
    return const_cast<Link&>(static_cast<const RobotModel*>(this)->get_link(n));
}

const Joint& RobotModel::get_joint(const std::string& n) const {
    auto it = joints_.find(n);
    if (it == joints_.end())
        throw std::out_of_range("RobotModel: joint '" + n + "' not found.");
    return it->second;
}

Joint& RobotModel::get_joint(const std::string& n) {
    return const_cast<Joint&>(static_cast<const RobotModel*>(this)->get_joint(n));
}

bool RobotModel::has_link(const std::string& n)  const noexcept { return links_.count(n)  > 0; }
bool RobotModel::has_joint(const std::string& n) const noexcept { return joints_.count(n) > 0; }

std::vector<std::string> RobotModel::link_names() const {
    std::vector<std::string> out;
    out.reserve(links_.size());
    for (const auto& [k, _] : links_) out.push_back(k);
    return out;
}

std::vector<std::string> RobotModel::joint_names() const {
    std::vector<std::string> out;
    out.reserve(joints_.size());
    for (const auto& [k, _] : joints_) out.push_back(k);
    return out;
}

std::size_t RobotModel::num_links()  const noexcept { return links_.size();  }
std::size_t RobotModel::num_joints() const noexcept { return joints_.size(); }

// ---------------------------------------------------------------------------
// Tree Queries
// ---------------------------------------------------------------------------

std::vector<std::string> RobotModel::child_joints_of(const std::string& link_name) const {
    std::vector<std::string> result;
    for (const auto& [jname, j] : joints_) {
        if (j.parent_link == link_name)
            result.push_back(jname);
    }
    return result;
}

std::optional<std::string> RobotModel::parent_joint_of(const std::string& link_name) const {
    for (const auto& [jname, j] : joints_) {
        if (j.child_link == link_name)
            return jname;
    }
    return std::nullopt;
}

std::vector<std::string> RobotModel::child_links_of(const std::string& link_name) const {
    std::vector<std::string> result;
    for (const auto& [_, j] : joints_) {
        if (j.parent_link == link_name)
            result.push_back(j.child_link);
    }
    return result;
}

std::vector<std::string> RobotModel::kinematic_chain_dfs() const {
    std::vector<std::string> out;
    std::vector<std::string> visited;
    if (root_link_name_.empty()) return out;
    dfs_impl(root_link_name_, out, visited);
    return out;
}

void RobotModel::dfs_impl(const std::string& link_name,
                           std::vector<std::string>& out,
                           std::vector<std::string>& visited) const {
    // Guard against cycles (should not exist in a valid model)
    if (std::find(visited.begin(), visited.end(), link_name) != visited.end())
        return;

    visited.push_back(link_name);
    out.push_back(link_name);

    for (const std::string& child : child_links_of(link_name)) {
        dfs_impl(child, out, visited);
    }
}

// ---------------------------------------------------------------------------
// Validation
// ---------------------------------------------------------------------------

void RobotModel::validate() const {
    // 1. Root must be set
    if (root_link_name_.empty()) {
        throw std::runtime_error("RobotModel::validate: root link is not set.");
    }
    if (!links_.count(root_link_name_)) {
        throw std::runtime_error(
            "RobotModel::validate: root link '" + root_link_name_ + "' is not registered.");
    }

    // 2. Revolute/Prismatic joints must have limits
    for (const auto& [jname, j] : joints_) {
        if ((j.type == JointType::Revolute || j.type == JointType::Prismatic)
            && !j.limits) {
            throw std::runtime_error(
                "RobotModel::validate: joint '" + jname +
                "' is revolute/prismatic but has no limits defined.");
        }
    }

    // 3. Every link must be reachable from root (no orphans)
    const auto reachable = kinematic_chain_dfs();

    for (const auto& [lname, _] : links_) {
        bool found = std::find(reachable.begin(), reachable.end(), lname) != reachable.end();
        if (!found) {
            throw std::runtime_error(
                "RobotModel::validate: link '" + lname +
                "' is not reachable from the root link '" + root_link_name_ + "'.");
        }
    }
}

} // namespace oms
