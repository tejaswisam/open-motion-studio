#include "robot_model/robot_model.hpp"

#include <cassert>
#include <iostream>

// ---------------------------------------------------------------------------
// Builds a minimal 3-link robot arm:
//
//   base_link
//       └── joint_1 (revolute, z-axis) ──▶ shoulder_link
//               └── joint_2 (revolute, z-axis) ──▶ elbow_link
//
// Demonstrates the full mutation + query API.
// ---------------------------------------------------------------------------

int main() {
  oms::RobotModel robot("simple_arm");

  // -----------------------------------------------------------------------
  // Links
  // -----------------------------------------------------------------------

  { // base_link — fixed to ground; no inertial needed for kinematics-only
    oms::Link base;
    base.name = "base_link";

    oms::Visual vis;
    vis.origin = oms::Transform3D::identity();
    vis.geometry = oms::BoxGeometry{0.1, 0.1, 0.05};
    vis.material = oms::Material{"grey", {0.7f, 0.7f, 0.7f, 1.0f}, ""};
    base.visual = vis;

    oms::Collision col;
    col.origin = oms::Transform3D::identity();
    col.geometry = oms::BoxGeometry{0.1, 0.1, 0.05};
    base.collision = col;

    robot.add_link(std::move(base));
  }

  { // shoulder_link — upper arm
    oms::Link shoulder;
    shoulder.name = "shoulder_link";

    // Inertial
    oms::Inertial inertial;
    inertial.mass = 1.2;
    inertial.inertia = {0.01, 0.0, 0.0, 0.01, 0.0, 0.02};
    inertial.origin.z = 0.1; // CoM at mid-link
    shoulder.inertial = inertial;

    // Visual — cylinder representing the upper arm
    oms::Visual vis;
    vis.geometry = oms::CylinderGeometry{0.03, 0.2};
    vis.origin.z = 0.1;
    vis.material = oms::Material{"arm_material", {0.2f, 0.6f, 0.9f, 1.0f}, ""};
    shoulder.visual = vis;

    oms::Collision col;
    col.geometry = oms::CylinderGeometry{0.03, 0.2};
    col.origin.z = 0.1;
    shoulder.collision = col;

    robot.add_link(std::move(shoulder));
  }

  { // elbow_link — forearm
    oms::Link elbow;
    elbow.name = "elbow_link";

    oms::Inertial inertial;
    inertial.mass = 0.8;
    inertial.inertia = {0.005, 0.0, 0.0, 0.005, 0.0, 0.01};
    inertial.origin.z = 0.075;
    elbow.inertial = inertial;

    oms::Visual vis;
    vis.geometry = oms::CylinderGeometry{0.025, 0.15};
    vis.origin.z = 0.075;
    vis.material = oms::Material{"arm_material", {0.2f, 0.6f, 0.9f, 1.0f}, ""};
    elbow.visual = vis;

    oms::Collision col;
    col.geometry = oms::CylinderGeometry{0.025, 0.15};
    col.origin.z = 0.075;
    elbow.collision = col;

    robot.add_link(std::move(elbow));
  }

  // -----------------------------------------------------------------------
  // Joints
  // -----------------------------------------------------------------------

  { // base → shoulder (revolute about z)
    oms::Joint j;
    j.name = "joint_1";
    j.type = oms::JointType::Revolute;
    j.parent_link = "base_link";
    j.child_link = "shoulder_link";
    j.origin.z = 0.05; // mount 5 cm above base top face
    j.axis = {0.0, 0.0, 1.0};

    oms::JointLimits lim;
    lim.lower = -1.5708; // -90°
    lim.upper = 1.5708;  //  90°
    lim.effort = 50.0;
    lim.velocity = 1.0;
    j.limits = lim;

    j.dynamics = oms::JointDynamics{0.5, 0.1};

    robot.add_joint(std::move(j));
  }

  { // shoulder → elbow (revolute about z)
    oms::Joint j;
    j.name = "joint_2";
    j.type = oms::JointType::Revolute;
    j.parent_link = "shoulder_link";
    j.child_link = "elbow_link";
    j.origin.z = 0.2; // tip of upper arm
    j.axis = {0.0, 0.0, 1.0};

    oms::JointLimits lim;
    lim.lower = -2.0944; // -120°
    lim.upper = 2.0944;  //  120°
    lim.effort = 30.0;
    lim.velocity = 1.5;
    j.limits = lim;

    j.dynamics = oms::JointDynamics{0.3, 0.05};

    robot.add_joint(std::move(j));
  }

  // -----------------------------------------------------------------------
  // Root + validation
  // -----------------------------------------------------------------------

  robot.set_root_link("base_link");
  robot.validate(); // throws on any structural error

  // -----------------------------------------------------------------------
  // Print summary
  // -----------------------------------------------------------------------

  std::cout << "Robot: " << robot.name() << "\n";
  std::cout << "  Links  (" << robot.num_links() << "): ";
  for (const auto &l : robot.link_names())
    std::cout << l << "  ";
  std::cout << "\n";

  std::cout << "  Joints (" << robot.num_joints() << "): ";
  for (const auto &j : robot.joint_names())
    std::cout << j << "  ";
  std::cout << "\n";

  std::cout << "\nKinematic chain (DFS from root):\n";
  for (const auto &l : robot.kinematic_chain_dfs()) {
    auto pj = robot.parent_joint_of(l);
    if (pj) {
      const auto &joint = robot.get_joint(*pj);
      std::cout << "  [" << oms::to_string(joint.type) << "] "
                << joint.parent_link << " --(" << joint.name << ")--> " << l
                << "\n";
    } else {
      std::cout << "  (root) " << l << "\n";
    }
  }

  std::cout << "\nAll checks passed.\n";
  return 0;
}
