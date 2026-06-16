/**
 * pick_place_mtc.cpp
 *
 * MoveIt Task Constructor pick-and-place for arm6dof.
 *
 * Exposes a ROS 2 service /mtc_pick_place (robot_interfaces/srv/Pick is reused
 * for the pick target; place target is declared as node parameters).
 *
 * Robot geometry (matches moveit_config_2 SRDF):
 *   arm group   : joint1-6, chain base_link → link6
 *   gripper     : finger_left_joint  (named states: "open" / "close")
 *   end-effector: "gripper", parent link6
 *
 * IK frame offset: finger midpoint is 0.12 m beyond link6 along the arm axis
 *   link6 → gripper_base : 0.06 m  (gripper_base_joint origin z=0.06)
 *   gripper_base → finger midpoint : 0.06 m (finger origin z=0.02, half-height 0.04)
 */

#include <rclcpp/rclcpp.hpp>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/stages/generate_place_pose.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

#include <robot_interfaces/srv/pick.hpp>

#include <tf2_eigen/tf2_eigen.hpp>

namespace mtc = moveit::task_constructor;

// ── Constants matching the SRDF and URDF geometry ─────────────────────────────
static constexpr char ARM_GROUP[]    = "arm";
static constexpr char HAND_GROUP[]   = "gripper";
static constexpr char EEF_NAME[]     = "gripper";
static constexpr char HAND_FRAME[]   = "gripper_base";  // EEF parent link (fixed to link6)
static constexpr char WORLD_FRAME[]  = "world";
static constexpr char OBJECT_NAME[]  = "pick_object";

// Distance from link6 (arm group tip) to the finger-pad midpoint, along
// the local arm Z-axis.  Used as the IK-frame offset for ComputeIK.
static constexpr double GRASP_OFFSET_Z = 0.12;   // 0.06 (link6→gripper_base) + 0.06 (→finger mid)

// Approach / lift / retreat distances (metres)
static constexpr double APPROACH_MIN  = 0.05;
static constexpr double APPROACH_MAX  = 0.15;
static constexpr double LIFT_MIN      = 0.05;
static constexpr double LIFT_MAX      = 0.20;

// Object geometry (0.06 m cube)
static constexpr double OBJ_HALF = 0.03;


class MtcPickPlace : public rclcpp::Node
{
public:
  explicit MtcPickPlace(const rclcpp::NodeOptions & opts = rclcpp::NodeOptions())
  : Node("mtc_pick_place", opts)
  {
    // ── ROS 2 parameters ──────────────────────────────────────────────────
    declare_parameter("pick_x",  0.50);
    declare_parameter("pick_y",  0.00);
    declare_parameter("pick_z",  0.15);
    declare_parameter("place_x", 0.35);
    declare_parameter("place_y", 0.50);
    declare_parameter("place_z", 0.07);

    // ── Service ───────────────────────────────────────────────────────────
    // Reuse Pick.srv: target_pose = pick pose, approach_height unused (MTC handles it).
    // Place pose is always taken from parameters.
    service_ = create_service<robot_interfaces::srv::Pick>(
      "/mtc_pick_place",
      [this](const std::shared_ptr<robot_interfaces::srv::Pick::Request>  req,
             std::shared_ptr<robot_interfaces::srv::Pick::Response>        res) {
        runTask(req->target_pose.position.x,
                req->target_pose.position.y,
                req->target_pose.position.z,
                get_parameter("place_x").as_double(),
                get_parameter("place_y").as_double(),
                get_parameter("place_z").as_double(),
                res);
      });

    RCLCPP_INFO(get_logger(),
      "MTC pick-and-place ready.  Call /mtc_pick_place with pick pose "
      "(place pose from ~place_x/y/z params, default: (0.35, 0.50, 0.07)).");
  }

private:
  // ── Spawn the pick object in the MoveIt planning scene ──────────────────
  void spawnObject(double x, double y, double z)
  {
    moveit::planning_interface::PlanningSceneInterface psi;

    moveit_msgs::msg::CollisionObject obj;
    obj.id                = OBJECT_NAME;
    obj.header.frame_id   = WORLD_FRAME;
    obj.operation         = moveit_msgs::msg::CollisionObject::ADD;

    shape_msgs::msg::SolidPrimitive box;
    box.type = shape_msgs::msg::SolidPrimitive::BOX;
    box.dimensions = {0.06, 0.06, 0.06};
    obj.primitives.push_back(box);

    geometry_msgs::msg::Pose pose;
    pose.position.x    = x;
    pose.position.y    = y;
    pose.position.z    = z;
    pose.orientation.w = 1.0;
    obj.primitive_poses.push_back(pose);

    psi.applyCollisionObject(obj);
    RCLCPP_INFO(get_logger(), "Spawned '%s' at (%.3f, %.3f, %.3f) in planning scene.",
                OBJECT_NAME, x, y, z);
  }

  // ── Remove the object from the planning scene ────────────────────────────
  void removeObject()
  {
    moveit::planning_interface::PlanningSceneInterface psi;
    psi.removeCollisionObjects({OBJECT_NAME});
  }

  // ── Build the full MTC task ──────────────────────────────────────────────
  mtc::Task buildTask(double lx, double ly, double lz)
  {
    mtc::Task task;
    task.stages()->setName("pick_and_place");
    task.loadRobotModel(shared_from_this());

    // Solvers
    auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(shared_from_this());
    auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian_planner->setMaxVelocityScalingFactor(0.3);
    cartesian_planner->setMaxAccelerationScalingFactor(0.3);
    cartesian_planner->setStepSize(0.01);

    auto joint_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

    // Task-level properties (propagated to child stages via configureInitFrom)
    task.setProperty("group",    ARM_GROUP);
    task.setProperty("eef",      EEF_NAME);
    task.setProperty("hand",     HAND_GROUP);
    task.setProperty("hand_grasping_frame", HAND_FRAME);
    task.setProperty("ik_frame", HAND_FRAME);

    // IK frame offset: place the finger-pad midpoint at the grasp target
    Eigen::Isometry3d grasp_tf = Eigen::Isometry3d::Identity();
    grasp_tf.translation().z() = GRASP_OFFSET_Z;

    // ── Stage 0: Current state ─────────────────────────────────────────────
    mtc::Stage * current_state_ptr = nullptr;
    {
      auto stage = std::make_unique<mtc::stages::CurrentState>("current state");
      current_state_ptr = stage.get();
      task.add(std::move(stage));
    }

    // ── Stage 1: Open gripper ──────────────────────────────────────────────
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("open gripper", joint_planner);
      stage->setGroup(HAND_GROUP);
      stage->setGoal("open");
      task.add(std::move(stage));
    }

    // ── Stage 2: Move to pre-grasp region ─────────────────────────────────
    {
      auto stage = std::make_unique<mtc::stages::Connect>(
        "move to pick",
        mtc::stages::Connect::GroupPlannerVector{{ARM_GROUP, sampling_planner}});
      stage->setTimeout(10.0);
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      task.add(std::move(stage));
    }

    // ── Stage 3: Pick container ────────────────────────────────────────────
    auto pick_container = std::make_unique<mtc::SerialContainer>("pick object");
    task.properties().exposeTo(pick_container->properties(),
                               {"eef", "group", "hand", "ik_frame"});
    pick_container->properties().configureInitFrom(mtc::Stage::PARENT,
                                                   {"eef", "group", "hand", "ik_frame"});

    // 3a. Approach object (move -Z in world frame)
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("approach object",
                                                                cartesian_planner);
      stage->properties().set("marker_ns", "approach");
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      stage->setMinMaxDistance(APPROACH_MIN, APPROACH_MAX);

      geometry_msgs::msg::Vector3Stamped dir;
      dir.header.frame_id = WORLD_FRAME;
      dir.vector.z = -1.0;
      stage->setDirection(dir);
      pick_container->insert(std::move(stage));
    }

    // 3b. Generate grasp pose + IK
    {
      auto grasp_stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp");
      grasp_stage->properties().configureInitFrom(mtc::Stage::PARENT);
      grasp_stage->setPreGraspPose("open");
      grasp_stage->setObject(OBJECT_NAME);
      grasp_stage->setAngleDelta(M_PI / 8.0);    // try 8 rotations around Z
      grasp_stage->setMonitoredStage(current_state_ptr);

      auto ik_wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp IK",
                                                                   std::move(grasp_stage));
      ik_wrapper->setMaxIKSolutions(8);
      ik_wrapper->setMinSolutionDistance(1.0);
      ik_wrapper->setIKFrame(grasp_tf, HAND_FRAME);
      ik_wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
      ik_wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
      pick_container->insert(std::move(ik_wrapper));
    }

    // 3c. Allow collision between gripper fingers and object
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
        "allow collision (fingers, object)");
      stage->allowCollisions(OBJECT_NAME,
        task.getRobotModel()
            ->getJointModelGroup(HAND_GROUP)
            ->getLinkModelNamesWithCollisionGeometry(),
        true);
      pick_container->insert(std::move(stage));
    }

    // 3d. Close gripper (grasp)
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("close gripper", joint_planner);
      stage->setGroup(HAND_GROUP);
      stage->setGoal("close");
      pick_container->insert(std::move(stage));
    }

    // 3e. Attach object to the gripper in the planning scene
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      stage->attachObject(OBJECT_NAME, HAND_FRAME);
      pick_container->insert(std::move(stage));
    }

    // 3f. Lift object (+Z in world frame)
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("lift object",
                                                                cartesian_planner);
      stage->properties().set("marker_ns", "lift");
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      stage->setMinMaxDistance(LIFT_MIN, LIFT_MAX);

      geometry_msgs::msg::Vector3Stamped dir;
      dir.header.frame_id = WORLD_FRAME;
      dir.vector.z = 1.0;
      stage->setDirection(dir);
      pick_container->insert(std::move(stage));
    }

    task.add(std::move(pick_container));

    // ── Stage 4: Move to pre-place region ─────────────────────────────────
    {
      auto stage = std::make_unique<mtc::stages::Connect>(
        "move to place",
        mtc::stages::Connect::GroupPlannerVector{
          {ARM_GROUP,  sampling_planner},
          {HAND_GROUP, joint_planner}});
      stage->setTimeout(10.0);
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      task.add(std::move(stage));
    }

    // ── Stage 5: Place container ───────────────────────────────────────────
    auto place_container = std::make_unique<mtc::SerialContainer>("place object");
    task.properties().exposeTo(place_container->properties(),
                               {"eef", "group", "hand", "ik_frame"});
    place_container->properties().configureInitFrom(mtc::Stage::PARENT,
                                                    {"eef", "group", "hand", "ik_frame"});

    // 5a. Generate place pose + IK
    {
      auto place_stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place");
      place_stage->properties().configureInitFrom(mtc::Stage::PARENT);
      place_stage->properties().set("marker_ns", "place_pose");
      place_stage->setObject(OBJECT_NAME);

      geometry_msgs::msg::PoseStamped place_target;
      place_target.header.frame_id = WORLD_FRAME;
      place_target.pose.position.x    = lx;
      place_target.pose.position.y    = ly;
      place_target.pose.position.z    = lz + OBJ_HALF;  // centre above landing surface
      place_target.pose.orientation.w = 1.0;
      place_stage->setPose(place_target);
      place_stage->setMonitoredStage(task.stages()->findChild("pick object"));

      auto ik_wrapper = std::make_unique<mtc::stages::ComputeIK>("place IK",
                                                                   std::move(place_stage));
      ik_wrapper->setMaxIKSolutions(4);
      ik_wrapper->setMinSolutionDistance(1.0);
      ik_wrapper->setIKFrame(grasp_tf, HAND_FRAME);
      ik_wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
      ik_wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
      place_container->insert(std::move(ik_wrapper));
    }

    // 5b. Lower object (-Z)
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("lower object",
                                                                cartesian_planner);
      stage->properties().set("marker_ns", "lower");
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      stage->setMinMaxDistance(APPROACH_MIN, APPROACH_MAX);

      geometry_msgs::msg::Vector3Stamped dir;
      dir.header.frame_id = WORLD_FRAME;
      dir.vector.z = -1.0;
      stage->setDirection(dir);
      place_container->insert(std::move(stage));
    }

    // 5c. Open gripper (release)
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("open gripper", joint_planner);
      stage->setGroup(HAND_GROUP);
      stage->setGoal("open");
      place_container->insert(std::move(stage));
    }

    // 5d. Forbid collision (fingers, object) — object is now released
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
        "forbid collision (fingers, object)");
      stage->allowCollisions(OBJECT_NAME,
        task.getRobotModel()
            ->getJointModelGroup(HAND_GROUP)
            ->getLinkModelNamesWithCollisionGeometry(),
        false);
      place_container->insert(std::move(stage));
    }

    // 5e. Detach object
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
      stage->detachObject(OBJECT_NAME, HAND_FRAME);
      place_container->insert(std::move(stage));
    }

    // 5f. Retreat (+Z)
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat",
                                                                cartesian_planner);
      stage->properties().set("marker_ns", "retreat");
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      stage->setMinMaxDistance(LIFT_MIN, LIFT_MAX);

      geometry_msgs::msg::Vector3Stamped dir;
      dir.header.frame_id = WORLD_FRAME;
      dir.vector.z = 1.0;
      stage->setDirection(dir);
      place_container->insert(std::move(stage));
    }

    task.add(std::move(place_container));

    return task;
  }

  // ── Plan and execute ─────────────────────────────────────────────────────
  void runTask(double px, double py, double pz,
               double lx, double ly, double lz,
               std::shared_ptr<robot_interfaces::srv::Pick::Response> res)
  {
    // Put the object in the MoveIt planning scene at the pick position
    spawnObject(px, py, pz);

    try {
      auto task = buildTask(lx, ly, lz);

      try {
        task.init();
      } catch (const mtc::InitStageException & ex) {
        res->success = false;
        res->message = std::string("MTC stage init exception: ") + ex.what();
        RCLCPP_ERROR(get_logger(), "%s", res->message.c_str());
        return;
      }

      RCLCPP_INFO(get_logger(), "Planning MTC task …");
      if (task.plan(5 /*solutions*/) != moveit::core::MoveItErrorCode::SUCCESS) {
        res->success = false;
        res->message = "MTC planning failed — no solution found";
        RCLCPP_ERROR(get_logger(), "%s", res->message.c_str());
        return;
      }

      RCLCPP_INFO(get_logger(), "Executing best solution …");
      auto result = task.execute(*task.solutions().front());
      if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
        res->success = false;
        res->message = "MTC execution failed (error " + std::to_string(result.val) + ")";
        RCLCPP_ERROR(get_logger(), "%s", res->message.c_str());
        return;
      }

      removeObject();
      res->success = true;
      res->message = "MTC pick-and-place completed successfully";
      RCLCPP_INFO(get_logger(), "Pick-and-place complete.");
    } catch (const std::exception & e) {
      res->success = false;
      res->message = std::string("MTC exception: ") + e.what();
      RCLCPP_ERROR(get_logger(), "%s", res->message.c_str());
    }
  }

  rclcpp::Service<robot_interfaces::srv::Pick>::SharedPtr service_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions opts;
  opts.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<MtcPickPlace>(opts);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
