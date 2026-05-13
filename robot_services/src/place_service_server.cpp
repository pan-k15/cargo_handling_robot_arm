#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <robot_interfaces/srv/place.hpp>

#include <cmath>
#include <algorithm>
#include <chrono>
#include <future>
#include <thread>

static constexpr double JOINT2_Z = 0.15;
static constexpr double L1       = 0.30;
static constexpr double L2       = 0.53;

static const std::vector<std::string> ARM_JOINTS =
    {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};

static bool computeIK(double tx, double ty, double tz,
                      std::vector<double> & joints)
{
    const double j1 = std::atan2(ty, tx);
    const double r  = std::hypot(tx, ty);
    const double h  = tz - JOINT2_Z;
    const double d  = std::hypot(r, h);

    if (d > L1 + L2 || d < std::fabs(L1 - L2)) {
        return false;
    }

    const double c3 = (d * d - L1 * L1 - L2 * L2) / (2.0 * L1 * L2);
    const double s3 = -std::sqrt(std::max(0.0, 1.0 - c3 * c3));
    const double j3 = std::atan2(s3, c3);
    const double j2 = std::atan2(h, r) - std::atan2(L2 * s3, L1 + L2 * c3);

    if (j2 < -M_PI / 2 || j2 > M_PI / 2) return false;
    if (j3 < -3 * M_PI / 4 || j3 > 0.0)  return false;

    joints = {j1, j2, j3, 0.0, 0.0, 0.0};
    return true;
}

// ── PlaceServiceServer ────────────────────────────────────────────────────────
class PlaceServiceServer : public rclcpp::Node
{
public:
    using Place         = robot_interfaces::srv::Place;
    using FJT           = control_msgs::action::FollowJointTrajectory;
    using GoalHandleFJT = rclcpp_action::ClientGoalHandle<FJT>;

    explicit PlaceServiceServer()
    : Node("place_service_server")
    {
        srv_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        service_ = create_service<Place>(
            "place",
            std::bind(&PlaceServiceServer::handlePlace, this,
                      std::placeholders::_1, std::placeholders::_2),
            rclcpp::ServicesQoS(),
            srv_cbg_);

        arm_client_ = rclcpp_action::create_client<FJT>(
            this, "/arm_controller/follow_joint_trajectory");

        gripper_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
            "/gripper_controller/commands",
            rclcpp::QoS(10).best_effort());

        RCLCPP_INFO(get_logger(), "Place service server ready  →  /place");
    }

private:
    rclcpp::CallbackGroup::SharedPtr srv_cbg_;
    rclcpp::Service<Place>::SharedPtr service_;
    rclcpp_action::Client<FJT>::SharedPtr arm_client_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gripper_pub_;

    bool sendArm(const std::vector<double> & positions, double duration_s,
                 std::chrono::seconds timeout = std::chrono::seconds(30))
    {
        if (!arm_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(get_logger(), "Arm action server not available");
            return false;
        }

        FJT::Goal goal;
        goal.trajectory.joint_names = ARM_JOINTS;

        trajectory_msgs::msg::JointTrajectoryPoint pt;
        pt.positions       = positions;
        pt.time_from_start = rclcpp::Duration::from_seconds(duration_s);
        goal.trajectory.points.push_back(pt);

        auto promise = std::make_shared<std::promise<bool>>();
        auto future  = promise->get_future();

        rclcpp_action::Client<FJT>::SendGoalOptions opts;
        opts.result_callback = [promise](const GoalHandleFJT::WrappedResult & r) {
            promise->set_value(r.code == rclcpp_action::ResultCode::SUCCEEDED);
        };

        arm_client_->async_send_goal(goal, opts);

        if (future.wait_for(timeout) != std::future_status::ready) {
            RCLCPP_ERROR(get_logger(), "Arm trajectory timed out");
            return false;
        }
        return future.get();
    }

    void commandGripper(double opening)
    {
        std_msgs::msg::Float64MultiArray msg;
        msg.data = {opening};
        for (int i = 0; i < 20; ++i) {
            gripper_pub_->publish(msg);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }

    void handlePlace(const std::shared_ptr<Place::Request>  req,
                     std::shared_ptr<Place::Response>        res)
    {
        const auto & p  = req->target_pose.position;
        const double az = p.z + req->approach_height;

        RCLCPP_INFO(get_logger(),
                    "Place request → target=(%.3f, %.3f, %.3f)  approach_h=%.3f",
                    p.x, p.y, p.z, req->approach_height);

        std::vector<double> pre_joints, place_joints;

        if (!computeIK(p.x, p.y, az, pre_joints)) {
            res->success = false;
            res->message = "IK failed: pre-place pose unreachable";
            RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
            return;
        }
        if (!computeIK(p.x, p.y, p.z, place_joints)) {
            res->success = false;
            res->message = "IK failed: place pose unreachable";
            RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
            return;
        }

        // 1. Move to pre-place (above drop-off)
        RCLCPP_INFO(get_logger(), "[place] Moving to pre-place");
        if (!sendArm(pre_joints, 3.0)) {
            res->success = false;
            res->message = "Failed to reach pre-place pose";
            return;
        }

        // 2. Descend to place height
        RCLCPP_INFO(get_logger(), "[place] Descending to place height");
        if (!sendArm(place_joints, 2.0)) {
            res->success = false;
            res->message = "Failed to reach place pose";
            return;
        }

        // 3. Release object
        RCLCPP_INFO(get_logger(), "[place] Opening gripper");
        commandGripper(0.03);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // 4. Retract to pre-place height
        RCLCPP_INFO(get_logger(), "[place] Retracting");
        if (!sendArm(pre_joints, 2.0)) {
            res->success = false;
            res->message = "Failed to retract after place";
            return;
        }

        res->success = true;
        res->message = "Place completed successfully";
        RCLCPP_INFO(get_logger(), "Place complete");
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PlaceServiceServer>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
