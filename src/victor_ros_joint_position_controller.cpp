#include "openrave_RRTplugin/victor_ros_joint_position_controller.h"

#include <boost/bind.hpp>
#include <victor_hardware_interface/MotionCommand.h>
#include <arc_utilities/ros_helpers.hpp>
#include <arc_utilities/pretty_print.hpp>
#include <arc_utilities/eigen_helpers.hpp>

using namespace OpenRAVE;
using namespace armlab_or_plugins;


const std::vector<std::string> VictorROSJointPositionController::left_arm_joint_names_ =
{
    "victor_left_arm_joint_1",
    "victor_left_arm_joint_2",
    "victor_left_arm_joint_3",
    "victor_left_arm_joint_4",
    "victor_left_arm_joint_5",
    "victor_left_arm_joint_6",
    "victor_left_arm_joint_7"
};
const std::vector<std::string> VictorROSJointPositionController::right_arm_joint_names_ =
{
    "victor_right_arm_joint_1",
    "victor_right_arm_joint_2",
    "victor_right_arm_joint_3",
    "victor_right_arm_joint_4",
    "victor_right_arm_joint_5",
    "victor_right_arm_joint_6",
    "victor_right_arm_joint_7"
};

const std::vector<std::string> VictorROSJointPositionController::left_gripper_joint_names_ =
{
    "victor_left_gripper_fingerA_joint_2",
    "victor_left_gripper_fingerA_joint_3",
    "victor_left_gripper_fingerA_joint_4",
    "victor_left_gripper_fingerB_knuckle",
    "victor_left_gripper_fingerB_joint_2",
    "victor_left_gripper_fingerB_joint_3",
    "victor_left_gripper_fingerB_joint_4",
    "victor_left_gripper_fingerC_knuckle",
    "victor_left_gripper_fingerC_joint_2",
    "victor_left_gripper_fingerC_joint_3",
    "victor_left_gripper_fingerC_joint_4"
};
const std::vector<std::string> VictorROSJointPositionController::right_gripper_joint_names_ =
{
    "victor_right_gripper_fingerA_joint_2",
    "victor_right_gripper_fingerA_joint_3",
    "victor_right_gripper_fingerA_joint_4",
    "victor_right_gripper_fingerB_knuckle",
    "victor_right_gripper_fingerB_joint_2",
    "victor_right_gripper_fingerB_joint_3",
    "victor_right_gripper_fingerB_joint_4",
    "victor_right_gripper_fingerC_knuckle",
    "victor_right_gripper_fingerC_joint_2",
    "victor_right_gripper_fingerC_joint_3",
    "victor_right_gripper_fingerC_joint_4"
};



VictorROSJointPositionController::VictorROSJointPositionController(EnvironmentBasePtr penv, std::istream& ss)
    : ControllerBase(penv)
//    , robot_(nullptr)
    , new_desired_position_(false)
    , new_desired_trajectory_(false)
//    , desired_trajectory_(nullptr)
    , target_reached_(false)
    , state_machine_state_(INIT)
    , traj_waypoint_target_ind_(-1)
 {
    (void)ss;
    __description = ":Interface Authors: Dale McConachie\n\nROS controller translation layer for Victor.";
}

VictorROSJointPositionController::~VictorROSJointPositionController()
{}

bool VictorROSJointPositionController::Init(RobotBasePtr robot, const std::vector<int> &dofindices, int nControlTransformation)
{
    // Initialize internal copies of pointers and values
    robot_ = robot;
    if(dofindices.size() != 36)
    {
        RAVELOG_FATAL("VictorROSJointPositionController is only intended to work with the full robot");
        return false;
    }

    dof_indices_ = dofindices;
    if (nControlTransformation)
    {
        RAVELOG_WARN("VictorROSJointPositionController cannot control transformation\n");
    }

    // Get the joint indices for the left and right arm so that we can publish to ROS correctly
    left_arm_dof_indices_ = GetDOFIndices(left_arm_joint_names_);
    right_arm_dof_indices_ = GetDOFIndices(right_arm_joint_names_);

    InitializeROSPubSub();
    spin_thread_ = std::thread(&ROSHelpers::Spin, 0.01);
    state_machine_thread_ = std::thread(&VictorROSJointPositionController::StateMachineLoop, this);

    return true;
}

const std::vector<int>& VictorROSJointPositionController::GetControlDOFIndices() const
{
    return dof_indices_;
}

int VictorROSJointPositionController::IsControlTransformation() const
{
    return false;
}

RobotBasePtr VictorROSJointPositionController::GetRobot() const
{
    return robot_.lock();
}

void VictorROSJointPositionController::Reset(int options)
{
    (void)options;
    RAVELOG_WARN("VictorROSJointPositionController: Reset called, but ignored\n");
}

bool VictorROSJointPositionController::SetDesired(const std::vector<dReal>& values, TransformConstPtr trans)
{
    std::cout << "New desired configuration recieved\n";

    (void)trans;
    if( values.size() != dof_indices_.size() )
    {
        throw openrave_exception(str(boost::format("wrong desired dimensions %d!=%d") % values.size() % dof_indices_.size()), ORE_InvalidArguments);
    }
    std::lock_guard<std::mutex> lock(state_machine_input_mutex_);
    new_desired_position_ = true;
    desired_position_values_ = values;
    target_reached_.store(false);
    return true;
}

bool VictorROSJointPositionController::SetPath(TrajectoryBaseConstPtr ptraj)
{
    std::cout << "New path recieved\n";

    std::lock_guard<std::mutex> lock(state_machine_input_mutex_);
    new_desired_trajectory_ = true;
    desired_trajectory_ = ptraj;
    target_reached_.store(false);
    return true;
}

void VictorROSJointPositionController::SimulationStep(dReal fTimeEllapsed)
{
    // This function is not implemented, as we are not using this controller in simulation
    (void)fTimeEllapsed;
}

bool VictorROSJointPositionController::IsDone()
{
    return target_reached_.load();
}



void VictorROSJointPositionController::InitializeROSPubSub()
{
    const std::string joint_states_topic = ROSHelpers::GetParam<std::string>(nh_, "joint_status_topic", "joint_states");
    const std::string right_arm_motion_command_topic = ROSHelpers::GetParam<std::string>(nh_, "right_arm_motion_command_topic", "right_arm/motion_command");
    const std::string left_arm_motion_command_topic = ROSHelpers::GetParam<std::string>(nh_, "left_arm_motion_command_topic", "left_arm/motion_command");

    joint_states_subscriber_ = nh_.subscribe(joint_states_topic, 1, &VictorROSJointPositionController::JointStateCallback, this);
    right_arm_motion_command_publisher_ = nh_.advertise<victor_hardware_interface::MotionCommand>(right_arm_motion_command_topic, 1, false);
    left_arm_motion_command_publisher_ = nh_.advertise<victor_hardware_interface::MotionCommand>(left_arm_motion_command_topic, 1, false);
}

std::vector<int> VictorROSJointPositionController::GetDOFIndices(const std::vector<std::string>& joint_names)
{
    RobotBasePtr robot = robot_.lock();
    EnvironmentMutex::scoped_lock lockenv(robot->GetEnv()->GetMutex());

    std::vector<int> indices(joint_names.size());
    for (size_t joint_name_ind = 0; joint_name_ind < joint_names.size(); joint_name_ind++)
    {
        indices[joint_name_ind] = robot->GetJoint(joint_names[joint_name_ind])->GetJointIndex();
    }
    return indices;
}

void VictorROSJointPositionController::JointStateCallback(const sensor_msgs::JointState::ConstPtr& joint_state)
{
    // Read in the current joint values so that we only replace those that we got data for
    RobotBasePtr robot = robot_.lock();
    EnvironmentMutex::scoped_lock lockenv(robot->GetEnv()->GetMutex());
    std::vector<dReal> curr_joint_values;
    robot->GetDOFValues(curr_joint_values);

    // Read in each joint's name and position, setting the values in OpenRAVE to match
    for (size_t joint_name_ind = 0; joint_name_ind < joint_state->name.size(); joint_name_ind++)
    {
        const size_t joint_ind = robot->GetJoint(joint_state->name[joint_name_ind])->GetJointIndex();
        curr_joint_values[joint_ind] = joint_state->position[joint_name_ind];
    }
    robot->SetDOFValues(curr_joint_values, false);
}



void VictorROSJointPositionController::StateMachineLoop()
{
    ros::Rate loop_rate(100);

    while (ros::ok())
    {
        // Read input data
        const InputData input_data = ReadInputData();
        state_machine_state_ = DetermineNextState(state_machine_state_, input_data);
//        state_machine_state_ = input_data.new_desired_trajectory_ ? STARTING_NEW_TRAJ : NO_ACTIVE_TARGET;

        // Set output based on current state
        switch (state_machine_state_)
        {
            case INIT:
            {
                // This shouldn't be possible
                assert(false);
                break;
            }

            case NO_ACTIVE_TARGET:
            {
                traj_waypoint_target_ind_ = -1;
                std::lock_guard<std::mutex> lock(state_machine_input_mutex_);
                target_reached_.store(!new_desired_position_ && !new_desired_trajectory_);
                break;
            }

            case GOING_TO_SINGLE_POSITION:
            {
                traj_waypoint_target_ind_ = -1;
                SendJointCommand(input_data.desired_position_values_);
                target_reached_.store(false);
                break;
            }

            case STARTING_NEW_TRAJ:
            {
                assert(input_data.desired_trajectory_.get() != nullptr);
                assert(input_data.desired_trajectory_->GetNumWaypoints() > 0);

                traj_waypoint_target_ind_ = 0;
                const std::vector<dReal> target_joint_values = GetJointValuesFromTrajectory(input_data.desired_trajectory_, traj_waypoint_target_ind_);
                SendJointCommand(target_joint_values);
                target_reached_.store(false);
                break;
            }

            case FOLLOWING_TRAJ:
            {
                std::vector<dReal> target_joint_values = GetJointValuesFromTrajectory(input_data.desired_trajectory_, traj_waypoint_target_ind_);
                if (IsGoalReached(input_data.curr_joint_values_, target_joint_values, right_arm_dof_indices_))
                {
                    traj_waypoint_target_ind_++;
                    target_joint_values = GetJointValuesFromTrajectory(input_data.desired_trajectory_, traj_waypoint_target_ind_);
                }
                SendJointCommand(target_joint_values);
                target_reached_.store(false);
                break;
            }

            default:
                // This shouldn't be possible
                assert(false);
        }
        loop_rate.sleep();
    }
}



VictorROSJointPositionController::InputData VictorROSJointPositionController::ReadInputData()
{
    VictorROSJointPositionController::InputData input_data;

    RobotBasePtr robot = robot_.lock();
    EnvironmentMutex::scoped_lock lockenv(robot->GetEnv()->GetMutex());
    std::lock_guard<std::mutex> lock(state_machine_input_mutex_);
    input_data.new_desired_position_            = new_desired_position_;
    input_data.desired_position_values_         = desired_position_values_;
    input_data.new_desired_trajectory_          = new_desired_trajectory_;
    input_data.desired_trajectory_              = desired_trajectory_;

    robot->GetDOFValues(input_data.curr_joint_values_);
    input_data.traj_waypoint_target_ind_        = traj_waypoint_target_ind_;

    new_desired_position_ = false;
    new_desired_trajectory_ = false;

    return input_data;
}

VictorROSJointPositionController::STATE VictorROSJointPositionController::DetermineNextState(const VictorROSJointPositionController::STATE current_state, const InputData& input_data) const
{
    VictorROSJointPositionController::STATE next_state = current_state;
    switch (current_state)
    {
        case INIT:
        {
            // Joint position targets take precedence over Trajectory targets
            if (input_data.new_desired_position_)
            {
                next_state = GOING_TO_SINGLE_POSITION;
            }
            else if (input_data.new_desired_trajectory_)
            {
                next_state = STARTING_NEW_TRAJ;
            }
            else
            {
                next_state = NO_ACTIVE_TARGET;
            }

            break;
        }

        case NO_ACTIVE_TARGET:
        {
            // Joint position targets take precedence over Trajectory targets
            if (input_data.new_desired_position_)
            {
                next_state = GOING_TO_SINGLE_POSITION;
            }
            else if (input_data.new_desired_trajectory_)
            {
                next_state = STARTING_NEW_TRAJ;
            }
            else
            {
                next_state = NO_ACTIVE_TARGET;
            }

            break;
        }

        case GOING_TO_SINGLE_POSITION:
        {
            // Joint position targets take precedence over Trajectory targets
            if (input_data.new_desired_position_)
            {
                next_state = GOING_TO_SINGLE_POSITION;
            }
            else if (input_data.new_desired_trajectory_)
            {
                next_state = STARTING_NEW_TRAJ;
            }
            else if (IsGoalReached(input_data.curr_joint_values_, input_data.desired_position_values_, right_arm_dof_indices_))
            {
                next_state = NO_ACTIVE_TARGET;
            }
            else
            {
                next_state = GOING_TO_SINGLE_POSITION;
            }

            break;
        }

        case STARTING_NEW_TRAJ:
        {
            if (input_data.new_desired_position_)
            {
                next_state = GOING_TO_SINGLE_POSITION;
            }
            else if (input_data.new_desired_trajectory_)
            {
                next_state = STARTING_NEW_TRAJ;
            }
            else
            {
                next_state = FOLLOWING_TRAJ;
            }
            break;
        }

        case FOLLOWING_TRAJ:
        {
            // Joint position targets take precedence over Trajectory targets
            if (input_data.new_desired_position_)
            {
                next_state = GOING_TO_SINGLE_POSITION;
            }
            else if (input_data.new_desired_trajectory_)
            {
                next_state = STARTING_NEW_TRAJ;
            }
            else
            {
                assert(input_data.desired_trajectory_.get() != nullptr);
                const int num_waypoints = (int)input_data.desired_trajectory_->GetNumWaypoints();
                assert(input_data.traj_waypoint_target_ind_ >= 0);
                assert(input_data.traj_waypoint_target_ind_ < num_waypoints);

                const std::vector<dReal> target_joint_values = GetJointValuesFromTrajectory(input_data.desired_trajectory_, input_data.traj_waypoint_target_ind_);
                if (input_data.traj_waypoint_target_ind_ + 1 < num_waypoints || !IsGoalReached(input_data.curr_joint_values_, target_joint_values, right_arm_dof_indices_))
                {
                    next_state = FOLLOWING_TRAJ;
                }
                else
                {
                    next_state = NO_ACTIVE_TARGET;
                }
            }

            break;
        }

        default:
            // This shouldn't be possible
            assert(false);
    }

    return next_state;
}

std::vector<dReal> VictorROSJointPositionController::GetJointValuesFromTrajectory(const OpenRAVE::TrajectoryBaseConstPtr trajectory, const int waypoint_ind) const
{
    std::vector<dReal> target_joint_values(dof_indices_.size());
    std::vector<dReal> waypoint_data;
    trajectory->GetWaypoint(waypoint_ind, waypoint_data);
    trajectory->GetConfigurationSpecification().ExtractJointValues(target_joint_values.begin(), waypoint_data.begin(), robot_.lock(), dof_indices_);
    return target_joint_values;
}

bool VictorROSJointPositionController::IsGoalReached(const std::vector<dReal>& current_vals, const std::vector<dReal>& target_vals, const std::vector<int> dof_indices_to_check)
{
    for (int dof_ind : dof_indices_to_check)
    {
        if (!EigenHelpers::CloseEnough(current_vals[dof_ind], target_vals[dof_ind], 5e-2))
        {
            return false;
        }
    }

    return true;
}

void VictorROSJointPositionController::SendJointCommand(const std::vector<dReal>& joint_vals)
{
    assert(joint_vals.size() == 36);

    victor_hardware_interface::MotionCommand left_arm_cmd;
    left_arm_cmd.header.frame_id = "mocap_world";

    left_arm_cmd.control_mode = victor_hardware_interface::MotionCommand::JOINT_POSITION;
    left_arm_cmd.joint_position.joint_1 = joint_vals[left_arm_dof_indices_[0]];
    left_arm_cmd.joint_position.joint_2 = joint_vals[left_arm_dof_indices_[1]];
    left_arm_cmd.joint_position.joint_3 = joint_vals[left_arm_dof_indices_[2]];
    left_arm_cmd.joint_position.joint_4 = joint_vals[left_arm_dof_indices_[3]];
    left_arm_cmd.joint_position.joint_5 = joint_vals[left_arm_dof_indices_[4]];
    left_arm_cmd.joint_position.joint_6 = joint_vals[left_arm_dof_indices_[5]];
    left_arm_cmd.joint_position.joint_7 = joint_vals[left_arm_dof_indices_[6]];

    victor_hardware_interface::MotionCommand right_arm_cmd;
    right_arm_cmd.header.frame_id = "mocap_world";

    right_arm_cmd.control_mode = victor_hardware_interface::MotionCommand::JOINT_POSITION;
    right_arm_cmd.joint_position.joint_1 = joint_vals[right_arm_dof_indices_[0]];
    right_arm_cmd.joint_position.joint_2 = joint_vals[right_arm_dof_indices_[1]];
    right_arm_cmd.joint_position.joint_3 = joint_vals[right_arm_dof_indices_[2]];
    right_arm_cmd.joint_position.joint_4 = joint_vals[right_arm_dof_indices_[3]];
    right_arm_cmd.joint_position.joint_5 = joint_vals[right_arm_dof_indices_[4]];
    right_arm_cmd.joint_position.joint_6 = joint_vals[right_arm_dof_indices_[5]];
    right_arm_cmd.joint_position.joint_7 = joint_vals[right_arm_dof_indices_[6]];

    left_arm_cmd.header.stamp = ros::Time::now();
    right_arm_cmd.header.stamp = left_arm_cmd.header.stamp;
    left_arm_motion_command_publisher_.publish(left_arm_cmd);
    right_arm_motion_command_publisher_.publish(right_arm_cmd);
}


// called to create a new plugin
InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    if (type == PT_Controller && interfacename == "victorrosjointpositioncontroller")
    {
        return InterfaceBasePtr(new armlab_or_plugins::VictorROSJointPositionController(penv, sinput));
    }

    return InterfaceBasePtr();
}

// called to query available plugins
void GetPluginAttributesValidated(PLUGININFO& info)
{
    info.interfacenames[PT_Controller].push_back("VictorROSJointPositionController");
}

// called before plugin is terminated
OPENRAVE_PLUGIN_API void DestroyPlugin()
{
}
