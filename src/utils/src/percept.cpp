#include "mios/data_structures/percept.hpp"
#include "mirmi_cpp_utils/math/math.hpp"

namespace mios {

Percept::Percept(){
    internal_model.hand_activity_state=HandActivityState::hsIdle;
    proprioception.K_F_ext_K.setZero(); 

}

void Percept::update(const control::RobotState& robot_state, const control::RobotModel& model,
                     const control::GripperState& gripper_state,
                     std::optional<Eigen::Matrix<double,3,3>> O_R_T) {
    internal_model.B_J_EE = Eigen::Matrix<double,6,7>(model.body_jacobian.data());
    internal_model.B_J_O = Eigen::Matrix<double,6,7>(model.zero_jacobian.data());
    internal_model.M = Eigen::Matrix<double,7,7>(model.mass.data());
    internal_model.C = Eigen::Matrix<double,7,1>(model.coriolis.data());
    internal_model.G = Eigen::Matrix<double,7,1>(model.gravity.data());
    internal_model.max_finger_width = gripper_state.max_width;

    proprioception.q = Eigen::Matrix<double,7,1>(robot_state.position.data());
    proprioception.dq = Eigen::Matrix<double,7,1>(robot_state.velocity.data());
    proprioception.theta = Eigen::Matrix<double,7,1>(robot_state.motor_position.data());
    proprioception.dtheta = Eigen::Matrix<double,7,1>(robot_state.motor_velocity.data());
    proprioception.tau_j = Eigen::Matrix<double,7,1>(robot_state.effort.data());
    proprioception.tau_ext = Eigen::Matrix<double,7,1>(robot_state.external_effort.data());
    proprioception.O_T_EE = Eigen::Matrix<double,4,4>(robot_state.base_to_end_effector.data());
    proprioception.O_F_ext_K =
        Eigen::Matrix<double,6,1>(robot_state.external_wrench_origin.data());
    proprioception.K_F_ext_K =
        Eigen::Matrix<double,6,1>(robot_state.external_wrench_stiffness.data());
    proprioception.O_dX_EE = internal_model.B_J_O * proprioception.dq;
    proprioception.EE_dX_EE = internal_model.B_J_EE * proprioception.dq;

    const Eigen::Matrix<double,3,3> O_R_T_id = Eigen::Matrix<double,3,3>::Identity();
    const auto& task_rotation = O_R_T.value_or(O_R_T_id);
    proprioception.T_T_EE = mirmi_utils::rotate_matrix(proprioception.O_T_EE, task_rotation.transpose());
    proprioception.TF_F_ext_K =
        mirmi_utils::rotate_vector(proprioception.O_F_ext_K, task_rotation.transpose());
    proprioception.TF_dX_EE =
        mirmi_utils::rotate_vector(proprioception.O_dX_EE, task_rotation.transpose());
    proprioception.finger_width = gripper_state.width;
    proprioception.finger_temperature = gripper_state.temperature;
    proprioception.is_grasping = gripper_state.is_grasped;
    robot_mode = robot_state.user_stopped ? control::RobotMode::kUserStopped
                                          : robot_state.robot_mode;
    time = std::chrono::high_resolution_clock::now();
    controller.O_R_T = task_rotation;
}

void Percept::update_controller(){
    controller.q_d=proprioception.q;
    controller.TF_T_EE_d=proprioception.T_T_EE;
}

}
