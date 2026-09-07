#include "mios/task/taskfactory.hpp"

#include "mios/task/task.hpp"
#include "mios/tasks/null_task.hpp"
#include "mios/tasks/idle_task.hpp"
#include "mios/tasks/test_task_1.hpp"
#include "mios/tasks/test_task_2.hpp"
#include "mios/tasks/test_task_3.hpp"
#include "mios/tasks/learner_test.hpp"
#include "mios/tasks/move_to_cart_pose.hpp"
#include "mios/tasks/move_to_joint_pose.hpp"
#include "mios/tasks/generic_task.hpp"
#include "mios/tasks/insert_object.hpp"
#include "mios/tasks/extract_object.hpp"
#include "mios/tasks/ros2_no_motion_self_test.hpp"

#include "mirmi_cpp_utils/files/files.hpp"
#include "spdlog/spdlog.h"

namespace mios{

TaskName TaskFactory::get_task_name(const std::string& task){
    spdlog::debug("TaskFactory::get_task_name("+task+")");
    switch(mirmi_utils::str_to_int(task.c_str())){
    case mirmi_utils::str_to_int("NullTask"):
        return TaskNameNullTask;
    case mirmi_utils::str_to_int("TestTask2"):
        return TaskNameTestTask2;
    case mirmi_utils::str_to_int("LearnerTest"):
        return TaskNameLearnerTest;
    case mirmi_utils::str_to_int("MoveToCartPose"):
        return TaskNameMoveToCartPose;
    case mirmi_utils::str_to_int("MoveToJointPose"):
        return TaskNameMoveToJointPose;
    case mirmi_utils::str_to_int("TestTask3"):
        return TaskNameTestTask3;
    case mirmi_utils::str_to_int("GenericTask"):
        return TaskNameGenericTask;
    case mirmi_utils::str_to_int("TestTask1"):
        return TaskNameTestTask1;
    case mirmi_utils::str_to_int("IdleTask"):
        return TaskNameIdleTask;
    case mirmi_utils::str_to_int("InsertObject"):
        return TaskNameInsertObject;
    case mirmi_utils::str_to_int("ExtractObject"):
        return TaskNameExtractObject;
    case mirmi_utils::str_to_int("Ros2NoMotionSelfTest"):
        return TaskNameRos2NoMotionSelfTest;
    default:
        spdlog::error("Task with id " + task + " does not exist.");
        return TaskNameNullTask;
    }
}

std::shared_ptr<Task> TaskFactory::create_task(TaskName task, Core* core){
    switch(task){
    case TaskNameNullTask:
        return std::make_shared<NullTask>(core);
    case TaskNameIdleTask:
        return std::make_shared<IdleTask>(core);
    case TaskNameTestTask1:
        return std::make_shared<TestTask1>(core);
    case TaskNameTestTask2:
        return std::make_shared<TestTask2>(core);
    case TaskNameTestTask3:
        return std::make_shared<TestTask3>(core);
    case TaskNameLearnerTest:
        return std::make_shared<LearnerTest>(core);
    case TaskNameMoveToCartPose:
        return std::make_shared<MoveToCartPose>(core);
    case TaskNameMoveToJointPose:
        return std::make_shared<MoveToJointPose>(core);
    case TaskNameGenericTask:
        return std::make_shared<GenericTask>(core);
    case TaskNameInsertObject:
        return std::make_shared<InsertObject>(core);
    case TaskNameExtractObject:
        return std::make_shared<ExtractObject>(core);
    case TaskNameRos2NoMotionSelfTest:
        return std::make_shared<Ros2NoMotionSelfTest>(core);
    default:
        spdlog::critical("Invalid task name.");
        return std::make_shared<NullTask>(core);
    }
}
}
