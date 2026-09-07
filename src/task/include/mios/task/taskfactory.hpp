#pragma once

#include <memory>
#include <string>

namespace mios{
class Task;
class Core;
enum TaskName{TaskNameNullTask,TaskNameTestTask2,TaskNameLearnerTest,TaskNameMoveToCartPose,TaskNameMoveToJointPose,TaskNameTestTask3,TaskNameGenericTask,TaskNameTestTask1,TaskNameIdleTask,
              TaskNameInsertObject,TaskNameExtractObject,TaskNameRos2NoMotionSelfTest};
class TaskFactory{
public:
    static TaskName get_task_name(const std::string& task);
    static std::shared_ptr<Task> create_task(TaskName task, Core* core);
};
}
