import omnigibson.utils.transform_utils as T

import mimicgen.utils.pose_utils as PoseUtils
from mimicgen.env_interfaces.base import MG_EnvInterface


class MG_OmniEnvInterface(MG_EnvInterface):
    INTERFACE_TYPE = "omnigibson"

    def get_robot_eef_pose(self):
        robot = self.env.robots[0]
        pos = robot.get_eef_position()
        quat = robot.get_eef_orientation()
        rot = T.quat2mat(quat)
        return PoseUtils.make_pose(pos, rot)

    def target_pose_to_action(self, target_pose):
        raise NotImplementedError("Subclass must implement target_pose_to_action")

    def action_to_target_pose(self, action, relative=True):
        raise NotImplementedError("Subclass must implement action_to_target_pose")

    def action_to_gripper_action(self, action):
        return action[-1:]

    def get_object_pose(self, obj_name):
        obj = self.env.scene.object_registry("name", obj_name)
        pos, quat = obj.get_position_orientation()
        rot = T.quat2mat(quat)
        return PoseUtils.make_pose(pos, rot)


class MG_Sim2RealGen(MG_OmniEnvInterface):
    def get_object_poses(self):
        return self.env.task.get_object_poses()

    def get_subtask_term_signals(self):
        return self.env.task.get_subtask_term_signals()
