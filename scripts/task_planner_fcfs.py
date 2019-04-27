#!/usr/bin/env python
# -*- coding: utf8 -*-
"""
Solve task planning with first come first serve
"""

from human_id import *
from thesis.msg import *
import rospkg


class TaskPlannerFCFS:
    def __init__(self):
        self.sub = rospy.Subscriber('/thesis/instruction_buffer', InstructionArray, self.plan_task, queue_size=10)
        self.task_pub = rospy.Publisher('/thesis/task_buffer', InstructionArray, queue_size=1)
        self._pkg_dir = rospkg.RosPack().get_path('thesis')

    @staticmethod
    def show_task(in_task_list):
        id_list = [in_task_list[i].id for i in range(len(in_task_list))]
        rospy.loginfo('task order: ', id_list)
        return

    def get_pkg_dir(self):
        print 'pkg path', self._pkg_dir

    def plan_task(self, instr_list):
        self.show_task(instr_list)
        self.task_pub.publish(instr_list)
        return


if __name__ == '__main__':
    rospy.init_node('task_planner_fcfs', anonymous=True, log_level=rospy.INFO)
    task_planner = TaskPlannerFCFS()
    rospy.spin()
