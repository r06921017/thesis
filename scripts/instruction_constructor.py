#!/usr/bin/env python
# -*- coding: UTF-8 -*-
"""
Construct instructions based on human request and robot perception.
"""

import rospy
from jsk_gui_msgs.msg import VoiceMessage
from thesis.msg import *
from node_viz import create_map_graph
from std_msgs.msg import Int8
import time
import random


class InstructionConstructor:
    def __init__(self):
        self.instr_sub = rospy.Subscriber('/thesis/instruction_buffer', InstructionArray, self.instr_cb, queue_size=10)
        self.verbal_sub = rospy.Subscriber('/thesis/verbal_buffer', VoiceMessage, self.verbal_cb, queue_size=10)
        self.temp_sub = rospy.Subscriber('/thesis/int_buffer', Int8, self.int_cb, queue_size=10)

        self.instr_pub = rospy.Publisher('/thesis/instruction_buffer', InstructionArray, queue_size=1)
        self.map_graph = create_map_graph()
        self.instr_dict = dict()
        self.instr_dest_dict = {n: set() for n in self.map_graph.nodes}
        self.last_id = 0  # record the last id in current instruction buffer
        self.loc_symbol = {0: 'office', 1: 'bedroom', 2: 'charge', 3: 'alley1', 4: 'alley2',
                           5: 'livingroom', 6: 'diningroom', 7: 'greet', 8: 'emergency'}

        # ignore emergency tasks
        self.task_loc = [0, 1, 2, 5, 6, 7]
        self.task_priority = range(1,  5)  # 1~4
        self.task_duration = range(1, 10)  # 1~9
        self.b_dict = {1: 0.9, 2: 0.92, 3: 0.94, 4: 0.96}  # 5: 0.98

    def instr_cb(self, in_instructions):
        rospy.loginfo('instruction callback')
        self.instr_dict = dict()
        # Convert InstructionArray into dictionary
        if type(in_instructions) == thesis.msg._InstructionArray.InstructionArray:
            for instr in in_instructions.data:
                self.instr_dest_dict[instr.destination].add(instr.id)
                self.instr_dict[instr.id] = instr
        rospy.loginfo('After instruction callback ...')
        self.show_instr()
        return

    def int_cb(self, in_int):
        if in_int.data == 0:
            self.instr_dict[self.last_id] = Instruction(id=self.last_id, type=0, duration=1, source='Charlie', status=0,
                                                        r=1, b=0.9, function=0, target='Bob', destination=2)
            self.last_id += 1

        self.launch_instr()
        return

    def show_instr(self):
        print '--------------------------------------------------------------------------'
        for key, instr in self.instr_dict.iteritems():
            print 'instr {0}: dest={1}, function={2}, duration={3}, r={4}'.format(instr.id,
                                                                                  instr.destination,
                                                                                  instr.function,
                                                                                  instr.duration,
                                                                                  instr.r)
        print '--------------------------------------------------------------------------'
        return

    def verbal_cb(self, voice_data):
        return

    def launch_instr(self):
        rospy.loginfo('Launching instructions!')
        _instr_list = InstructionArray()
        for key, instr in self.instr_dict.iteritems():
            _instr_list.data.append(instr)

        # Update maximum instruction id
        # if len(self.instr_dict.keys()) > 0:
        #     self.last_id = max(self.instr_dict.keys())

        rospy.sleep(1)
        self.instr_pub.publish(_instr_list)
        return

    def test_scenario(self):
        max_num = 10
        des = [random.choice(self.task_loc) for _ in range(max_num)]
        r_list = [random.choice(self.task_priority) for _ in range(max_num)]  # reward list
        b_list = [self.b_dict[r] for r in r_list]  # decay factor list
        d_list = [random.choice(self.task_duration) for _ in range(max_num)]  # duration list

        # print '++++++++++++++++++++++++++++++++++++++++++++++++++++++++'
        # print 'des: ', des
        # print 'r_list: ', r_list
        # print 'b_list: ', b_list
        # print '++++++++++++++++++++++++++++++++++++++++++++++++++++++++'

        # max_num = 3
        # des = [0, 5, 0, 6, 8]  # destination
        # r_list = [4, 5, 1, 2, 3]  # reward
        # b_list = [0.95, 0.99, 0.9, 0.92, 0.93]  # decay factor
        # d_list = [1, 1, 1, 1, 1]  # duration

        for i in range(max_num):
            temp_i = Instruction(id=self.last_id, type=0, duration=d_list[i], source='Charlie', status=0,
                                 r=r_list[i], b=b_list[i], function=0, target='Bob', destination=des[i])
            self.instr_dict[self.last_id] = temp_i
            self.last_id += 1

        rospy.loginfo('Total task duration: {0} (s)'.format(sum(d_list)))

        t = 1
        rospy.loginfo('Sleep for {0} seconds'.format(str(t)))
        rospy.sleep(t)

        start_time = time.time()
        self.launch_instr()
        rospy.sleep(t)

        while not rospy.is_shutdown():
            try:
                temp_instr = rospy.wait_for_message('/thesis/instruction_buffer', InstructionArray, timeout=0.5)
                if len(temp_instr.data) == 0:
                    end_time = time.time()
                    rospy.loginfo('Total task duration: {0} (s)'.format(sum(d_list)))
                    rospy.loginfo('Task process time: {0} (s)'.format(end_time - start_time - t))
                    rospy.loginfo('Navigation process time: {0} (s)'.format(end_time - start_time - t - sum(d_list)))
                    break

            except rospy.ROSException:  # timeout
                pass

        return


if __name__ == '__main__':
    rospy.init_node('instruction_constructor', anonymous=True, log_level=rospy.INFO)
    instr_constructor = InstructionConstructor()
    instr_constructor.test_scenario()
