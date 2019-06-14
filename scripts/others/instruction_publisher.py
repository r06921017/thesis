#!/usr/bin/env python
# -*- coding: UTF-8 -*-
"""
Construct instructions based on human request and robot perception.
"""

import rospy
from thesis.msg import *


def get_instr_dest(in_instruction):
    temp = set()
    for instr in in_instruction.data:
        temp.add(instr.destination)
    return list(temp)


if __name__ == '__main__':
    rospy.init_node('instruction_publisher', anonymous=True, log_level=rospy.INFO)
    instr_pub = rospy.Publisher('/thesis/instruction_buffer', InstructionArray, queue_size=1)
    rospy.loginfo('instruction_constructor start!')

    loc_symbol = {0: 'office',
                  1: 'bedroom',
                  2: 'charge',
                  3: 'alley1',
                  4: 'alley2',
                  5: 'livingroom',
                  6: 'diningroom',
                  7: 'greet',
                  8: 'emergency'}

    # initialize parameters
    loc_num = len(loc_symbol)
    max_num = 3  # maximum number for simultaneous instructions in the period
    freq = 0.1  # 10 seconds
    des = [0, 5, 0, 6, 8]  # destination
    r_list = [4, 5, 1, 2, 3]
    b_list = [0.95, 0.99, 0.9, 0.92, 0.93]

    i_list = []  # i for 'instruction'
    instruction_node_set = set()  # int set for instruction destination
    last_id = 0

    for i in range(max_num):
        temp_i = Instruction(id=last_id, type=0, duration=1, source='Charlie', status=0, r=r_list[i], b=b_list[i],
                             function=0, target='Bob', destination=des[i])
        i_list.append(temp_i)
        instruction_node_set.add(temp_i.destination)
        last_id += 1

    print 'instruction_node_set = ', instruction_node_set
    instruction_node_set = list(instruction_node_set)
    print 'instruction_node_set = ', instruction_node_set

    rospy.sleep(1)
    instr_pub.publish(i_list)
    # instr_dest_pub.publish(instruction_node_set)

    rospy.loginfo('Publish instructions first.')
    # rospy.sleep(20)
    #
    # rospy.loginfo('Wait for undo instructions ...')
    # i_list = rospy.wait_for_message('/thesis/instruction_buffer', InstructionArray, timeout=10)
    # for i in range(max_num, len(des)):
    #     temp_i = Instruction(id=last_id, type=0, duration=1, source='Charlie', status=0, r=r_list[i], b=b_list[i],
    #                          function=0, target='Bob', destination=des[i])
    #     i_list.data.append(temp_i)
    # rospy.sleep(1)
    # instr_pub.publish(i_list)
    # rospy.loginfo('Publish instructions again.')
