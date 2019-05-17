#!/usr/bin/env python
# -*- coding: UTF-8 -*-
"""
Construct instructions based on human request and robot perception.
"""

import rospy
import rospkg
from jsk_gui_msgs.msg import VoiceMessage
from thesis.msg import *
from node_viz import create_map_graph
from std_msgs.msg import Int8
import time
import random
import os
import argparse
from vaderSentiment.vaderSentiment import SentimentIntensityAnalyzer
import human_id


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

        self.symptoms = {'fever', 'chill', 'headache', 'vomit', 'asthenia', 'dizziness', 'arthralgia', 'stomachache',
                         'diarrhea', 'thirst', 'spasm', 'restlessness', 'cough', 'dizziness', 'anorexia', 'sneeze',
                         'sore throat'}

        self.analyser = SentimentIntensityAnalyzer()  # analyze human emotion sentiment

        # human_dict = {'name':{'Name': Human()}, 'ip':{'192.168.0.xxx':'Name'}}
        self.human_dict = human_id.load_human_info2dict(rospkg.RosPack().get_path('thesis') + '/human_info/')

        # ignore emergency tasks
        self.task_loc = [0, 1, 2, 5, 6, 7]
        self.task_priority = range(1,  5)  # 1~4
        self.task_duration = range(1, 10)  # 1~9
        self.b_dict = {1: 0.9, 2: 0.92, 3: 0.94, 4: 0.96, 5: 0.98}

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
        def check_status(compound_score):
            """
            Convert emotion into status and reward
            :param compound_score: emotional sentiment score from vader
            :return: status, reward
            """
            if compound_score > 0.05:
                return 0, 1
            elif -0.05 < compound_score < 0.05:
                return 1, 2
            elif compound_score < -0.05:
                return 2, 3
            else:
                rospy.logerr('Invalid sentiment.')
                exit(1)

        ver_instr = Instruction(id=self.last_id,
                                type=0,
                                source=human_id.identify_voice(self.human_dict['ip'], voice_data),
                                destination=-1,
                                status=-1)

        words = voice_data.texts[0].split()

        for w in words:  # Check physical status
            if w in self.symptoms:
                ver_instr.status = 3
                ver_instr.r = 4.0
                ver_instr.function = 9

            elif w in self.human_dict['ip'].values():  # self.human_dict['ip'].values(): list of names
                ver_instr.target = w
                ver_instr.destination = self.human_dict['name'][w].location

        if ver_instr.status == -1:  # Check sentiment of human if not physical
            ver_instr.status, ver_instr.r = check_status(self.analyser.polarity_scores(words)['compound'])

        ver_instr.b = self.b_dict[ver_instr.r]

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
        if is_random:  # for random generate tasks
            max_num = args.max_num
            des_ls = [random.choice(self.task_loc) for _ in range(max_num)]
            r_list = [random.choice(self.task_priority) for _ in range(max_num)]  # reward list
            b_list = [self.b_dict[r] for r in r_list]  # decay factor list
            d_list = [random.choice(self.task_duration) for _ in range(max_num)]  # duration list

        else:
            # des_ls = [5, 7, 0, 1, 1, 1, 0, 0, 7, 5]  # destination
            # r_list = [3, 4, 2, 4, 1, 3, 1, 4, 3, 3]  # reward
            # d_list = [9, 1, 3, 7, 7, 6, 3, 2, 2, 5]  # duration
            # b_list = [self.b_dict[r] for r in r_list]  # decay factor list

            des_ls = [0, 5, 5, 0, 7, 2, 1, 6, 2, 7, 2, 6, 1, 6, 0, 6, 5]  # destination
            r_list = [2, 2, 1, 2, 1, 3, 2, 2, 2, 3, 1, 4, 1, 2, 4, 1, 4]  # reward
            d_list = [9, 2, 8, 3, 6, 9, 3, 7, 3, 8, 5, 7, 9, 1, 6, 5, 9]  # duration
            b_list = [self.b_dict[r] for r in r_list]  # decay factor list

            max_num = len(des_ls)

        for i in range(max_num):
            temp_i = Instruction(id=self.last_id, type=0, duration=d_list[i], source='Charlie', status=0,
                                 r=r_list[i], b=b_list[i], function=0, target='Bob', destination=des_ls[i])
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
    rospy.init_node(os.path.basename(__file__).split('.')[0], anonymous=True, log_level=rospy.INFO)
    parser = argparse.ArgumentParser(description='Check roslaunch arg')
    parser.add_argument('--max_num', type=int, default=10)
    parser.add_argument('--is_rand', type=int, default=1)
    args = parser.parse_args(rospy.myargv()[1:])

    if args.is_rand == 1:
        is_random = True
    elif args.is_rand == 0:
        is_random = False
    else:
        rospy.logerr('is_rand only supports 0 or 1.')
        exit(1)

    instr_constructor = InstructionConstructor()
    instr_constructor.test_scenario()
