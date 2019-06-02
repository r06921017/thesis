#!/usr/bin/env python
# -*- coding: utf8 -*-
"""
Solve task planning with dynamic programming (value iteration)
"""
from task_motion_planner_fcfs import *
import operator
from robot_motions import *
from std_msgs.msg import Int32


class TaskMotionPlannerDP(TaskMotionPlannerFCFS):
    def __init__(self):
        """
        self.cur_node: the node where robot starts to move, initial at charge (2), type: int
        """
        TaskMotionPlannerFCFS.__init__(self)
        self.shortest_path = nx.floyd_warshall_numpy(self.map_graph)

        # for real world motion
        self.motion_sub = rospy.Subscriber('/thesis/next_node', Int32, self.nav_cb, queue_size=5)
        self.sac = actionlib.SimpleActionClient('move_base', MoveBaseAction)  # type: SimpleActionClient
        self.move_lock = False
        self.robot_x, self.robot_y, _ = get_cur_pos(pos_topic='/amcl_pose', t=0.2)
        self.dis_unit = 0.2

        print '----------------------------------------'
        print self.shortest_path
        print self.shortest_path.shape
        print '----------------------------------------'

    def value_iter(self):
        # Compare the accumulated reward of neighbor, move to the maximum one.
        neighbor_node = np.where(np.array(self.cur_neighbor) > 0)[0].tolist()
        candidate_steps = dict()  # {neighbor_node: {'reward': float, 'neighbor': np.array}}

        for n in neighbor_node:
            temp_neighbor = self.move_adjacency_node(n)  # assuming moving toward the neighbor node n.
            temp_neighbor_node = np.where(np.array(temp_neighbor) > 0)[0].tolist()
            rospy.logdebug('temp_neighbor_node = {0}'.format(temp_neighbor_node))

            # Calculate accumulated reward of a candidate step from all instructions
            total_reward = 0.0
            for _, instr in self.instr_dict.iteritems():
                if instr.prev_id not in self.instr_dict.keys():
                    # Possible distances from instr to candidate steps
                    temp_dis = np.zeros(len(temp_neighbor_node)).astype(float)
                    for j, temp_n in enumerate(temp_neighbor_node):
                        temp_dis[j] = self.shortest_path[instr.destination, temp_n] + temp_neighbor[temp_n]

                    total_reward += instr.r * pow(instr.b, np.min(temp_dis))

            rospy.logdebug('total reward in candidate node {0}: {1}'.format(n, total_reward))

            candidate_steps[n] = {'reward': total_reward, 'neighbor': temp_neighbor}

        # Pick the node with maximum accumulated reward
        self.next_node = max(candidate_steps, key=lambda x: candidate_steps[x]['reward'])
        return

    def plan_task(self, in_instructions):
        rospy.loginfo('Start task planning!')

        # Convert InstructionArray into dictionary
        if type(in_instructions) == thesis.msg._InstructionArray.InstructionArray:
            for instr in in_instructions.data:
                self.instr_dest_dict[instr.destination].add(instr.id)
                self.instr_dict[instr.id] = instr

            print 'self.instr_dict: ', self.instr_dict

        rospy.logdebug('len(self.instr_dict.keys()): {0}'.format(len(self.instr_dict.keys())))

        if len(self.instr_dict.keys()) > 0:  # if there exists instructions
            if len(self.instr_dest_dict[self.cur_node]) > 0:
                # for two-stage instruction
                temp_count = 0
                for instr_id in self.instr_dest_dict[self.cur_node]:
                    if self.instr_dict[instr_id].prev_id in self.instr_dict.keys():
                        temp_count += 1
                if temp_count == len(self.instr_dest_dict[self.cur_node]):
                    self.value_iter()
                else:
                    self.next_node = self.cur_node

                # for single stage instruction
                # self.next_node = self.cur_node

            else:
                self.value_iter()

            rospy.loginfo('plan_task result: {0}'.format(self.next_node))

        return

    def plan_motion_viz(self):
        if self.cur_node == self.next_node:
            rospy.loginfo('Motion: Reach node {0}.'.format(self.next_node))

            if len(self.instr_dest_dict[self.cur_node]) > 0:
                # Create reward_dict = {'id (int)': 'reward (float)'}
                reward_dict = dict()
                for idx in self.instr_dest_dict[self.cur_node]:
                    # Check two-stage instruction
                    if self.instr_dict[idx].prev_id not in self.instr_dict.keys():
                        reward_dict[self.instr_dict[idx].id] = self.instr_dict[idx].r

                # Sort the instructions with the max reward
                for r in sorted(reward_dict.items(), key=operator.itemgetter(1), reverse=True):
                    do_instr = self.instr_dict[r[0]]
                    rospy.loginfo('Do instr {0}: {1}'.format(do_instr.id, do_instr.function))
                    rospy.sleep(do_instr.duration)
                    del self.instr_dict[r[0]]
                    self.show_instr()

                # Reset the set when all the tasks in the instructions are done.
                self.instr_dest_dict[self.cur_node].clear()
                rospy.logdebug('self.instr_dest_dict: {0}'.format(self.instr_dest_dict))

                # Convert undo_tasks to a list() and publish to /thesis/instruction_buffer
                undo_instr_list = list()
                for key, value in self.instr_dict.iteritems():
                    undo_instr_list.append(value)

                self.task_pub.publish(undo_instr_list)

            else:
                rospy.loginfo('No instructions on task {0}'.format(self.cur_node))
                if len(self.instr_dict) > 0:
                    self.plan_task(self.instr_dict)

        else:
            rospy.loginfo('Motion: from {0} to {1}'.format(self.cur_node, self.next_node))
            self.move_adjacency_node(self.next_node, sim=False, render=True)

        return

    def nav_cb(self, in_next_node):
        """

        :param in_next_node: next_node from task planner
        :return:
        """
        # Moving toward node
        if self.move_lock:
            shutdown(self.sac)
        else:
            if in_next_node != self.next_node:
                shutdown(self.sac)
                simple_move_base(self.sac, loc[in_next_node][0], loc[in_next_node][1], loc[in_next_node][2])
                self.cur_node = self.next_node
                self.move_lock = True

        # Reach node
        if len(self.instr_dest_dict[self.cur_node]) > 0:
            # Create reward_dict = {'id (int)': 'reward (float)'}
            reward_dict = dict()
            for idx in self.instr_dest_dict[self.cur_node]:
                # Check two-stage instruction
                if self.instr_dict[idx].prev_id not in self.instr_dict.keys():
                    reward_dict[self.instr_dict[idx].id] = self.instr_dict[idx].r

            # Sort the instructions with the max reward
            for r in sorted(reward_dict.items(), key=operator.itemgetter(1), reverse=True):
                do_instr = self.instr_dict[r[0]]
                rospy.loginfo('Do instr {0}: {1}'.format(do_instr.id, do_instr.function))
                rospy.sleep(do_instr.duration)
                del self.instr_dict[r[0]]
                self.show_instr()

            # Reset the set when all the tasks in the instructions are done.
            self.instr_dest_dict[self.cur_node].clear()
            rospy.logdebug('self.instr_dest_dict: {0}'.format(self.instr_dest_dict))

            # Convert undo_tasks to a list() and publish to /thesis/instruction_buffer
            undo_instr_list = list()
            for key, value in self.instr_dict.iteritems():
                undo_instr_list.append(value)

            self.task_pub.publish(undo_instr_list)

        else:
            rospy.loginfo('No instructions on task {0}'.format(self.cur_node))
            if len(self.instr_dict) > 0:
                self.plan_task(self.instr_dict)

        return

    def pos_cb(self, r_amcl):
        rospy.loginfo('pos_cb!')
        move_dis = norm(r_amcl.pose.pose.position.x-self.robot_x, r_amcl.pose.pose.position.y-self.robot_y)

        rospy.loginfo('cur_pos: {0}, {1}'.format(r_amcl.pose.pose.position.x, r_amcl.pose.pose.position.y))
        rospy.loginfo('move_dis: {0}'.format(move_dis))

        if move_dis >= self.dis_unit:
            self.move_adjacency_node(dest_neighbor_node=self.next_node, sim=False, render=True)
            self.robot_x = r_amcl.pose.pose.position.x
            self.robot_y = r_amcl.pose.pose.position.y

        rospy.loginfo('cur_neighbor: {0}'.format(self.cur_neighbor))

        return


if __name__ == '__main__':
    rospy.init_node(os.path.basename(__file__).split('.')[0], log_level=rospy.INFO)
    tamp = TaskMotionPlannerDP()
    tamp.run_plan_viz()
