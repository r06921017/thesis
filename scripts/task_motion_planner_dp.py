#!/usr/bin/env python
# -*- coding: utf8 -*-
"""
Solve task planning with dynamic programming (value iteration)
"""
from task_motion_planner_fcfs import *
import operator
from motion.robot_motions import *
# from std_msgs.msg import Int32


class TaskMotionPlannerDP(TaskMotionPlannerFCFS):
    def __init__(self):
        """
        self.cur_node: the node where robot starts to move, initial at charge (2), type: int
        """
        TaskMotionPlannerFCFS.__init__(self)
        self.shortest_path = nx.floyd_warshall_numpy(self.map_graph)

        # for real world motion
        set_initial_pose(loc[self.cur_node][0], loc[self.cur_node][1], loc[self.cur_node][2], self.cur_node)
        self.robot_x, self.robot_y = loc[self.cur_node][0], loc[self.cur_node][1]
        self.sac = actionlib.SimpleActionClient('move_base', MoveBaseAction)  # type: SimpleActionClient
        self.move_lock = False
        self.dis_unit = 0.15

        self.amcl_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pos_cb)
        # self.motion_sub = rospy.Subscriber('/thesis/next_node', Int32, self.nav_cb, queue_size=5)
        # self.motion_pub = rospy.Publisher('/thesis/next_node', Int32, queue_size=5)

        print '----------------------------------------'
        print self.shortest_path
        print self.shortest_path.shape
        print '----------------------------------------'
        rospy.loginfo('TAMP Initialized!')

    def value_iter(self, in_node=None):
        # Check in_node exists or not
        if in_node is None:
            _in_neighbor = list(self.cur_neighbor)  # copy the list, not changing self.cur_neighbor
            in_node = self.cur_node
        else:
            _in_neighbor = list(self.adjacency_matrix[in_node].astype(int).tolist())

        rospy.loginfo('value_iter, in_node: {0}'.format(in_node))

        # Compare the accumulated reward of neighbor, move to the maximum one.
        neighbor_node = np.where(np.array(_in_neighbor) > 0)[0].tolist()
        candidate_steps = dict()  # {neighbor_node: {'reward': float, 'neighbor': np.array}}

        for n in neighbor_node:
            temp_neighbor = self.move_adjacency_node(dest_neighbor_node=n, in_node=in_node)  # assuming moving toward the neighbor node n.
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

            rospy.loginfo('total reward in candidate node {0}: {1}'.format(n, total_reward))

            candidate_steps[n] = {'reward': total_reward, 'neighbor': temp_neighbor}

        # Pick the node with maximum accumulated reward
        _temp_next_node = max(candidate_steps, key=lambda x: candidate_steps[x]['reward'])

        if _temp_next_node == 3 or _temp_next_node == 4:
            self.value_iter(in_node=_temp_next_node)
        else:
            self.next_node = _temp_next_node
            rospy.loginfo('plan_task result: {0}'.format(self.next_node))

        return

    def plan_task(self, in_instructions):
        rospy.loginfo('Start task planning!')
        self.move_lock = True
        # Convert InstructionArray into dictionary
        if type(in_instructions) == thesis.msg._InstructionArray.InstructionArray:
            for instr in in_instructions.data:
                self.instr_dest_dict[instr.destination].add(instr.id)
                self.instr_dict[instr.id] = instr

            # print 'self.instr_dict: ', self.instr_dict

        rospy.logdebug('len(self.instr_dict.keys()): {0}'.format(len(self.instr_dict.keys())))

        if len(self.instr_dict.keys()) > 0:  # if there exists instructions
            if len(self.instr_dest_dict[self.cur_node]) > 0:
                # for two-stage instruction, count the instructions that still have previous instr
                temp_count = 0
                for instr_id in self.instr_dest_dict[self.cur_node]:
                    if self.instr_dict[instr_id].prev_id in self.instr_dict.keys():
                        temp_count += 1

                # all the instructions in cur_node are sequential
                if temp_count == len(self.instr_dest_dict[self.cur_node]):
                    self.value_iter()  # update self.next_node

                # there are instructions exist in cur_node
                else:
                    self.next_node = self.cur_node

                # for single stage instruction
                # self.next_node = self.cur_node

            else:
                self.value_iter()  # update self.next_node

            rospy.loginfo('plan_task result: {0}'.format(self.next_node))
            # self.motion_pub.publish(self.next_node)
            self.move_lock = False
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

    def plan_motion(self):
        """
        navigation callback for Pepper
        :return: None, only move the robot
        """
        # Moving toward node
        if self.move_lock:
            rospy.loginfo('move_lock .........................................')
            shutdown()

        else:
            if self.cur_node != self.next_node:
                rospy.loginfo('Start simple_mode in nav_cb.')
                rospy.sleep(0.2)
                simple_move_base(self.sac, loc[self.next_node][0], loc[self.next_node][1], loc[self.next_node][2])
                self.cur_node = self.next_node
                self.cur_neighbor = self.adjacency_matrix[self.cur_node].astype(int)
                rospy.set_param('/thesis/pepper_location', self.cur_node)
                rospy.loginfo('Change cur_node to: {0}'.format(self.cur_node))

            else:
                # Reach node
                if len(self.instr_dest_dict[self.cur_node]) > 0:
                    # add to eliminate amcl divergence
                    # set_initial_pose(x=self.robot_x,
                    #                  y=self.robot_y,
                    #                  yaw=get_cur_pos(pos_topic='/amcl_pose', t=0.5)[2],
                    #                  init_location=self.cur_node)
                    # end: add to eliminate amcl divergence

                    # Create reward_dict = {'id (int)': 'reward (float)'}
                    reward_dict = dict()
                    for idx in self.instr_dest_dict[self.cur_node]:
                        # Check two-stage instruction
                        if self.instr_dict[idx].prev_id not in self.instr_dict.keys():
                            reward_dict[self.instr_dict[idx].id] = self.instr_dict[idx].r

                    # Stop the move_base at first
                    self.move_lock = True
                    shutdown()
                    # Sort the instructions with the max reward
                    for r in sorted(reward_dict.items(), key=operator.itemgetter(1), reverse=True):
                        do_instr = self.instr_dict[r[0]]
                        simple_rotate(loc[self.cur_node][2] - get_cur_pos(pos_topic='/amcl_pose', t=0.5)[2])

                        rospy.loginfo('Do instr {0}: {1}'.format(do_instr.id, do_instr.function))
                        rospy.sleep(do_instr.duration)
                        del self.instr_dict[r[0]]
                        self.show_instr()

                    # Reset the set when all the tasks in the instructions are done.
                    self.instr_dest_dict[self.cur_node].clear()
                    # rospy.logdebug('self.instr_dest_dict: {0}'.format(self.instr_dest_dict))

                    # Convert undo_tasks to a list() and publish to /thesis/instruction_buffer
                    undo_instr_list = list()
                    for key, value in self.instr_dict.iteritems():
                        undo_instr_list.append(value)
                    self.task_pub.publish(undo_instr_list)

                    # Relaunch move_base
                    relaunch_move_base()
                    self.move_lock = False

                else:
                    rospy.loginfo('No instructions on task {0}'.format(self.cur_node))
                    rospy.sleep(2)
                    if len(self.instr_dict) > 0:
                        self.plan_task(self.instr_dict)

        return

    def run_plan(self):
        rospy.loginfo('Start TAMP!')

        # Publish the initial position node of the robot to visualization
        rospy.sleep(0.5)
        self.viz_node_pub.publish(String(data=str(self.cur_node)))

        # Start running motion planning and visualization
        rate = rospy.Rate(1.0 / self.time_step)
        while not rospy.is_shutdown():
            self.plan_motion()
            rate.sleep()

        return

    def pos_cb(self, r_amcl):
        # rospy.loginfo('pos_cb!')
        move_dis = norm([r_amcl.pose.pose.position.x-self.robot_x, r_amcl.pose.pose.position.y-self.robot_y])

        # rospy.loginfo('cur_pos: {0}, {1}'.format(r_amcl.pose.pose.position.x, r_amcl.pose.pose.position.y))
        # rospy.loginfo('move_dis: {0}'.format(move_dis))

        if move_dis >= self.dis_unit:
            # self.move_adjacency_node(dest_neighbor_node=self.next_node, sim=False, render=True)
            self.robot_x = r_amcl.pose.pose.position.x
            self.robot_y = r_amcl.pose.pose.position.y

        # rospy.loginfo('cur_neighbor: {0}'.format(self.cur_neighbor))
        # rospy.loginfo('cur_node: {0}'.format(self.cur_node))

        return


if __name__ == '__main__':
    rospy.init_node(os.path.basename(__file__).split('.')[0], log_level=rospy.DEBUG)
    tamp = TaskMotionPlannerDP()
    # tamp.run_plan_viz()  # this is for simulation
    tamp.run_plan()  # this is for real world application
