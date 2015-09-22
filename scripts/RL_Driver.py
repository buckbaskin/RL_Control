#!/usr/bin/env python

'''
Copyright 2015 William Baskin

/*****************************************
 LICENSE SUMMARY

 This package is licensed under the 
    MIT License. Please see the LICENSE.md
    file in the root folder for the 
    complete license.

 *****************************************/
 '''
'''
 Reinforcement Learning Driver

 The Driver follows the interface pattern of the
 drive_stack/Driver.

 This Driver looks at the past data and
 commands that it outputs and attempts to learn
 how the robot moves, so it can better match the 
 current published pose. It relies on a handful of
 subsidiary nodes to assist with the transition 
 function approximation.

 This is based on the reinforcement learning concept
 in artificial intelligence. The basic premise is that
 the agent will learn the value of an action A given a
 state S and a pose P. This will allow the robot to 
 take an action A* that is action it thinks will 
 result in the highest actual value. The actual value
 of an action is going to be measured by the distance 
 of the next state S' from the desired pose P'..
 This distance will be measured in x,y and theta space.

 This will allow the robot to learn what commands are
 appropriate for what situation, without actually 
 having to "tell" the robot how to controller itself.
 ''' #pylint: disable=pointless-string-statement

import rospy
import math

from tf import transformations

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose

def quat_to_euler(q):
    # Use the tf.transformations method to convert yaw from quaternion
    return transformations.euler_from_quaternion(q)

def euler_to_quat(heading):
    # Use the tf.transformations method to convert quaternion from yaw
    return transformations.quaternion_about_axis(heading, (0,0,1))

def err_along_axis(state):
    current = state[0]
    desired = state[1]
    # TODO(buckbaskin): calculate error along axis
    return 0

def err_off_axis(state):
    current = state[0]
    desired = state[1]
    # TODO(buckbaskin): calculate error perpendicular to axis
    return 0

def err_heading(state):
    current = state[0]
    desired = state[1]
    c_orientation = current.pose.pose.orientation
    d_orientation = desired.pose.pose.orientation
    c_theta = quat_to_euler(c_orientation)
    d_theta = quat_to_euler(d_orientation)
    return d_theta - c_theta

class QManager(object):
    def __init__(self):
        self.functions = []
        self.a = { # alpha
                        'learning':0.2,
                        'exploitation':0.1
                    }

    def add_function(self, f):
        self.functions.append((f,0,))

    def f(self):
        return self.functions

    def update(self, l):
        self.functions = l

    def standard(self):
        self.add_function(err_along_axis)
        self.add_function(err_off_axis)
        self.add_function(err_heading)

class Controller(object):
    pass

class LearningController(Controller):
    def __init__(self):
        rospy.init_node('learn_control', anonymous=True)

        # score is measured from the current pose and state, 
        # and applied to the stored (state+pose, action)

        self.state = None # last state
        self.action = None # last action
        self.pose = None # last Pose
        self.nxt_state = None # state to measure score
        self.nxt_pose = None # pose to measure score

        self.qm = QManager()
        self.qm.standard()

        # PUB/SUB Setup
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.loc_sub = rospy.Subscriber('odom', Odometry, self.loc_cb)
        self.pose_sub = rospy.Subscriber('/path/pose', Odometry, self.pose_cb)

    def loc_cb(self, odom):
        # take in a new location, update, and publish a new command message
        self.update_with_loc(odom)
        cmd = self.loc_to_cmd(odom)
        self.cmd_pub.publish(cmd)

    def pose_cb(self, pose):
        # take in a new pose, and update. publish a command message
        self.update_with_pose(pose)
        cmd = self.pose_to_cmd(pose)
        self.cmd_pub.publish(cmd)

    # Q LEARNING - the actual learning

    def update_with_loc(self, odom):
        # Update the Q learning based on new data
        if self.state and self.action and self.pose:
            score = self.score(self.pose, odom)
            #     ( state , action, real value)
            self.q_update((self.state, self.pose,), self.action, score)
            self.state = odom
        else:
            # no data to compare to ahead of time on first step
            self.state = odom
            self.action = Twist()

    def update_with_pose(self, pose):
        # Update the Q learning based on new data
        if self.state and self.action and self.pose:
            score = self.score(pose, self.state)
            #     ( state , action, real value)
            self.q_update((self.state, self.pose,), self.action, score)
            self.pose = pose
        else:
            # no data to compare to ahead of time on first step
            self.pose = pose
            self.action = Twist()

    def q_update(self, state, action, real_value):
        '''
        state: pair (Odometry, Odometry) for robot odom, desired pose
        action: Twist for cmd_vel
        real_value: "distance" evaluated by score function
        '''
        # current = state[0]
        # desired = state[1]
        action = action
        reward = real_value
        
        # for each W_new
        # W_new = (1-alpha)*W_old + alpha*(Reward + max(W_old*f(s') for all s')) / f(s)
        new_l = []
        alpha = self.qm.a['learning']
        for f, w in self.qm.f():
            w_new = (1-alpha)*w + alpha*(reward + self.find_max_child_q(state, action)) / f(state)
            new_l.append((f, w_new,))

        self.qm.update(new_l)

    def q(self, state, action):
        # for each W_new
        # W_new = (1-alpha)*W_old + alpha*(Reward + max(W_old*f(s') for all s')) / f(s)
        '''
        state: pair (Odometry, Odometry) for robot odom, desired pose
        action: Twist for cmd_vel
        real_value: "distance" evaluated by score function
        '''
        # current = state[0]
        # desired = state[1]
        action = action
        
        q_accum = 0
        for f, w in self.qm.f():
            q_accum = q_accum + w*f(state)
            

    # Q LEARNING - helper methods

    def transition(self, state, twist, dt):
        desired = state[1]
        current = state[0]
        # do something
        n = Odometry()
        n.header = current.header
        n.twist.twist = self.update_twist(current.twist.twist, twist)
        n.pose.pose = self.update_pose(current.pose.pose, current.twist.twist, n.twist.twist, dt)
        return (n, desired, )

    def update_pose(self, pose, old_twist, new_twist, dt):
        p = Pose()
        v_avg = old_twist.linear.x/2 + new_twist.linear.x/2
        w_avg = old_twist.angular.z/2 + new_twist.angular.z/2
        
        theta_old = quat_to_euler(pose.orientation)
        theta_new = theta_old+dt*w_avg
        theta_avg = theta_old/2 + theta_new/2
        
        dx = v_avg*math.cos(theta_avg)
        dy = v_avg*math.sin(theta_avg)

        p.position.x = pose.position.x + dx
        p.position.y = pose.position.y + dy
        p.orientation = euler_to_quat(theta_new)
        
        return p

    def update_twist(self, twist, cmd):
        max_a = .1
        max_alpha = .1

        t = Twist()

        if twist.linear.x > cmd.linear.x:
            t.linear.x = max(twist.linear.x - max_a, cmd.linear.x)
        elif twist.linear.x < cmd.linear.x:
            t.linear.x = min(twist.linear.x + max_a, cmd.linear.x)
        else:
            t.linear.x = twist.linear.x

        if twist.angular.z > cmd.angular.z:
            t.angular.z = max(twist.angular.z - max_alpha, cmd.angular.z)
        elif twist.angular.z < cmd.angular.z:
            t.angular.z = min(twist.angular.z + max_alpha, cmd.angular.z)
        else:
            t.angular.z = twist.angular.z

        return t

    def score(self, pose, state):
        max_x = 15
        max_y = 15
        max_theta = 6.28

        # calculate normalized distances in x, y, theta
        dx = (pose.pose.pose.position.x - state.pose.pose.position.x)/max_x
        dy = (pose.pose.pose.position.y - state.pose.pose.position.y)/max_y
        dtheta = (quat_to_euler(pose.pose.pose.orientation) - quat_to_euler(pose.pose.pose.orientation))/max_theta

        return math.sqrt(dx*dx + dy*dy + dtheta*dtheta)

    def find_max_child_q(self, state, action):
        # state is s, action is a => state s', that is what we are evaluating.
        # We need to find the q for the best action leaving s'
        hz = 10 # check this/set this, or make it a method parameter
        dt = 1/hz
        s_prime = self.transition(state, action, dt)
        return self.q(s_prime, self.find_max_child_twist(state))

    def find_max_child_twist(self, state):
        # find the best child of the ^ state

        # "iterate" through possible children of the transition model's first output
        # but the state space is continuous, bounded by motion model
        # find max child by heuristically selecting a possible maximum, then gradient ascent
        # iterate/gradient through possible Twists
        # score/f(t) = q(^state, t)

        best = self.heuristic_best_child_twist(state)
        best_q = self.q(state, best)
        # improve 100 times (currently fixed step size)
        for _ in range(0,100):
            # get neighbors of current best
            options = self.iterate_twist(best)
            for twist in options:
                twist_q = self.q(state, twist)
                # if a neighbor is better, take that
                if twist_q > best_q:
                    best = twist
                    best_q = twist_q

        return Twist()

    def heuristic_best_child_twist(self, state):
        # For a given state, guess a best Twist to return current odom to match desired
        current = state[0]
        # desired = state[1]
        e_along = err_along_axis(state)
        e_offset = err_off_axis(state)
        # e_theta = err_heading(state)

        cur_theta = quat_to_euler(current.pose.pose.orientation)
        sug_theta = -math.atan(e_offset*2)
        d_theta = sug_theta - cur_theta
        sug_w = -d_theta
        sug_w = max(sug_w, -1)
        sug_w = min(sug_w, 1)

        sug_v = -e_along
        suggested_twist = Twist()
        suggested_twist.linear.x = sug_v
        suggested_twist.angular.z = sug_w

        return suggested_twist


    def iterate_twist(self, t):
        # for a given twist, choose neighbors in linear vel, angular vel space
        step_size = .02
        l = []
        
        t0 = Twist()
        t0.linear.x = t.linear.x + step_size
        t0.angular.z = t.angular.z

        t1 = Twist()
        t1.linear.x = t.linear.x - step_size
        t1.angular.z = t.angular.z

        t2 = Twist()
        t2.linear.x = t.linear.x
        t2.angular.z = t.angular.z + step_size

        t3 = Twist()
        t3.linear.x = t.linear.x
        t3.angular.z = t.angular.z - step_size

        l.append(t0)
        l.append(t1)
        l.append(t2)
        l.append(t3)

        return l

    # CONVERT Q LEARNING TO ACTION

    def loc_to_cmd(self, loc):
        self.state = loc
        if not self.state or not self.action or not self.pose:
            # some of the data has not yet been recieved
            self.action = Twist()
        else:
            # calculate optimal Twist based on passed info, last pose, new location
            self.action = self.find_max_child_twist((self.state, self.pose,))

        return self.action


    def pose_to_cmd(self, pose):
        self.pose = pose
        if not self.state or not self.action or not self.pose:
            # some of the data has not yet been recieved
            self.action = Twist()
        else:
            # calculate optimal Twist based on passed info, last location, new pose
            self.action = self.find_max_child_twist((self.state, self.pose,))
        
        return self.action