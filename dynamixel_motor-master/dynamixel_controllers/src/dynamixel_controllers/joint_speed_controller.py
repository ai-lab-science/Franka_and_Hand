# -*- coding: utf-8 -*-
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2010-2011, Antons Rebguns.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of University of Arizona nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from __future__ import division

__author__ = 'Antons Rebguns'
__copyright__ = 'Copyright (c) 2010-2011 Antons Rebguns'
__credits__ = 'Cara Slutter'

__license__ = 'BSD'
__maintainer__ = 'Antons Rebguns'
__email__ = 'anton@email.arizona.edu'

import rospy

from dynamixel_driver.dynamixel_const import *
from dynamixel_controllers.joint_controller import JointController

from dynamixel_msgs.msg import JointState


class JointSpeedController(JointController):
    def __init__(self, dxl_io, controller_namespace, port_namespace):
        JointController.__init__(self, dxl_io, controller_namespace, port_namespace)

        self.motor_id = rospy.get_param(self.controller_namespace + '/motor/id')
        self.initial_position_raw = rospy.get_param(self.controller_namespace + '/motor/init')
        self.min_angle_raw = rospy.get_param(self.controller_namespace + '/motor/min')
        self.max_angle_raw = rospy.get_param(self.controller_namespace + '/motor/max')
        self.min_angle_treshold = rospy.get_param(self.controller_namespace + '/motor/minAngle')
        self.max_angle_treshold = rospy.get_param(self.controller_namespace + '/motor/maxAngle')

        if rospy.has_param(self.controller_namespace + '/motor/acceleration'):
            self.acceleration = rospy.get_param(self.controller_namespace + '/motor/acceleration')
        else:
            self.acceleration = None

        self.flipped = self.min_angle_raw > self.max_angle_raw

        self.joint_state = JointState(name=self.joint_name, motor_ids=[self.motor_id])

    def initialize(self):
        # verify that the expected motor is connected and responding
        available_ids = rospy.get_param('dynamixel/%s/connected_ids' % self.port_namespace, [])
        if not self.motor_id in available_ids:
            rospy.logwarn('The specified motor id is not connected and responding.')
            rospy.logwarn('Available ids: %s' % str(available_ids))
            rospy.logwarn('Specified id: %d' % self.motor_id)
            return False

        self.RADIANS_PER_ENCODER_TICK = rospy.get_param(
            'dynamixel/%s/%d/radians_per_encoder_tick' % (self.port_namespace, self.motor_id))
        self.ENCODER_TICKS_PER_RADIAN = rospy.get_param(
            'dynamixel/%s/%d/encoder_ticks_per_radian' % (self.port_namespace, self.motor_id))

        self.joint_angle_raw = 0
        self.at_max_angle = False
        self.at_min_angle = False

        if self.flipped:
            self.min_angle = (self.initial_position_raw - self.min_angle_raw) * self.RADIANS_PER_ENCODER_TICK
            self.max_angle = (self.initial_position_raw - self.max_angle_raw) * self.RADIANS_PER_ENCODER_TICK
        else:
            self.min_angle = (self.min_angle_raw - self.initial_position_raw) * self.RADIANS_PER_ENCODER_TICK
            self.max_angle = (self.max_angle_raw - self.initial_position_raw) * self.RADIANS_PER_ENCODER_TICK

        self.ENCODER_RESOLUTION = rospy.get_param(
            'dynamixel/%s/%d/encoder_resolution' % (self.port_namespace, self.motor_id))
        self.MAX_POSITION = self.ENCODER_RESOLUTION - 1
        self.VELOCITY_PER_TICK = rospy.get_param(
            'dynamixel/%s/%d/radians_second_per_encoder_tick' % (self.port_namespace, self.motor_id))
        self.MAX_VELOCITY = rospy.get_param('dynamixel/%s/%d/max_velocity' % (self.port_namespace, self.motor_id))
        self.MIN_VELOCITY = self.VELOCITY_PER_TICK

        if self.compliance_slope is not None: self.set_compliance_slope(self.compliance_slope)
        if self.compliance_margin is not None: self.set_compliance_margin(self.compliance_margin)
        if self.compliance_punch is not None: self.set_compliance_punch(self.compliance_punch)
        if self.torque_limit is not None: self.set_torque_limit(self.torque_limit)
        if self.acceleration is not None:
            rospy.loginfo("Setting acceleration of %d to %d" % (self.motor_id, self.acceleration))
            self.dxl_io.set_acceleration(self.motor_id, self.acceleration)

        self.joint_max_speed = rospy.get_param(self.controller_namespace + '/joint_max_speed', self.MAX_VELOCITY)

        if self.joint_max_speed < self.MIN_VELOCITY:
            self.joint_max_speed = self.MIN_VELOCITY
        elif self.joint_max_speed > self.MAX_VELOCITY:
            self.joint_max_speed = self.MAX_VELOCITY

        if self.joint_speed < self.MIN_VELOCITY:
            self.joint_speed = self.MIN_VELOCITY
        elif self.joint_speed > self.joint_max_speed:
            self.joint_speed = self.joint_max_speed

        self.set_speed(self.joint_speed)
        self.dxl_io.set_speed(self.motor_id, 0)

        # self.max_angle_treshold = self.raw_to_rad(self.max_angle_treshold, self.initial_position_raw, self.flipped, self.RADIANS_PER_ENCODER_TICK)
        # self.min_angle_treshold = self.raw_to_rad(self.min_angle_treshold, self.initial_position_raw, self.flipped, self.RADIANS_PER_ENCODER_TICK)

        return True

    def pos_rad_to_raw(self, pos_rad):
        if pos_rad < self.min_angle:
            pos_rad = self.min_angle
        elif pos_rad > self.max_angle:
            pos_rad = self.max_angle
        return self.rad_to_raw(pos_rad, self.initial_position_raw, self.flipped, self.ENCODER_TICKS_PER_RADIAN)

    def spd_rad_to_raw(self, spd_rad):
        # include negative values for direction
        # if spd_rad < self.MIN_VELOCITY: spd_rad = self.MIN_VELOCITY
        # elif spd_rad > self.joint_max_speed: spd_rad = self.joint_max_speed
        # velocity of 0 means maximum, make sure that doesn't happen
        # return max(1, int(round(spd_rad / self.VELOCITY_PER_TICK)))

        # add angle limits, maybe new parameter need in yaml file
        if spd_rad == 0.0:
            return 0

        if self.at_max_angle and spd_rad > 0:
            print("At end MX " + str(self.at_max_angle))
            return 0

        if self.at_min_angle and spd_rad < 0:
            print("At end MN " + str(self.at_min_angle))
            return 0

        if spd_rad < -self.joint_max_speed:
            spd_rad = -self.joint_max_speed
        elif spd_rad > self.joint_max_speed:
            spd_rad = self.joint_max_speed

        if spd_rad > 0:
            return max(1, int(round(spd_rad / self.VELOCITY_PER_TICK)))
        elif spd_rad < 0:
            return min(-1, int(round(spd_rad / self.VELOCITY_PER_TICK)))

    def set_torque_enable(self, torque_enable):
        mcv = (self.motor_id, torque_enable)
        self.dxl_io.set_multi_torque_enabled([mcv])

    def set_speed(self, speed):
        mcv = (self.motor_id, self.spd_rad_to_raw(speed))
        self.dxl_io.set_multi_speed([mcv])

    def set_compliance_slope(self, slope):
        if slope < DXL_MIN_COMPLIANCE_SLOPE:
            slope = DXL_MIN_COMPLIANCE_SLOPE
        elif slope > DXL_MAX_COMPLIANCE_SLOPE:
            slope = DXL_MAX_COMPLIANCE_SLOPE
        mcv = (self.motor_id, slope, slope)
        self.dxl_io.set_multi_compliance_slopes([mcv])

    def set_compliance_margin(self, margin):
        if margin < DXL_MIN_COMPLIANCE_MARGIN:
            margin = DXL_MIN_COMPLIANCE_MARGIN
        elif margin > DXL_MAX_COMPLIANCE_MARGIN:
            margin = DXL_MAX_COMPLIANCE_MARGIN
        else:
            margin = int(margin)
        mcv = (self.motor_id, margin, margin)
        self.dxl_io.set_multi_compliance_margins([mcv])

    def set_compliance_punch(self, punch):
        if punch < DXL_MIN_PUNCH:
            punch = DXL_MIN_PUNCH
        elif punch > DXL_MAX_PUNCH:
            punch = DXL_MAX_PUNCH
        else:
            punch = int(punch)
        mcv = (self.motor_id, punch)
        self.dxl_io.set_multi_punch([mcv])

    def set_torque_limit(self, max_torque):
        if max_torque > 1:
            max_torque = 1.0  # use all torque motor can provide
        elif max_torque < 0:
            max_torque = 0.0  # turn off motor torque
        raw_torque_val = int(DXL_MAX_TORQUE_TICK * max_torque)
        mcv = (self.motor_id, raw_torque_val)
        self.dxl_io.set_multi_torque_limit([mcv])

    def set_acceleration_raw(self, acc):
        if acc < 0:
            acc = 0
        elif acc > 254:
            acc = 254
        self.dxl_io.set_acceleration(self.motor_id, acc)

    def process_motor_states(self, state_list):
        if self.running:
            state = state_list.motor_states[-1]
            #state = filter(lambda state: state.id == self.motor_id, state_list.motor_states)
            if state:
                #state = state[0]
                self.joint_state.motor_temps = [state.temperature]
                self.joint_state.goal_pos = self.raw_to_rad(state.goal, self.initial_position_raw, self.flipped,
                                                            self.RADIANS_PER_ENCODER_TICK)
                self.joint_state.current_pos = self.raw_to_rad(state.position, self.initial_position_raw, self.flipped,
                                                               self.RADIANS_PER_ENCODER_TICK)
                self.joint_state.error = state.error * self.RADIANS_PER_ENCODER_TICK
                self.joint_state.velocity = state.speed * self.VELOCITY_PER_TICK
                self.joint_state.load = state.load
                self.joint_state.is_moving = state.moving
                self.joint_state.header.stamp = rospy.Time.from_sec(state.timestamp)
                self.joint_angle_raw = state.position

                self.joint_state_pub.publish(self.joint_state)

                # Encoder position and tick velocity based limitation
                if (self.max_angle_treshold != self.min_angle_treshold):
                    if (self.joint_state.velocity > 0 and abs(self.max_angle_treshold - 100) < (
                            state.position + state.speed * 1.0 / 20.0)):
                        print("Joint reach its limit " + str(self.max_angle_treshold) + " at " + str(
                            state.position) + " state speed " + str(state.speed))
                        self.dxl_io.set_speed(self.motor_id, 0)
                        self.at_max_angle = True
                    elif self.joint_state.velocity != 0:
                        self.at_max_angle = False

                    if (self.joint_state.velocity < 0 and abs(self.min_angle_treshold + 100) > (
                            state.position + state.speed * 1.0 / 20.0)):
                        print(
                        "Joint reach its limit " + str(self.min_angle_treshold) + " at " + str(
                            state.position) + " vel " + str(state.speed))
                        self.dxl_io.set_speed(self.motor_id, 0)
                        self.at_min_angle = True
                    elif self.joint_state.velocity != 0:
                        self.at_min_angle = False

    def process_command(self, msg):
        speed = self.spd_rad_to_raw(msg.data)
        print(speed)
        mcv = (self.motor_id, speed)
        self.dxl_io.set_speed(self.motor_id, speed)
