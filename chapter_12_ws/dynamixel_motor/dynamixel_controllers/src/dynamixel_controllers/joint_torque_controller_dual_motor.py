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

__license__ = 'BSD'
__maintainer__ = 'Antons Rebguns'
__email__ = 'anton@email.arizona.edu'


import rospy

from dynamixel_driver.dynamixel_const import *
from dynamixel_controllers.joint_controller import JointController

from dynamixel_msgs.msg import JointState

class JointTorqueControllerDualMotor(JointController):
    def __init__(self, dxl_io, controller_namespace, port_namespace):
        JointController.__init__(self, dxl_io, controller_namespace, port_namespace)
        
        self.master_id = rospy.get_param(self.controller_namespace + '/motor_master/id')
        self.master_initial_position_raw = rospy.get_param(self.controller_namespace + '/motor_master/init')
        self.master_min_angle_raw = rospy.get_param(self.controller_namespace + '/motor_master/min')
        self.master_max_angle_raw = rospy.get_param(self.controller_namespace + '/motor_master/max')
        
        self.slave_id = rospy.get_param(self.controller_namespace + '/motor_slave/id')
        
        self.flipped = self.master_min_angle_raw > self.master_max_angle_raw
        self.last_commanded_torque = 0.0
        
        self.joint_state = JointState(name=self.joint_name, motor_ids=[self.master_id, self.slave_id])


    def initialize(self):
        # verify that the expected motor is connected and responding
        available_ids = rospy.get_param('dynamixel/%s/connected_ids' % self.port_namespace, [])
        if not (self.master_id in available_ids and
                self.slave_id in available_ids):
            rospy.logwarn('The specified motor id is not connected and responding.')
            rospy.logwarn('Available ids: %s' % str(available_ids))
            rospy.logwarn('Specified ids: %d %d' % (self.master_id, self.slave_id))
            return False
            
        self.RADIANS_PER_ENCODER_TICK = rospy.get_param('dynamixel/%s/%d/radians_per_encoder_tick' % (self.port_namespace, self.master_id))
        self.ENCODER_TICKS_PER_RADIAN = rospy.get_param('dynamixel/%s/%d/encoder_ticks_per_radian' % (self.port_namespace, self.master_id))
        
        if self.flipped:
            self.master_min_angle = (self.master_initial_position_raw - self.master_min_angle_raw) * self.RADIANS_PER_ENCODER_TICK
            self.master_max_angle = (self.master_initial_position_raw - self.master_max_angle_raw) * self.RADIANS_PER_ENCODER_TICK
        else:
            self.master_min_angle = (self.master_min_angle_raw - self.master_initial_position_raw) * self.RADIANS_PER_ENCODER_TICK
            self.master_max_angle = (self.master_max_angle_raw - self.master_initial_position_raw) * self.RADIANS_PER_ENCODER_TICK
            
        self.ENCODER_RESOLUTION = rospy.get_param('dynamixel/%s/%d/encoder_resolution' % (self.port_namespace, self.master_id))
        self.MAX_POSITION = self.ENCODER_RESOLUTION - 1
        self.VELOCITY_PER_TICK = rospy.get_param('dynamixel/%s/%d/radians_second_per_encoder_tick' % (self.port_namespace, self.master_id))
        self.MAX_VELOCITY = rospy.get_param('dynamixel/%s/%d/max_velocity' % (self.port_namespace, self.master_id))
        self.MIN_VELOCITY = self.VELOCITY_PER_TICK
        
        if self.compliance_slope is not None: self.set_compliance_slope(self.compliance_slope)
        if self.compliance_margin is not None: self.set_compliance_margin(self.compliance_margin)
        if self.compliance_punch is not None: self.set_compliance_punch(self.compliance_punch)
        if self.torque_limit is not None: self.set_torque_limit(self.torque_limit)
        
        self.joint_max_speed = rospy.get_param(self.controller_namespace + '/joint_max_speed', self.MAX_VELOCITY)
        
        if self.joint_max_speed < self.MIN_VELOCITY: self.joint_max_speed = self.MIN_VELOCITY
        elif self.joint_max_speed > self.MAX_VELOCITY: self.joint_max_speed = self.MAX_VELOCITY
        
        if self.joint_speed < self.MIN_VELOCITY: self.joint_speed = self.MIN_VELOCITY
        elif self.joint_speed > self.joint_max_speed: self.joint_speed = self.joint_max_speed
        
        self.set_speed(0.0)
        
        return True


    def set_torque_enable(self, torque_enable):
        mcv_master = (self.master_id, torque_enable)
        mcv_slave = (self.slave_id, torque_enable)
        self.dxl_io.set_multi_torque_enabled([mcv_master, mcv_slave])


    def set_speed(self, speed):
        if speed < -self.joint_max_speed: speed = -self.joint_max_speed
        elif speed > self.joint_max_speed: speed = self.joint_max_speed
        self.last_commanded_torque = speed
        speed_raw = int(round(speed / self.VELOCITY_PER_TICK))
        mcv_master = (self.master_id, speed_raw)
        mcv_slave = (self.slave_id, -mcv_master[1])
        self.dxl_io.set_multi_speed([mcv_master, mcv_slave])


    def set_compliance_slope(self, slope):
        if slope < DXL_MIN_COMPLIANCE_SLOPE: slope = DXL_MIN_COMPLIANCE_SLOPE
        elif slope > DXL_MAX_COMPLIANCE_SLOPE: slope = DXL_MAX_COMPLIANCE_SLOPE
        mcv_master = (self.master_id, slope, slope)
        mcv_slave = (self.slave_id, slope, slope)
        self.dxl_io.set_multi_compliance_slopes([mcv_master, mcv_slave])


    def set_compliance_margin(self, margin):
        if margin < DXL_MIN_COMPLIANCE_MARGIN: margin = DXL_MIN_COMPLIANCE_MARGIN
        elif margin > DXL_MAX_COMPLIANCE_MARGIN: margin = DXL_MAX_COMPLIANCE_MARGIN
        else: margin = int(margin)
        mcv_master = (self.master_id, margin, margin)
        mcv_slave = (self.slave_id, margin, margin)
        self.dxl_io.set_multi_compliance_margins([mcv_master, mcv_slave])


    def set_compliance_punch(self, punch):
        if punch < DXL_MIN_PUNCH: punch = DXL_MIN_PUNCH
        elif punch > DXL_MAX_PUNCH: punch = DXL_MAX_PUNCH
        else: punch = int(punch)
        mcv_master = (self.master_id, punch)
        mcv_slave = (self.slave_id, punch)
        self.dxl_io.set_multi_punch([mcv_master, mcv_slave])


    def set_torque_limit(self, max_torque):
        if max_torque > 1: max_torque = 1.0
        elif max_torque < 0: max_torque = 0.0     # turn off motor torque
        raw_torque_val = int(DXL_MAX_TORQUE_TICK * max_torque)
        mcv_master = (self.master_id, raw_torque_val)
        mcv_slave = (self.slave_id, raw_torque_val)
        self.dxl_io.set_multi_torque_limit([mcv_master, mcv_slave])


    def process_motor_states(self, state_list):
        if self.running:
            states = {}
            
            for state in state_list.motor_states:
                if state.id in [self.master_id, self.slave_id]: states[state.id] = state
                
            if states:
                state = states[self.master_id]
                self.joint_state.motor_temps = [state.temperature, states[self.slave_id].temperature]
                self.joint_state.goal_pos = self.last_commanded_torque
                self.joint_state.current_pos = self.raw_to_rad(state.position, self.master_initial_position_raw, self.flipped, self.RADIANS_PER_ENCODER_TICK)
                self.joint_state.error = 0.0
                self.joint_state.velocity = state.speed * self.VELOCITY_PER_TICK
                self.joint_state.load = state.load
                self.joint_state.is_moving = state.moving
                self.joint_state.header.stamp = rospy.Time.from_sec(state.timestamp)
                
                self.joint_state_pub.publish(self.joint_state)


    def process_command(self, msg):
        self.set_speed(msg.data)

