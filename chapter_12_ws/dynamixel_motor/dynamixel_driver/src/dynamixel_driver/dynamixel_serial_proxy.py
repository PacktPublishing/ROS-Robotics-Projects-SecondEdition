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


__author__ = 'Antons Rebguns'
__copyright__ = 'Copyright (c) 2010-2011 Antons Rebguns'
__credits__ = 'Cody Jorgensen, Cara Slutter'

__license__ = 'BSD'
__maintainer__ = 'Antons Rebguns'
__email__ = 'anton@email.arizona.edu'


import math
import sys
import errno
from collections import deque
from threading import Thread
from collections import defaultdict

import roslib
roslib.load_manifest('dynamixel_driver')

import rospy
import dynamixel_io
from dynamixel_driver.dynamixel_const import *

from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue

from dynamixel_msgs.msg import MotorState
from dynamixel_msgs.msg import MotorStateList

class SerialProxy():
    def __init__(self,
                 port_name='/dev/ttyUSB0',
                 port_namespace='ttyUSB0',
                 baud_rate='1000000',
                 min_motor_id=1,
                 max_motor_id=25,
                 update_rate=5,
                 diagnostics_rate=1,
                 error_level_temp=75,
                 warn_level_temp=70,
                 readback_echo=False):
        self.port_name = port_name
        self.port_namespace = port_namespace
        self.baud_rate = baud_rate
        self.min_motor_id = min_motor_id
        self.max_motor_id = max_motor_id
        self.update_rate = update_rate
        self.diagnostics_rate = diagnostics_rate
        self.error_level_temp = error_level_temp
        self.warn_level_temp = warn_level_temp
        self.readback_echo = readback_echo
        
        self.actual_rate = update_rate
        self.error_counts = {'non_fatal': 0, 'checksum': 0, 'dropped': 0}
        self.current_state = MotorStateList()
        self.num_ping_retries = 5
        
        self.motor_states_pub = rospy.Publisher('motor_states/%s' % self.port_namespace, MotorStateList, queue_size=1)
        self.diagnostics_pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=1)

    def connect(self):
        try:
            self.dxl_io = dynamixel_io.DynamixelIO(self.port_name, self.baud_rate, self.readback_echo)
            self.__find_motors()
        except dynamixel_io.SerialOpenError, e:
            rospy.logfatal(e.message)
            sys.exit(1)
            
        self.running = True
        if self.update_rate > 0: Thread(target=self.__update_motor_states).start()
        if self.diagnostics_rate > 0: Thread(target=self.__publish_diagnostic_information).start()

    def disconnect(self):
        self.running = False

    def __fill_motor_parameters(self, motor_id, model_number):
        """
        Stores some extra information about each motor on the parameter server.
        Some of these paramters are used in joint controller implementation.
        """
        angles = self.dxl_io.get_angle_limits(motor_id)
        voltage = self.dxl_io.get_voltage(motor_id)
        voltages = self.dxl_io.get_voltage_limits(motor_id)
        
        rospy.set_param('dynamixel/%s/%d/model_number' %(self.port_namespace, motor_id), model_number)
        rospy.set_param('dynamixel/%s/%d/model_name' %(self.port_namespace, motor_id), DXL_MODEL_TO_PARAMS[model_number]['name'])
        rospy.set_param('dynamixel/%s/%d/min_angle' %(self.port_namespace, motor_id), angles['min'])
        rospy.set_param('dynamixel/%s/%d/max_angle' %(self.port_namespace, motor_id), angles['max'])
        
        torque_per_volt = DXL_MODEL_TO_PARAMS[model_number]['torque_per_volt']
        rospy.set_param('dynamixel/%s/%d/torque_per_volt' %(self.port_namespace, motor_id), torque_per_volt)
        rospy.set_param('dynamixel/%s/%d/max_torque' %(self.port_namespace, motor_id), torque_per_volt * voltage)
        
        velocity_per_volt = DXL_MODEL_TO_PARAMS[model_number]['velocity_per_volt']
        rpm_per_tick = DXL_MODEL_TO_PARAMS[model_number]['rpm_per_tick']
        rospy.set_param('dynamixel/%s/%d/velocity_per_volt' %(self.port_namespace, motor_id), velocity_per_volt)
        rospy.set_param('dynamixel/%s/%d/max_velocity' %(self.port_namespace, motor_id), velocity_per_volt * voltage)
        rospy.set_param('dynamixel/%s/%d/radians_second_per_encoder_tick' %(self.port_namespace, motor_id), rpm_per_tick * RPM_TO_RADSEC)
        
        encoder_resolution = DXL_MODEL_TO_PARAMS[model_number]['encoder_resolution']
        range_degrees = DXL_MODEL_TO_PARAMS[model_number]['range_degrees']
        range_radians = math.radians(range_degrees)
        rospy.set_param('dynamixel/%s/%d/encoder_resolution' %(self.port_namespace, motor_id), encoder_resolution)
        rospy.set_param('dynamixel/%s/%d/range_degrees' %(self.port_namespace, motor_id), range_degrees)
        rospy.set_param('dynamixel/%s/%d/range_radians' %(self.port_namespace, motor_id), range_radians)
        rospy.set_param('dynamixel/%s/%d/encoder_ticks_per_degree' %(self.port_namespace, motor_id), encoder_resolution / range_degrees)
        rospy.set_param('dynamixel/%s/%d/encoder_ticks_per_radian' %(self.port_namespace, motor_id), encoder_resolution / range_radians)
        rospy.set_param('dynamixel/%s/%d/degrees_per_encoder_tick' %(self.port_namespace, motor_id), range_degrees / encoder_resolution)
        rospy.set_param('dynamixel/%s/%d/radians_per_encoder_tick' %(self.port_namespace, motor_id), range_radians / encoder_resolution)
        
        # keep some parameters around for diagnostics
        self.motor_static_info[motor_id] = {}
        self.motor_static_info[motor_id]['model'] = DXL_MODEL_TO_PARAMS[model_number]['name']
        self.motor_static_info[motor_id]['firmware'] = self.dxl_io.get_firmware_version(motor_id)
        self.motor_static_info[motor_id]['delay'] = self.dxl_io.get_return_delay_time(motor_id)
        self.motor_static_info[motor_id]['min_angle'] = angles['min']
        self.motor_static_info[motor_id]['max_angle'] = angles['max']
        self.motor_static_info[motor_id]['min_voltage'] = voltages['min']
        self.motor_static_info[motor_id]['max_voltage'] = voltages['max']

    def __find_motors(self):
        rospy.loginfo('%s: Pinging motor IDs %d through %d...' % (self.port_namespace, self.min_motor_id, self.max_motor_id))
        self.motors = []
        self.motor_static_info = {}
        
        for motor_id in range(self.min_motor_id, self.max_motor_id + 1):
            for trial in range(self.num_ping_retries):
                try:
                    result = self.dxl_io.ping(motor_id)
                except Exception as ex:
                    rospy.logerr('Exception thrown while pinging motor %d - %s' % (motor_id, ex))
                    continue
                    
                if result:
                    self.motors.append(motor_id)
                    break
                    
        if not self.motors:
            rospy.logfatal('%s: No motors found.' % self.port_namespace)
            sys.exit(1)
            
        counts = defaultdict(int)
        
        to_delete_if_error = []
        for motor_id in self.motors:
            for trial in range(self.num_ping_retries):
                try:
                    model_number = self.dxl_io.get_model_number(motor_id)
                    self.__fill_motor_parameters(motor_id, model_number)
                except Exception as ex:
                    rospy.logerr('Exception thrown while getting attributes for motor %d - %s' % (motor_id, ex))
                    if trial == self.num_ping_retries - 1: to_delete_if_error.append(motor_id)
                    continue
                    
                counts[model_number] += 1
                break
                
        for motor_id in to_delete_if_error:
            self.motors.remove(motor_id)
            
        rospy.set_param('dynamixel/%s/connected_ids' % self.port_namespace, self.motors)
        
        status_str = '%s: Found %d motors - ' % (self.port_namespace, len(self.motors))
        for model_number,count in counts.items():
            if count:
                model_name = DXL_MODEL_TO_PARAMS[model_number]['name']
                status_str += '%d %s [' % (count, model_name)
                
                for motor_id in self.motors:
                    if self.motor_static_info[motor_id]['model'] == model_name:
                        status_str += '%d, ' % motor_id
                        
                status_str = status_str[:-2] + '], '
                
        rospy.loginfo('%s, initialization complete.' % status_str[:-2])

    def __update_motor_states(self):
        num_events = 50
        rates = deque([float(self.update_rate)]*num_events, maxlen=num_events)
        last_time = rospy.Time.now()
        
        rate = rospy.Rate(self.update_rate)
        while not rospy.is_shutdown() and self.running:
            # get current state of all motors and publish to motor_states topic
            motor_states = []
            for motor_id in self.motors:
                try:
                    state = self.dxl_io.get_feedback(motor_id)
                    if state:
                        motor_states.append(MotorState(**state))
                        if dynamixel_io.exception: raise dynamixel_io.exception
                except dynamixel_io.FatalErrorCodeError, fece:
                    rospy.logerr(fece)
                except dynamixel_io.NonfatalErrorCodeError, nfece:
                    self.error_counts['non_fatal'] += 1
                    rospy.logdebug(nfece)
                except dynamixel_io.ChecksumError, cse:
                    self.error_counts['checksum'] += 1
                    rospy.logdebug(cse)
                except dynamixel_io.DroppedPacketError, dpe:
                    self.error_counts['dropped'] += 1
                    rospy.logdebug(dpe.message)
                except OSError, ose:
                    if ose.errno != errno.EAGAIN:
                        rospy.logfatal(errno.errorcode[ose.errno])
                        rospy.signal_shutdown(errno.errorcode[ose.errno])
                        
            if motor_states:
                msl = MotorStateList()
                msl.motor_states = motor_states
                self.motor_states_pub.publish(msl)
                
                self.current_state = msl
                
                # calculate actual update rate
                current_time = rospy.Time.now()
                rates.append(1.0 / (current_time - last_time).to_sec())
                self.actual_rate = round(sum(rates)/num_events, 2)
                last_time = current_time
                
            rate.sleep()

    def __publish_diagnostic_information(self):
        diag_msg = DiagnosticArray()
        
        rate = rospy.Rate(self.diagnostics_rate)
        while not rospy.is_shutdown() and self.running:
            diag_msg.status = []
            diag_msg.header.stamp = rospy.Time.now()
            
            status = DiagnosticStatus()
            
            status.name = 'Dynamixel Serial Bus (%s)' % self.port_namespace
            status.hardware_id = 'Dynamixel Serial Bus on port %s' % self.port_name
            status.values.append(KeyValue('Baud Rate', str(self.baud_rate)))
            status.values.append(KeyValue('Min Motor ID', str(self.min_motor_id)))
            status.values.append(KeyValue('Max Motor ID', str(self.max_motor_id)))
            status.values.append(KeyValue('Desired Update Rate', str(self.update_rate)))
            status.values.append(KeyValue('Actual Update Rate', str(self.actual_rate)))
            status.values.append(KeyValue('# Non Fatal Errors', str(self.error_counts['non_fatal'])))
            status.values.append(KeyValue('# Checksum Errors', str(self.error_counts['checksum'])))
            status.values.append(KeyValue('# Dropped Packet Errors', str(self.error_counts['dropped'])))
            status.level = DiagnosticStatus.OK
            status.message = 'OK'
            
            if self.actual_rate - self.update_rate < -5:
                status.level = DiagnosticStatus.WARN
                status.message = 'Actual update rate is lower than desired'
                
            diag_msg.status.append(status)
            
            for motor_state in self.current_state.motor_states:
                mid = motor_state.id
                
                status = DiagnosticStatus()
                
                status.name = 'Robotis Dynamixel Motor %d on port %s' % (mid, self.port_namespace)
                status.hardware_id = 'DXL-%d@%s' % (motor_state.id, self.port_namespace)
                status.values.append(KeyValue('Model Name', str(self.motor_static_info[mid]['model'])))
                status.values.append(KeyValue('Firmware Version', str(self.motor_static_info[mid]['firmware'])))
                status.values.append(KeyValue('Return Delay Time', str(self.motor_static_info[mid]['delay'])))
                status.values.append(KeyValue('Minimum Voltage', str(self.motor_static_info[mid]['min_voltage'])))
                status.values.append(KeyValue('Maximum Voltage', str(self.motor_static_info[mid]['max_voltage'])))
                status.values.append(KeyValue('Minimum Position (CW)', str(self.motor_static_info[mid]['min_angle'])))
                status.values.append(KeyValue('Maximum Position (CCW)', str(self.motor_static_info[mid]['max_angle'])))
                
                status.values.append(KeyValue('Goal', str(motor_state.goal)))
                status.values.append(KeyValue('Position', str(motor_state.position)))
                status.values.append(KeyValue('Error', str(motor_state.error)))
                status.values.append(KeyValue('Velocity', str(motor_state.speed)))
                status.values.append(KeyValue('Load', str(motor_state.load)))
                status.values.append(KeyValue('Voltage', str(motor_state.voltage)))
                status.values.append(KeyValue('Temperature', str(motor_state.temperature)))
                status.values.append(KeyValue('Moving', str(motor_state.moving)))
                
                if motor_state.temperature >= self.error_level_temp:
                    status.level = DiagnosticStatus.ERROR
                    status.message = 'OVERHEATING'
                elif motor_state.temperature >= self.warn_level_temp:
                    status.level = DiagnosticStatus.WARN
                    status.message = 'VERY HOT'
                else:
                    status.level = DiagnosticStatus.OK
                    status.message = 'OK'
                    
                diag_msg.status.append(status)
                
            self.diagnostics_pub.publish(diag_msg)
            rate.sleep()

if __name__ == '__main__':
    try:
        serial_proxy = SerialProxy()
        serial_proxy.connect()
        rospy.spin()
        serial_proxy.disconnect()
    except rospy.ROSInterruptException: pass

