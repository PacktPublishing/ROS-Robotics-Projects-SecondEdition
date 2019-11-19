#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2010-2011, Cody Jorgensen, Antons Rebguns.
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


__author__ = 'Cody Jorgensen, Antons Rebguns'
__copyright__ = 'Copyright (c) 2010-2011 Cody Jorgensen, Antons Rebguns'

__license__ = 'BSD'
__maintainer__ = 'Antons Rebguns'
__email__ = 'anton@email.arizona.edu'


import sys
from optparse import OptionParser

import roslib
roslib.load_manifest('dynamixel_driver')

from dynamixel_driver import dynamixel_io
from dynamixel_driver.dynamixel_const import *

def print_data(values):
    ''' Takes a dictionary with all the motor values and does a formatted print.
    '''
    if values['freespin']:
        print '''\
    Motor %(id)d is connected:
        Freespin: True
        Model ------------------- %(model)s
        Current Speed ----------- %(speed)d
        Current Temperature ----- %(temperature)d%(degree_symbol)sC
        Current Voltage --------- %(voltage).1fv
        Current Load ------------ %(load)d
        Moving ------------------ %(moving)s
''' %values
    else:
        print '''\
    Motor %(id)d is connected:
        Freespin: False
        Model ------------------- %(model)s
        Min Angle --------------- %(min)d
        Max Angle --------------- %(max)d
        Current Position -------- %(position)d
        Current Speed ----------- %(speed)d
        Current Temperature ----- %(temperature)d%(degree_symbol)sC
        Current Voltage --------- %(voltage).1fv
        Current Load ------------ %(load)d
        Moving ------------------ %(moving)s
''' %values

if __name__ == '__main__':
    usage_msg = 'Usage: %prog [options] IDs'
    desc_msg = 'Prints the current status of specified Dynamixel servo motors.'
    epi_msg = 'Example: %s --port=/dev/ttyUSB1 --baud=57600 1 2 3 4 5' % sys.argv[0]
    
    parser = OptionParser(usage=usage_msg, description=desc_msg, epilog=epi_msg)
    parser.add_option('-p', '--port', metavar='PORT', default='/dev/ttyUSB0',
                      help='motors of specified controllers are connected to PORT [default: %default]')
    parser.add_option('-b', '--baud', metavar='BAUD', type="int", default=1000000,
                      help='connection to serial port will be established at BAUD bps [default: %default]')
                      
    (options, args) = parser.parse_args(sys.argv)
    
    if len(args) < 2:
        parser.print_help()
        exit(1)
        
    port = options.port
    baudrate = options.baud
    motor_ids = args[1:]
    
    try:
        dxl_io = dynamixel_io.DynamixelIO(port, baudrate)
    except dynamixel_io.SerialOpenError, soe:
        print 'ERROR:', soe
    else:
        responses = 0
        print 'Pinging motors:'
        for motor_id in motor_ids:
            motor_id = int(motor_id)
            print '%d ...' % motor_id,
            p = dxl_io.ping(motor_id)
            if p:
                responses += 1
                values = dxl_io.get_feedback(motor_id)
                angles = dxl_io.get_angle_limits(motor_id)
                model = dxl_io.get_model_number(motor_id)
                firmware = dxl_io.get_firmware_version(motor_id)
                values['model'] = '%s (firmware version: %d)' % (DXL_MODEL_TO_PARAMS[model]['name'], firmware)
                values['degree_symbol'] = u"\u00B0"
                values['min'] = angles['min']
                values['max'] = angles['max']
                values['voltage'] = values['voltage']
                values['moving'] = str(values['moving'])
                print 'done'
                if angles['max'] == 0 and angles['min'] == 0:
                    values['freespin'] = True
                else:
                    values['freespin'] = False
                print_data(values)
            else:
                print 'error'
        if responses == 0:
            print 'ERROR: None of the specified motors responded. Make sure to specify the correct baudrate.'

