#!/usr/bin/env python
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

__license__ = 'BSD'
__maintainer__ = 'Antons Rebguns'
__email__ = 'anton@email.arizona.edu'


import sys
from optparse import OptionParser

import roslib
roslib.load_manifest('dynamixel_driver')

from dynamixel_driver import dynamixel_io

if __name__ == '__main__':
    usage_msg = 'Usage: %prog [options] MOTOR_IDs'
    desc_msg = 'Sets various configuration options of specified Dynamixel servo motor.'
    epi_msg = 'Example: %s --port=/dev/ttyUSB1 --baud=57600 --baud-rate=1 --return-delay=1 5 9 23' % sys.argv[0]
    
    parser = OptionParser(usage=usage_msg, description=desc_msg, epilog=epi_msg)
    parser.add_option('-p', '--port', metavar='PORT', default='/dev/ttyUSB0',
                      help='motors of specified controllers are connected to PORT [default: %default]')
    parser.add_option('-b', '--baud', metavar='BAUD', type='int', default=1000000,
                      help='connection to serial port will be established at BAUD bps [default: %default]')
    parser.add_option('-r', '--baud-rate', type='int', metavar='RATE', dest='baud_rate',
                      help='set servo motor communication speed')
    parser.add_option('-d', '--return-delay', type='int', metavar='DELAY', dest='return_delay',
                      help='set servo motor return packet delay time')
    parser.add_option('--cw-angle-limit', type='int', metavar='CW_ANGLE', dest='cw_angle_limit',
                      help='set servo motor CW angle limit')
    parser.add_option('--ccw-angle-limit', type='int', metavar='CCW_ANGLE', dest='ccw_angle_limit',
                      help='set servo motor CCW angle limit')
    parser.add_option('--min-voltage-limit', type='int', metavar='MIN_VOLTAGE', dest='min_voltage_limit',
                      help='set servo motor minimum voltage limit')
    parser.add_option('--max-voltage-limit', type='int', metavar='MAX_VOLTAGE', dest='max_voltage_limit',
                      help='set servo motor maximum voltage limit')
                      
    (options, args) = parser.parse_args(sys.argv)
    print options
    
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
        for motor_id in motor_ids:
            motor_id = int(motor_id)
            print 'Configuring Dynamixel motor with ID %d' % motor_id
            if dxl_io.ping(motor_id):
                # check if baud rate needs to be changed
                if options.baud_rate:
                    valid_rates = (1,3,4,7,9,16,34,103,207,250,251,252)
                    
                    if options.baud_rate not in valid_rates:
                        print 'Requested baud rate is invalid, please use one of the following: %s' % str(valid_rates)
                        
                    if options.baud_rate <= 207:
                        print 'Setting baud rate to %d bps' % int(2000000.0/(options.baud_rate + 1))
                    elif options.baud_rate == 250:
                        print 'Setting baud rate to %d bps' % 2250000
                    elif options.baud_rate == 251:
                        print 'Setting baud rate to %d bps' % 2500000
                    elif options.baud_rate == 252:
                        print 'Setting baud rate to %d bps' % 3000000
                        
                    dxl_io.set_baud_rate(motor_id, options.baud_rate)
                    
                # check if return delay time needs to be changed
                if options.return_delay is not None:
                    if options.return_delay < 0 or options.return_delay > 254:
                        print 'Requested return delay time is out of valid range (0 - 254)'
                    else:
                        print 'Setting return delay time to %d us' % (options.return_delay * 2)
                        dxl_io.set_return_delay_time(motor_id, options.return_delay)
                        
                # check if CW angle limit needs to be changed
                if options.cw_angle_limit is not None:
                    print 'Setting CW angle limit to %d' % options.cw_angle_limit
                    dxl_io.set_angle_limit_cw(motor_id, options.cw_angle_limit)
                    
                # check if CCW angle limit needs to be changed
                if options.ccw_angle_limit is not None:
                    print 'Setting CCW angle limit to %d' % options.ccw_angle_limit
                    dxl_io.set_angle_limit_ccw(motor_id, options.ccw_angle_limit)
                    
                # check if minimum voltage limit needs to be changed
                if options.min_voltage_limit:
                    print 'Setting minimum voltage limit to %d' % options.min_voltage_limit
                    dxl_io.set_voltage_limit_min(motor_id, options.min_voltage_limit)
                    
                # check if maximum voltage limit needs to be changed
                if options.max_voltage_limit:
                    print 'Setting maximum voltage limit to %d' % options.max_voltage_limit
                    dxl_io.set_voltage_limit_max(motor_id, options.max_voltage_limit)
                    
                print 'done'
            else:
                print 'Unable to connect to Dynamixel motor with ID %d' % motor_id
