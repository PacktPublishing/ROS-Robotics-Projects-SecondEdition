^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package dynamixel_controllers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.1 (2017-01-19)
------------------
* bug fixes for issue `#33 <https://github.com/arebgun/dynamixel_motor/issues/33>`_ and warnnings on queue_size
* Fixed syntax error due to missing parentheses
* add error message when joint names did not match
* added in a queue size as none to remove warning messages
* Readback echo for simple one-wire (TTL) converters
* Support for reading current, setting acceleration, torque control mode (mainly MX series). Available features specified for each model.
* Always release lock on return
* Do not modify a controllers list we are iterating over.
  It's probably a bad idea to modify a list we are looping over as is the case with meta controller dependencies. List copies everywhere!
* Acquire lock before manipulating controllers.
  Make sure that no two threads can manipulate (start, stop or restart) controllers simultaneously.
* Allow negative speed values.
  There was a copy paste error where we were checking to make sure the speed wasn't set to 0 (meaning maximum speed), even though speed has different meaning when in wheel mode (0 is for stop, negative values for counterclockwise rotation and positive - for clockwise rotation). The slave motor should rotate in the opposite direction (assumes motors are mounted back to back with horns facing in opposite directions).
* fixed multi_packet[port] indentation
* added test to fix process_motor_states bug
* got action controller working with dual motor controller, still get an occasional key error with dual motor controller process_motor_states
* patch so action controller works with dual motor controllers
* Merge pull request `#3 <https://github.com/arebgun/dynamixel_motor/issues/3>`_ from arebgun/groovy
  Velocity bug fix
* Fix bug for velocity raw to radsec conversion in all remaining controllers
* Fix bug in velocity conversion
  The factor for the velocity conversion from raw to radsec and the other way
  round is model-specific and does not depend on the maximum motor speed.
  Therefore, a conversion factor is added to the motor data and employed
  for all velocity conversions. The set motor velocity and the motor velocity
  in the servo state does now correspond with the real servo speed in
  radians per second.
* Contributors: Andreas Wachaja, Antons Rebguns, Kei Okada, Nicolas, Nicolas Alt, Richard-Lab, Russell Toris, Tyler Slabinski, Yam Geva, richard-virtualros

0.4.0 (2013-07-26)
------------------
* stack is now catkin compatible
