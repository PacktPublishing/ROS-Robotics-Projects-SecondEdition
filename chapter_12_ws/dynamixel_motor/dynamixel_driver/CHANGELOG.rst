^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package dynamixel_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.1 (2017-01-19)
------------------
* Don't set return delay time if value is invalid
* Fix typo valie -> valid
* Add EX-106, rename EX-106+ accordingly (Fixes `#60 <https://github.com/arebgun/dynamixel_motor/issues/60>`_)
* Merge pull request `#49 <https://github.com/arebgun/dynamixel_motor/issues/49>`_ from anuragmakineni/master
  remove Serial set/get functions from dynamixel_io
* Update dynamixel_io.py
  In set_p_gain, set_i_gain, set_d_gain, small spelling mistake. Instead of 'slope', it should be p/i/d_gain, since slope is not defined for the given function.
* remove deprecated functions from dynamixel_io
* bug fixes for issue `#33 <https://github.com/arebgun/dynamixel_motor/issues/33>`_ and warnnings on queue_size
* Adds couple of methods for LED status fetching and changing.
* Catch occurancies when error_code is parsed as float
* added in a queue size as none to remove warning messages
* Readback echo for simple one-wire (TTL) converters
* Support for reading current, setting acceleration, torque control mode (mainly MX series). Available features specified for each model.
* New model MX-12W
* fix typo ;; min -> max
* Fix bug in velocity conversion
  The factor for the velocity conversion from raw to radsec and the other way
  round is model-specific and does not depend on the maximum motor speed.
  Therefore, a conversion factor is added to the motor data and employed
  for all velocity conversions. The set motor velocity and the motor velocity
  in the servo state does now correspond with the real servo speed in
  radians per second.
* Contributors: Andreas Wachaja, Antons Rebguns, Anurag Makineni, Gabrielius Mickevicius, Nicolas Alt, Russell Toris, Stefan Kohlbrecher, Yam Geva, Zilvinas, nozawa, parijat10, pazeshun

0.4.0 (2013-07-26)
------------------
* stack is now catkin compatible
