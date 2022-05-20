#!/usr/bin/env python
PKG = 'test_foo'
import roslib; roslib.load_manifest(PKG)  # This line is not needed with Catkin.
   
import sys
import unittest
   
    ## A sample python unit test
class TestWheelChairBot(unittest.TestCase):
		## test 1 == 1
	def test_one_equals_one(self): # only functions with 'test_'-prefix will be run!
		self.assertEquals(1, 1, "1!=1")
		
	
   
	if __name__ == '__main__':
		import rostest
		rostest.rosrun(PKG, 'test_wheelchair bot', TestWheelChairBot)
