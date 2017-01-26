#!/usr/bin/env python

import sys
import os
import unittest
import subprocess

import rospy
import rosunit

## A sample python unit test
class TestUrdf(unittest.TestCase):

	def test_correct_format(self):
		print sys.argv
		print len(sys.argv)

		if len(sys.argv) < 2:
			self.fail("no urdf file given, usage: " + os.path.basename(sys.argv[0]) + " file.urdf.xacro. \ninput parameters are: " + str(sys.argv))

		file_to_test = sys.argv[1]
		print "testing " + file_to_test

		# check if file exists
		if not os.path.exists(file_to_test):
			self.fail('file "' + file_to_test + '" not found')

		# check if xacro can be converted
		p = subprocess.Popen("`rospack find xacro`/xacro --inorder " + file_to_test + " > /tmp/test.urdf", shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
		if p.returncode != 0 and p.returncode != None:
			self.fail("cannot convert xacro. file: " + file_to_test + "\n" + p.stderr.read())

		# check if urdf is correct
		p = subprocess.Popen("check_urdf /tmp/test.urdf", shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
		if p.returncode != 0 and p.returncode != None:
			self.fail("urdf not correct. file: " + file_to_test + "\n" + p.stderr.read())

if __name__ == '__main__':
	rosunit.unitrun('cob_description', 'test_urdf', TestUrdf)
