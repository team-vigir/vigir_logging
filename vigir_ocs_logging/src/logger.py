#!/usr/bin/env python
#/*********************************************************************
# * Software License Agreement (BSD License)
# *
# *  Copyright (c) 2013-2015, TORC Robotics, LLC ( Team ViGIR )
# *  All rights reserved.
# *
# *  Redistribution and use in source and binary forms, with or without
# *  modification, are permitted provided that the following conditions
# *  are met:
# *
# *   * Redistributions of source code must retain the above copyright
# *     notice, this list of conditions and the following disclaimer.
# *   * Redistributions in binary form must reproduce the above
# *     copyright notice, this list of conditions and the following
# *     disclaimer in the documentation and/or other materials provided
# *     with the distribution.
# *   * Neither the name of Team ViGIR, TORC Robotics, nor the names of its
# *     contributors may be used to endorse or promote products derived
# *     from this software without specific prior written permission.
# *
# *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# *  POSSIBILITY OF SUCH DAMAGE.
# *********************************************************************/
# 
# /* Author: Benjamin Waxler (TORC Robotics) */

import os
import sys
import subprocess
import signal
import rospy
import datetime
import time
from std_msgs.msg import String
from vigir_ocs_msgs.msg import OCSLogging

class App(object):
	def main(self):
		self.listener()

	def __init__(self):
		self.logging = False
		self.toLog = ['/foobar','/test']
		self.folder = ''
		self.bagProcess = ''
		self.callback_data = None
		self.enableBDILogging = False
		self.name = ''
		self.logLocation = '/home/vigir/Experiments'
                self.logger_name = 'ocs_'
		self.pub = ''

	def __del__(self):
		print "Destroying logging node."
		self.killLogging('')

	def createExperiment(self, name, description):
		print "Creating experiment..."+name
		filename = self.logLocation +'/' + name
		if not os.path.exists(filename):
			os.makedirs(filename)
		self.folder = filename
		f = open(self.folder + '/Experiment.txt', 'w')
		f.write('<?xml version="1.0"?>\n')
		f.write(' <Experiment>\n')
		f.write(' <Name>'+name+'</Name>\n')
		self.startTime = datetime.datetime.now()
		f.write(' <StartTime>'+str(self.startTime)+'</StartTime>\n')
		f.write(' <Description>'+description+'</Description>\n')
		f.write(' <TopicsLogged>'+self.combined()+'</TopicsLogged>')
		f.write('</Experiment>')
		f.close()
		self.name = name
		
	def combined(self):
		output = ''
		for x in range(len(self.toLog)):
			output += self.toLog[x] + ' '
		print output
		return output

	def state(self, data):
		print data
		if self.logging:
			temp ="start"
		else:
			temp = "stop"
                self.pub.publish(self.logger_name + temp)

	def startLogging(self):
		print "Starting logs"
		bashCommand = ["/bin/bash", "--norc", "-c"]
                bagCommand = "rosbag record --split --duration=5m -O /"+ self.folder + "/log.bag " + self.combined()
		print bagCommand
		self.bagProcess = subprocess.Popen(bashCommand + [bagCommand], stdout=subprocess.PIPE, preexec_fn=os.setsid)
		self.logging = True
                print self.logger_name + "start"
                self.pub.publish(self.logger_name + "start")
		
	def killLogging(self, results):
		print "Killing logs"
		os.killpg(self.bagProcess.pid, signal.SIGINT)
		self.logging = False
		f = open(self.folder + '/Results.txt', 'w')
		f.write('<?xml version="1.0"?>\n')
		f.write(' <Experiment>\n')
		f.write(' <Name>'+self.name+'</Name>\n')
		f.write(' <StartTime>'+str(self.startTime)+'</StartTime>\n')
		f.write(' <EndTime>'+str(datetime.datetime.now())+'</EndTime>\n')
		f.write(' <Summary>'+results+'</Summary>\n')
		f.write(' <TopicsLogged>'+self.combined()+'</TopicsLogged>')
		f.write('</Experiment>')
		f.close()
		self.folder = ''
                print self.logger_name + "stop"
                self.pub.publish(self.logger_name + "stop")
			
	def listener(self):
		# setup call back for lgging
		print "Starting listener..."
		rospy.init_node('log_listener', anonymous=True)
		
		print "Looking for ros params..."
		print rospy.search_param('logging_location')
		print rospy.search_param('to_log')		
                if rospy.has_param('~logger_name'):
                        print("This logger is named")
                        self.logger_name = rospy.get_param('~logger_name')
                        print self.logger_name
		if rospy.has_param('~to_log'):
			print "logging the following topics..."
			self.toLog = rospy.get_param('~to_log')
			print self.toLog
		else:
			print "Failed to find topics to log."
		if rospy.has_param('~enable_Log_grabbing'):
			self.enableBDILogging = rospy.get_param('~enable_Log_grabbing')
		else:
			print "Not setting up log grabbing"
		if rospy.has_param("~logging_location"):
			print "logging to the following location..."
			self.logLocation = rospy.get_param('~logging_location')
			print self.logLocation
		else:
			print "Using default logging location"
		rospy.Subscriber('/vigir_logging', OCSLogging, self.callback)
		rospy.Subscriber('/vigir_logging_query', String, self.state)
		self.pub = rospy.Publisher('/vigir_logging_responce', String, queue_size=1)
		rospy.spin()
		
	def grabLogs(self, time):
		print "Grabbing robot logs!!"
		bashCommand = ["/bin/bash", "--norc", "-c"]
		if(not(self.folder == '')):
			if not os.path.exists(self.folder):
				os.makedirs(self.folder)
			bagCommand = "python atlas_log_downloader.py 192.168.130.103 /" + self.folder + ' ' + str(time)
		else:
			if not os.path.exists(self.logLocation + '/BDI_Logs'):
				os.makedirs(self.logLocation + '/BDI_Logs')	
			bagCommand = "python atlas_log_downloader.py 192.168.130.103 /" + self.logLocation + '/BDI_Logs ' + str(time)
		print bagCommand
		self.bagProcess = subprocess.Popen(bashCommand + [bagCommand], stdout=subprocess.PIPE, preexec_fn=os.setsid)
		

	def callback(self, data):
		print "Recieved message!"
		self.callback_data = data
		#rospy.loginfo(rospy.get_caller_id() + "I heard %s", self.callback_data.message)
		#print self.callback_data
		if(data.bdiLogTime > 0):			
			if(self.enableBDILogging):
				self.grabLogs(data.bdiLogTime)
			return
		if(not data.no_bags and data.run and self.logging ):
			self.killLogging('')
			self.createExperiment(data.experiment_name, data.description)
			self.startLogging()
		if(not data.no_bags and data.run and (not self.logging)):
			self.createExperiment(data.experiment_name, data.description)
			self.startLogging()
		if(not data.run and self.logging):
			self.killLogging(data.description)

if __name__ == "__main__":
	App().main()
