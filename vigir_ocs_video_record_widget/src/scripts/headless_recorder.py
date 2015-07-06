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
import threading
from std_msgs.msg import String
#import gmtime
from vigir_ocs_msgs.msg import OCSLogging

class App(object):
	def main(self):
		self.listener()

	def __init__(self):
		self.logging = False
		self.scriptsFolder = '/home/vigir/vigir_repo/catkin_ws/src/flor_external_monitoring/vigir_ocs_video_record_widget/src/scripts/'
		self.folder = ''
		self.cameras = ['1','2']
		self.cameraProcesses= ['','','','']
		self.experimentName= ''
		self.logLocation = '/home/testing/Experiments/'
		self.pub = ''

	def combined(self):
		output = ''
		for x in range(len(self.cameras)):
			output += self.cameras[x] + ' '
		rospy.loginfo( output )
		return output

	def state(self, data):
		rospy.loginfo( data )
		if self.logging:
			self.pub.publish("video_start")
		else:
			self.pub.publish("video_stop")

	def listener(self):
		# setup call back for lgging
		rospy.loginfo( "Starting listener...")
		rospy.init_node('video_recorder', anonymous=True)
		if rospy.has_param("~logging_location"):
			rospy.loginfo( "logging to the following location...")
			self.logLocation = rospy.get_param('~logging_location')
			rospy.loginfo( self.logLocation)
		else:
			rospy.loginfo( "Using default logging location")
		rospy.loginfo( "Using cameras:"	)	
		if rospy.has_param('~cameras'):
			self.cameras=ros.get_param('-cameras')
		rospy.loginfo( self.cameras)
		rospy.Subscriber('/vigir_logging', OCSLogging, self.callback)
		rospy.Subscriber('/vigir_logging_query', String, self.state)
		self.pub = rospy.Publisher('/vigir_logging_responce', String, queue_size=1)
		rospy.spin()
		self.pub.publish("video_stop")

	def createExperiment(self, name, description):
		rospy.loginfo( "Creating experiment..."+name)
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
		f.write(' <Cameras>'+self.combined()+'</TopicsLogged>')
		f.write('</Experiment>')
		f.close()
		self.name = name

	def startCameras(self):
		self.setupFolder()
		self.createExperiment(self.experimentName, self.callback_data.description)
		for cam in range(len(self.cameras)):
			bashCommand = ["/bin/bash", "-i", "-c"]
			bagCommand = self.scriptsFolder + "camera" + self.cameras[cam]+ "Headless.sh " + self.logLocation +self.experimentName +" camera" + self.cameras[cam]
			rospy.loginfo( bagCommand )
			self.cameraProcesses[cam] = subprocess.Popen(bashCommand + [bagCommand], stdout=subprocess.PIPE, preexec_fn=os.setsid)
		self.pub.publish("video_start")		
		self.logging = True
			
	def killLogging(self):
		rospy.loginfo( "Killing logs")
		for cam in range(len(self.cameras)):
                        rospy.loginfo(" camera = " + str(self.cameraProcesses[cam]))
			if not self.cameraProcesses[cam] == '':
				os.killpg(self.cameraProcesses[cam].pid, signal.SIGINT)
				self.cameraProcesses[cam] = ''
		os.system("pkill dvgrab")
		self.pub.publish("video_stop")
		self.logging = False
		f = open(self.folder + '/Results.txt', 'w')
		f.write('<?xml version="1.0"?>\n')
		f.write(' <Experiment>\n')
		f.write(' <Name>'+self.name+'</Name>\n')
		f.write(' <StartTime>'+str(self.startTime)+'</StartTime>\n')
		f.write(' <EndTime>'+str(datetime.datetime.now())+'</EndTime>\n')
		f.write(' <Summary>'+self.callback_data.description+'</Summary>\n')
		f.write('</Experiment>')
		f.close()
		time.sleep(2)
		self.folder = ''

	def setupFolder(self):
		if not os.path.exists(self.logLocation + self.callback_data.experiment_name):
			self.experimentName = self.callback_data.experiment_name
		else:
			self.experimentName = time.strftime("%H:%M%S_%Y-%m-%d")
		os.makedirs(self.logLocation + self.experimentName)
		rospy.loginfo( "Folder setup now creating Experiments file "+ self.experimentName)
		
	def callback(self, data):
		rospy.loginfo( "Recieved message!")
		if(not data.no_video):		
			self.callback_data = data
			if(data.run & self.logging ):
				self.killLogging()
				self.startCameras()
			if(data.run & (not self.logging)):
				self.startCameras()
			if(not data.run & self.logging):
				self.killLogging()
		else:
			rospy.loginfo("Message specified no video so ignoring")
	
if __name__ == "__main__":
	App().main()
		
