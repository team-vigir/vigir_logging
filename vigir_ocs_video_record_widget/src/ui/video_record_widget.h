/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013-2015, TORC Robotics, LLC ( Team ViGIR )
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Team ViGIR, TORC Robotics, nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
 
 /* Author: Benjamin Waxler (TORC Robotics) */
 
#ifndef VIDEO_RECORD_WIDGET_H
#define VIDEO_RECORD_WIDGET_H

#include <QWidget>
#include "ros/ros.h"
#include "ros/time.h"
#include <QBasicTimer>
#include <QCheckBox>
#include <vigir_ocs_msgs/OCSLogging.h>

namespace Ui {
class video_record_widget;
}

class video_record_widget : public QWidget
{
    Q_OBJECT

public:
    explicit video_record_widget(QWidget *parent = 0);
    ~video_record_widget();

public Q_SLOTS:
    void on_saveButton_clicked();
    void on_recordButton_clicked();
    void on_previewButton_clicked();
    void on_camera1Box_clicked();
    void on_camera2Box_clicked();
    void on_camera3Box_clicked();
    void on_camera4Box_clicked();
    void on_Ustream_clicked();

protected:
    void timerEvent(QTimerEvent *);
private:
    Ui::video_record_widget *ui;
    void createExperimentFile();
    void startFfmpegRecordingScript(int cameraNum);
    void startRawRecordingScript(int cameraNum);
    void startTestScript(int cameraNum);
    void checkCheckBoxes();
    void timerCallback(const ros::TimerEvent& evnt);
    void transcodeFiles();
    void logMsgRecieved(const vigir_ocs_msgs::OCSLogging::ConstPtr& msg);
    void handleUstreamVideo(bool newState);

    std::string scriptLoc;
    std::string expName;
    std::string experiment_location;

    pid_t recordCam1;
    pid_t recordCam2;
    pid_t recordCam3;
    pid_t recordCam4;
    pid_t ustreamPID;

    bool isRecording;

    time_t endTime;
    time_t startTime;

    QBasicTimer transcodeTimer;
    QBasicTimer timer;

    ros::Subscriber pump_sub;
    ros::Subscriber logging_sub;
    ros::Time       start_time;

    bool testingCameras;
};

#endif // VIDEO_RECORD_WIDGET_H
