/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013-2015, Team ViGIR ( TORC Robotics LLC, TU Darmstadt, Virginia Tech, Oregon State University, Cornell University, and Leibniz University Hanover )
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

#include "video_record_widget.h"
#include "ui_video_record_widget.h"
#include <sys/stat.h>
#include "boost/filesystem.hpp"
#include "boost/lexical_cast.hpp"
#include "QLineEdit"
#include <time.h>
#include <unistd.h>
#include <signal.h>
#include <sys/types.h>
#include <ctime>
#include "QMessageBox"
#include "fstream"
#include "QRegExp"
#include "QPlainTextEdit"
#include "QInputDialog"

video_record_widget::video_record_widget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::video_record_widget)
{
    ui->setupUi(this);
    ros::NodeHandle nh("~");
    ui->recordButton->setEnabled(false);
    isRecording = false;
    testingCameras = false;
    scriptLoc = "/home/vigir/vigir_repo/catkin_ws/src/flor_external_monitoring/vigir_ocs_video_record_widget/src/scripts/";
    experiment_location = "/home/vigir/Experiments/";
    recordCam1 = 0;
    recordCam2 = 0;
    recordCam3 = 0;
    recordCam4 = 0;
    ustreamPID = 0;

    expName = "";

    timer.start(33,this);
    logging_sub = nh.subscribe<vigir_ocs_msgs::OCSLogging>("/vigir_logging", 1, &video_record_widget::logMsgRecieved, this);
    if(nh.hasParam("experiment_location"))
    {
        nh.getParam("experiment_location",experiment_location);
        ROS_WARN("Experiment Location set to %s", experiment_location.c_str());
        if((experiment_location.substr(experiment_location.size()-1,1).compare("/") != 0))
            experiment_location.append("/");
    }
    if(nh.hasParam("scripts_location"))
    {
        nh.getParam("scripts_location",scriptLoc);
        ROS_WARN("Scripts Location set to %s", scriptLoc.c_str());
        if((scriptLoc.substr(scriptLoc.size()-1,1).compare("/") != 0))
            scriptLoc.append("/");
    }
    checkCheckBoxes();
}

void video_record_widget::timerEvent(QTimerEvent *event)
{
    ros::spinOnce();
}

void video_record_widget::logMsgRecieved(const vigir_ocs_msgs::OCSLogging::ConstPtr& msg)
{
    if(!msg->run && isRecording)
    {
	std::cout << "Closing old experiment..." << std::endl;
        on_recordButton_clicked();
        if(msg->experiment_name.c_str() != ui->experimentName->text().toStdString())
            std::cout << "New name on experiment. Closing experiment video." << std::endl;
        on_saveButton_clicked();
    }
        //|| (isRecording && msg->experiment_name.c_str() != ui->experimentName->text().toStdString()))
   if(isRecording && msg->experiment_name.c_str() != ui->experimentName->text().toStdString() && msg->run)
   {
        on_recordButton_clicked();
        on_saveButton_clicked();
        ui->experimentName->setText(QString::fromStdString(msg->experiment_name.c_str()));
        ui->descriptionTxtBox->setPlainText(QString::fromStdString(msg->description.c_str()));
        ui->Ustream->setChecked(msg->ustream);
        ui->camera1Box->setChecked(ui->camera1Box->isEnabled());
        ui->camera2Box->setChecked(ui->camera2Box->isEnabled());
        ui->camera3Box->setChecked(ui->camera3Box->isEnabled());
        ui->camera4Box->setChecked(ui->camera4Box->isEnabled());
        on_saveButton_clicked();
	sleep(2);
        on_recordButton_clicked();
    }
   if(!isRecording && msg->run && msg->experiment_name.c_str() != "")
   {
       ui->experimentName->setText(QString::fromStdString(msg->experiment_name.c_str()));
       ui->descriptionTxtBox->setPlainText(QString::fromStdString(msg->description.c_str()));
       ui->Ustream->setChecked(msg->ustream);
       ui->camera1Box->setChecked(ui->camera1Box->isEnabled());
       ui->camera2Box->setChecked(ui->camera2Box->isEnabled());
       ui->camera3Box->setChecked(ui->camera3Box->isEnabled());
       ui->camera4Box->setChecked(ui->camera4Box->isEnabled());
       on_saveButton_clicked();
       on_recordButton_clicked();
   }
}

video_record_widget::~video_record_widget()
{
    delete ui;
}

void video_record_widget::transcodeFiles()
{
    pid_t temp;
    std::string timeString ="";
    int hours = std::floor(start_time.sec/3600);
    int minutes = std::floor(start_time.sec/60) - hours;
    int seconds = start_time.sec/3600 - 60*minutes - 3600*hours;
    timeString += ""+boost::lexical_cast<std::string>(hours)+"\\:"+boost::lexical_cast<std::string>(minutes)+"\\:"+boost::lexical_cast<std::string>(seconds)+"\\;02";
    uint32_t result;
    temp = fork();
    if(temp >=0)
    {
        if(temp == 0)
        {
            std::string scriptCall = scriptLoc+"transcodeFile.sh "+experiment_location+ui->experimentName->text().toStdString()+"/ ";
            //if(ui->keepOrigionals->isChecked())
            //    scriptCall += "false";
            //else
            scriptCall += "false";
            scriptCall += " " +timeString;
            result = system(scriptCall.c_str());
            exit(result);
        }
        else
            std::cout << "Started transcoding the old files..." << std::endl;
    }
}

void video_record_widget::on_saveButton_clicked()
{
    if(ui->saveButton->text() == "Save")
    {
        QRegExp rx("(\s\\\\)");
        expName = (ui->experimentName->text().replace(rx,tr("_"))).toStdString();
        std::cout << "Exp name is " << expName << std::endl;
        boost::filesystem::path folder (std::string(experiment_location+expName));
        if(boost::filesystem::exists(folder))
        {
            std::cout << "Folder already exists " << folder.c_str() << std::endl;
            QMessageBox msg;
            msg.setWindowTitle(QString::fromStdString("Video Recorder Error"));
            msg.setInformativeText(QString::fromStdString("Cannot have two experiments with the same name.\n\nPlease rename the experiment to continue."));
            msg.exec();
        }
        else
        {

            if(boost::filesystem::create_directory(folder))
                std::cout<< "Created new folder at " << folder.c_str() << std::endl;
            ui->recordButton->setEnabled(true);
            ui->recordButton->setStyleSheet("color: rgb(0,255,0)");
        ui->experimentName->setEnabled(false);
        }
    }
    else
    {
        std::string dir = experiment_location+expName;
        std::string name = dir+"/experiment.txt";
        std::string summary = dir+"/summary.txt";
        FILE * summaryFile;
        summaryFile = fopen(summary.c_str(),"w+");
        if(summaryFile == NULL)
        {
            std::cout << "Failed to create Summary File." << std::endl;
            return;
        }
        else
        {
            std::ifstream experimentFile(name.c_str());
            std::string line;
            for(int i=0;i<4;i++)
            {
                std::getline(experimentFile,line);
                fprintf(summaryFile,"%s\n",line.c_str());
            }
            struct tm * timeinfo;
            time(&endTime);
            timeinfo = localtime( &endTime);
            fprintf(summaryFile," <EndTime>%d-%d-%d_%d:%d:%d</EndTime>\n",timeinfo->tm_mon,timeinfo->tm_mday,timeinfo->tm_year+1900,timeinfo->tm_hour,timeinfo->tm_min,timeinfo->tm_sec);
            std::cout << "Length of time recorded was " << difftime(endTime,startTime) << " seconds"<< std::endl;
            while(std::getline(experimentFile,line))
            {
                fprintf(summaryFile,"%s\n",line.c_str());
                if(line == " </Cameras>")
                    break;
            }
            fprintf(summaryFile," <Results>%s</Results>/n",ui->results->document()->toPlainText().toStdString().c_str());
            fprintf(summaryFile,"</Experiment>");
            ui->camera1Box->setEnabled(true);
            ui->camera2Box->setEnabled(true);
            ui->camera3Box->setEnabled(true);
            ui->camera4Box->setEnabled(true);
            ui->saveButton->setText(QString::fromStdString("Save"));
            ui->experimentName->setEnabled(true);
            ui->results->document()->clear();
        }
    }
}

void video_record_widget::on_previewButton_clicked()
{
    if(!testingCameras)
    {
        if(ui->camera1Box->isChecked())
            startTestScript(1);
        if(ui->camera2Box->isChecked())
            startTestScript(2);
        if(ui->camera3Box->isChecked())
            startTestScript(3);
        if(ui->camera4Box->isChecked())
            startTestScript(4);
        testingCameras = true;
        ui->previewButton->setText(QString::fromStdString("Stop Preview"));
        ui->recordButton->setEnabled(false);
    }
    else
    {
        if(recordCam1 != 0)
            recordCam1 = kill(-recordCam1,SIGTERM);
        if(recordCam2 != 0)
            recordCam2 = kill(-recordCam2,SIGTERM);
        if(recordCam3 != 0)
            recordCam3 = kill(-recordCam3,SIGTERM);
        if(recordCam4 != 0)
            recordCam4 = kill(-recordCam4,SIGTERM);
        ui->previewButton->setText(QString::fromStdString("Preview Video"));
        testingCameras = false;
        ui->recordButton->setEnabled(true);
    }
}

void video_record_widget::checkCheckBoxes()
{
    std::string sysCall = scriptLoc+"camerasAvailable.sh";
    uint32_t foo = 0;
    foo = boost::lexical_cast<int>(system(sysCall.c_str()));
    foo = foo >> 8;
    std::cout << "Exit code = " << foo << std::endl;
    int test1 = foo%2;
    std::cout<< "Test for box 1 = " << test1 << std::endl;
    int test2 = foo >> 1 << 31;
    std::cout << "Test for box 2 = " << test2 << std::endl;
    int test3 = foo >> 2 << 31;
    std::cout << "Test for box 3 = " << test3 << std::endl;
    int test4 = foo >> 3;
    std::cout << "Test for box 4 = " << test4 << std::endl;
    if(!test1)
        ui->camera1Box->setChecked(false);
    ui->camera1Box->setEnabled(test1);
    if(!test2)
        ui->camera2Box->setChecked(false);
    ui->camera2Box->setEnabled(test2);
    if(!test3)
        ui->camera3Box->setChecked(false);
    ui->camera3Box->setEnabled(test3);
    if(!test4)
        ui->camera4Box->setChecked(false);
    ui->camera4Box->setEnabled(test4);

}

void video_record_widget::handleUstreamVideo(bool newState)
{
    if(ustreamPID !=0 && newState)
        std::cout << "Tried to turn on Ustream video when it is already running!" << std::endl;
    else if(newState)
    {
        std::string Loc = scriptLoc+"desktopStreaming.sh";
        pid_t temp;
        temp = fork();
        if(temp >=0)
        {
            if( temp == 0)
            {
                temp = setsid();
                int ret_val = system(Loc.c_str());
            }
            else
            {
                std::cout<< "Started streaming child" << std::endl;
                ustreamPID = temp;
            }
        }
        else
            std::cout << "Failed to make child process to start script in.." <<std::endl;
    }
    else
    {
       ustreamPID = kill(-ustreamPID,SIGTERM);
       std::cout << "Killed streaming child." << std::endl;
    }
}

void video_record_widget::on_recordButton_clicked()
{
    if(!isRecording)
    {
        start_time = ros::Time::now();
        ui->saveButton->setEnabled(false);
        ui->recordButton->setStyleSheet("color: rgb(255,0,0)");
        ui->recordButton->setText("Stop Recording");
        createExperimentFile();
        ui->camera1Box->setEnabled(false);
        ui->camera2Box->setEnabled(false);
        ui->camera3Box->setEnabled(false);
        ui->camera4Box->setEnabled(false);
        ui->previewButton->setEnabled(false);
        isRecording = true;
        if(ui->camera1Box->isChecked())
            startRawRecordingScript(1);
        if(ui->camera2Box->isChecked())
            startRawRecordingScript(2);
        if(ui->camera3Box->isChecked())
            startRawRecordingScript(3);
        if(ui->camera4Box->isChecked())
            startRawRecordingScript(4);
        if(ui->Ustream->isChecked())
            handleUstreamVideo(true);
        ui->Ustream->setEnabled(false);
    }
    else
    {
        std::cout << "Killing camera recording processes...." << std::endl;
        if(recordCam1 != 0)
            recordCam1 = kill(-recordCam1,SIGTERM);
        if(recordCam2 != 0)
            recordCam2 = kill(-recordCam2,SIGTERM);
        if(recordCam3 != 0)
            recordCam3 = kill(-recordCam3,SIGTERM);
        if(recordCam4 != 0)
            recordCam4 = kill(-recordCam4,SIGTERM);
        if(ui->Ustream->isChecked())
            handleUstreamVideo(false);
        isRecording = false;
        ui->Ustream->setEnabled(true);
        ui->saveButton->setEnabled(true);
        ui->previewButton->setEnabled(true);
        ui->saveButton->setText(QString::fromStdString("Save Summary"));
        ui->recordButton->setText(QString::fromStdString("Record"));
        ui->recordButton->setStyleSheet("color: rgb(0,0,0)");
        ui->recordButton->setEnabled(false);
        checkCheckBoxes();
        //if(ui->transcode->isChecked())
        transcodeFiles();
    }
}

void video_record_widget::createExperimentFile()
{
    std::cout << "Creating experiment.txt file....." << std::endl;
    FILE * experimentTxt;
    std::string name = experiment_location+ui->experimentName->text().toStdString()+"/experiment.txt";
    experimentTxt = fopen(name.c_str(),"w+");
    if(experimentTxt == NULL)
    {
        std::cout << "Failed to create experiments.txt file." << std::endl;
        return;
    }
    fprintf(experimentTxt,"<?xml version=\"1.0\"?>\n");
    fprintf(experimentTxt,"<Experiment>\n <Name>%s</Name>\n",ui->experimentName->text().toStdString().c_str());

    struct tm * timeinfo;
    time(&startTime);
    timeinfo = localtime( &startTime);
    fprintf(experimentTxt," <StartTime>%d-%d-%d_%d:%d:%d</StartTime>\n",timeinfo->tm_mon,timeinfo->tm_mday,timeinfo->tm_year+1900,timeinfo->tm_hour,timeinfo->tm_min,timeinfo->tm_sec);

    fprintf(experimentTxt," <Description>%s</Description>\n",ui->descriptionTxtBox->document()->toPlainText().toStdString().c_str());

    fprintf(experimentTxt," <Cameras>\n");
    if(ui->camera1Box->isChecked())
        fprintf(experimentTxt,"  <CameraName>Camera 1</CameraName>\n");
    if(ui->camera2Box->isChecked())
        fprintf(experimentTxt,"  <CameraName>Camera 2</CameraName>\n");
    if(ui->camera3Box->isChecked())
        fprintf(experimentTxt,"  <CameraName>Camera 3</CameraName>\n");
    if(ui->camera4Box->isChecked())
        fprintf(experimentTxt,"  <CameraName>Camera 4</CameraName>\n");
    fprintf(experimentTxt," </Cameras>\n");
    fprintf(experimentTxt,"</Experiment>");
    fclose(experimentTxt);
    std::cout << "Finished setting up Experiments.txt now starting camera scripts..." << std::endl;
}

void video_record_widget::startFfmpegRecordingScript(int cameraNum)
{
    std::cout << "Start ffmpeg record script called on camera " << cameraNum <<std::endl;
    std::ifstream scriptTest;
    std::string Loc = scriptLoc+"camera"+boost::lexical_cast<std::string>(cameraNum)+"Ffmpeg.sh";
    std::string experimentDirectory = experiment_location+expName;
    scriptTest.open(Loc.c_str());
    if(!scriptTest.is_open())
    {
        std::cout << "Could not find ffmpeg recording script for the camera " << cameraNum << " at " << scriptLoc << std::endl;
        return;
    }
    scriptTest.close();
    std::cout<< "Start script found." << std::endl;
    pid_t temp;
    temp = fork();
    int return_val;
    if(temp >=0)
    {
        if( temp == 0)
        {
            temp = setsid();
            std::string scriptCall = Loc+" "+experimentDirectory+" camera"+boost::lexical_cast<std::string>(cameraNum);
            std::cout << scriptCall << std::endl;
            return_val = system(scriptCall.c_str());
        }
        else
            std::cout<< "Started recording script for Camera " << cameraNum;
            return_val = system("sleep 0.5");
        switch(cameraNum)
        {
        case 1:
            recordCam1 = temp;
            break;
        case 2:
            recordCam2 = temp;
            break;
        case 3:
            recordCam3 = temp;
            break;
        case 4:
            recordCam4 = temp;
        default:
            break;
        }
    }
    else
        std::cout << "Failed to make child process to start script in.." <<std::endl;
}

void video_record_widget::startRawRecordingScript(int cameraNum)
{
    std::cout << "Start Raw record script called on camera " << cameraNum <<std::endl;
    std::ifstream scriptTest;
    std::string Loc = scriptLoc+"camera"+boost::lexical_cast<std::string>(cameraNum)+"Raw.sh";
    std::string experimentDirectory = experiment_location+expName;
    scriptTest.open(Loc.c_str());
    if(!scriptTest.is_open())
    {
        std::cout << "Could not find Raw recording script for the camera " << cameraNum << " at " << scriptLoc << std::endl;
        return;
    }
    scriptTest.close();
    std::cout<< "Start script found." << std::endl;
    pid_t temp;
    int return_val;
    temp = fork();
    if(temp >=0)
    {
        if( temp == 0)
        {
            temp = setsid();
            std::string scriptCall = Loc+" "+experimentDirectory+" camera"+boost::lexical_cast<std::string>(cameraNum);
            std::cout << scriptCall << std::endl;
            try{
                int i = system(scriptCall.c_str());
                std::cout << "Returned " << i <<std::endl;         
            }
            catch(...)
            {
            }
        }
        else
            std::cout<< "Started recording script for Camera " << cameraNum;
            return_val = system("sleep 0.5");
            switch(cameraNum)
            {
            case 1:
                recordCam1 = temp;
                return_val = system("wmctrl -r 'playdv' -T 'Camera 1'");
                return_val = system("wmctrl -r 'Camera 1' -e 0,20,20,720,480");
                break;
            case 2:
                recordCam2 = temp;
                return_val = system("wmctrl -r 'playdv' -T 'Camera 2'");
                return_val = system("wmctrl -r 'Camera 2' -e 0,800,20,720,480");
                break;
            case 3:
                recordCam3 = temp;
                return_val = system("wmctrl -r 'playdv' -T 'Camera 3'");
                return_val = system("wmctrl -r 'Camera 3' -e 0,20,540,720,480");
                break;
            case 4:
                recordCam4 = temp;
                return_val = system("wmctrl -r 'playdv' -T 'Camera 4'");
                return_val = system("wmctrl -r 'Camera 4' -e 0,800,540,720,480");
            default:
                break;
            }
    }
    else
        std::cout << "Failed to make child process to start script in.." <<std::endl;
}

void video_record_widget::startTestScript(int cameraNum)
{
    std::cout << "Start Raw record script called on camera " << cameraNum <<std::endl;
    std::ifstream scriptTest;
    std::string Loc = scriptLoc+"camera"+boost::lexical_cast<std::string>(cameraNum)+"Test.sh";
    scriptTest.open(Loc.c_str());
    if(!scriptTest.is_open())
    {
        std::cout << "Could not find Test script for the camera " << cameraNum << " at " << scriptLoc << std::endl;
        return;
    }
    scriptTest.close();
    std::cout<< "Start script found." << std::endl;
    pid_t temp;
    int return_val;
    temp = fork();
    if(temp >=0)
    {
        if( temp == 0)
        {
            temp = setsid();
            try{
                int i = system(Loc.c_str());
                std::cout << "Returned " << i <<std::endl;
            }
            catch(...)
            {
            }
        }
        else
            std::cout<< "Started test script for Camera " << cameraNum;
            return_val = system("sleep 0.5");
        switch(cameraNum)
        {
        case 1:
            recordCam1 = temp;
            return_val = system("wmctrl -r 'playdv' -T 'Camera 1'");
            return_val = system("wmctrl -r 'Camera 1' -e 0,20,20,720,480");
            break;
        case 2:
            recordCam2 = temp;
            return_val = system("wmctrl -r 'playdv' -T 'Camera 2'");
            return_val = system("wmctrl -r 'Camera 2' -e 0,800,20,720,480");
            break;
        case 3:
            recordCam3 = temp;
            return_val = system("wmctrl -r 'playdv' -T 'Camera 3'");
            return_val = system("wmctrl -r 'Camera 3' -e 0,20,540,720,480");
            break;
        case 4:
            recordCam4 = temp;
            return_val = system("wmctrl -r 'playdv' -T 'Camera 4'");
            return_val = system("wmctrl -r 'Camera 4' -e 0,800,540,720,480");
        default:
            break;
        }
    }
    else
        std::cout << "Failed to make child process to start script in.." <<std::endl;
}

void video_record_widget::on_camera1Box_clicked()
{
    checkCheckBoxes();
}

void video_record_widget::on_camera2Box_clicked()
{
    checkCheckBoxes();
}
void video_record_widget::on_camera3Box_clicked()
{
    checkCheckBoxes();
}
void video_record_widget::on_camera4Box_clicked()
{
    checkCheckBoxes();
}

void video_record_widget::on_Ustream_clicked()
{
    handleUstreamVideo(ui->Ustream->isChecked());
}
