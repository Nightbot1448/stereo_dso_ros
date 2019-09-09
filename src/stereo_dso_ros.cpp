/**
* This file is part of DSO.
*
* Copyright 2016 Technical University of Munich and Intel.
* Developed by Jakob Engel <engelj at in dot tum dot de>,
* for more information see <http://vision.in.tum.de/dso>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* DSO is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* DSO is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with DSO. If not, see <http://www.gnu.org/licenses/>.
*/




#include <clocale>
#include <csignal>
#include <cstdlib>
#include <cstdio>
#include <unistd.h>

#include "util/settings.h"
#include "FullSystem/FullSystem.h"
#include "util/Undistort.h"
#include "IOWrapper/Pangolin/PangolinDSOViewer.h"
#include "IOWrapper/OutputWrapper/SampleOutputWrapper.h"


#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "cv_bridge/cv_bridge.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "util/DatasetReader.h"


std::string calib;
std::string vignetteFile;
std::string gammaFile;
bool useSampleOutput = false;
bool preload = false;
int mode = 0;

float playbackSpeed=0;	// 0 for linearize (play as fast as possible, while sequentializing tracking & mapping). otherwise, factor on timestamps.

using namespace dso;

void settingsDefault(int preset)
{
    printf("\n=============== PRESET Settings: ===============\n");
    if(preset == 0 || preset == 1)
    {
        printf("DEFAULT settings:\n"
                "- %s real-time enforcing\n"
                "- 2000 active points\n"
                "- 5-7 active frames\n"
                "- 1-6 LM iteration each KF\n"
                "- original image resolution\n", preset==0 ? "no " : "1x");

        playbackSpeed = (preset==0 ? 0 : 1);
        preload = preset==1;

        setting_desiredImmatureDensity = 1500;    //original 1500. set higher
        setting_desiredPointDensity = 2000;       //original 2000
        setting_minFrames = 5;
        setting_maxFrames = 7;
        setting_maxOptIterations=6;
        setting_minOptIterations=1;

        /*林辉灿注释掉
        setting_kfGlobalWeight=0.3;   // original is 1.0. 0.3 is a balance between speed and accuracy. if tracking lost, set this para higher
        setting_maxShiftWeightT= 0.04f * (640 + 128);   // original is 0.04f * (640+480); this para is depend on the crop size.
        setting_maxShiftWeightR= 0.04f * (640 + 128);   // original is 0.0f * (640+480);
        setting_maxShiftWeightRT= 0.02f * (640 + 128);  // original is 0.02f * (640+480);
        */
        /*林辉灿重设参数 KITTI*
        setting_kfGlobalWeight=1.0;   // original is 1.0. 0.3 is a balance between speed and accuracy. if tracking lost, set this para higher
        setting_maxShiftWeightT= 0.04f * (960 + 320);   // original is 0.04f * (640+480); this para is depend on the crop size.
        setting_maxShiftWeightR= 0.04f * (960 + 320);    // original is 0.0f * (640+480);
        setting_maxShiftWeightRT= 0.02f * (960 + 320);  // original is 0.02f * (640+480);
        /*林辉灿重设参数  EuRoC*/
        setting_kfGlobalWeight = 0.3f;   // original is 1.0. 0.3 is a balance between speed and accuracy. if tracking lost, set this para higher
        setting_maxShiftWeightT= 0.04f * (640 + 360);   // original is 0.04f * (640+480); this para is depend on the crop size.
        setting_maxShiftWeightR= 0.04f * (640 + 360);    // original is 0.0f * (640+480);
        setting_maxShiftWeightRT= 0.02f * (640 + 360);  // original is 0.02f * (640+480);*/


        setting_logStuff = false;
    }
    else if(preset == 2 || preset == 3)
    {
        printf("FAST settings:\n"
                "- %s real-time enforcing\n"
                "- 800 active points\n"
                "- 4-6 active frames\n"
                "- 1-4 LM iteration each KF\n"
                "- 424 x 320 image resolution\n", preset==0 ? "no " : "5x");

        playbackSpeed = (preset==2 ? 0 : 5);
        preload = preset==3;
        setting_desiredImmatureDensity = 600;
        setting_desiredPointDensity = 800;
        setting_minFrames = 4;
        setting_maxFrames = 6;
        setting_maxOptIterations=4;
        setting_minOptIterations=1;

        benchmarkSetting_width = 424;
        benchmarkSetting_height = 320;

        setting_logStuff = false;
    }

    printf("==============================================\n");
}

void parseArgument(char* arg)
{
    int option;
    char buf[1000];

    if(1==sscanf(arg,"preset=%d",&option))
    {
        settingsDefault(option);
        return;
    }

    if(1==sscanf(arg,"mode=%d",&option))
    {
        mode = option;
        if(option==0)
        {
            printf("PHOTOMETRIC MODE WITH CALIBRATION!\n");
        }
        if(option==1)
        {
            printf("PHOTOMETRIC MODE WITHOUT CALIBRATION!\n");
            setting_photometricCalibration = 0;
            setting_affineOptModeA = 0; //-1: fix. >=0: optimize (with prior, if > 0).
            setting_affineOptModeB = 0; //-1: fix. >=0: optimize (with prior, if > 0).
        }
        if(option==2)
        {
            printf("PHOTOMETRIC MODE WITH PERFECT IMAGES!\n");
            setting_photometricCalibration = 0;
            setting_affineOptModeA = -1; //-1: fix. >=0: optimize (with prior, if > 0).
            setting_affineOptModeB = -1; //-1: fix. >=0: optimize (with prior, if > 0).
            setting_minGradHistAdd=3;
        }
        return;
    }

    if(1==sscanf(arg,"sampleoutput=%d",&option))
    {
        if(option==1)
        {
            useSampleOutput = true;
            printf("USING SAMPLE OUTPUT WRAPPER!\n");
        }
        return;
    }

    if(1==sscanf(arg,"quiet=%d",&option))
    {
        if(option==1)
        {
            setting_debugout_runquiet = true;
            printf("QUIET MODE, I'll shut up!\n");
        }
        return;
    }


    if(1==sscanf(arg,"nolog=%d",&option))
    {
        if(option==1)
        {
            setting_logStuff = false;
            printf("DISABLE LOGGING!\n");
        }
        return;
    }

    if(1==sscanf(arg,"nogui=%d",&option))
    {
        if(option==1)
        {
            disableAllDisplay = true;
            printf("NO GUI!\n");
        }
        return;
    }
    if(1==sscanf(arg,"nomt=%d",&option))
    {
        if(option==1)
        {
            multiThreading = false;
            printf("NO MultiThreading!\n");
        }
        return;
    }
    if(1==sscanf(arg,"calib=%s",buf))
    {
        calib = buf;
        printf("loading calibration from %s!\n", calib.c_str());
        return;
    }
    if(1==sscanf(arg,"vignette=%s",buf))
    {
        vignetteFile = buf;
        printf("loading vignette from %s!\n", vignetteFile.c_str());
        return;
    }

    if(1==sscanf(arg,"gamma=%s",buf))
    {
        gammaFile = buf;
        printf("loading gammaCalib from %s!\n", gammaFile.c_str());
        return;
    }

    printf("could not parse argument \"%s\"!!\n", arg);
}




FullSystem* fullSystem = 0;
Undistort* undistorter = 0;
int frameID = 0;

void vidCb(const sensor_msgs::ImageConstPtr img)
{
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    assert(cv_ptr->image.type() == CV_8U);
    assert(cv_ptr->image.channels() == 1);

    if(setting_fullResetRequested)
    {
        std::vector<IOWrap::Output3DWrapper*> wraps = fullSystem->outputWrapper;
        delete fullSystem;
        for(IOWrap::Output3DWrapper* ow : wraps) ow->reset();
        fullSystem = new FullSystem();
        fullSystem->linearizeOperation=false;
        fullSystem->outputWrapper = wraps;
        if(undistorter->photometricUndist != 0)
            fullSystem->setGammaFunction(undistorter->photometricUndist->getG());
        setting_fullResetRequested=false;
    }

    MinimalImageB minImg((int)cv_ptr->image.cols, (int)cv_ptr->image.rows,(unsigned char*)cv_ptr->image.data);
    ImageAndExposure* undistImg = undistorter->undistort<unsigned char>(&minImg, 1,0, 1.0f);
    //fullSystem->addActiveFrame(undistImg, frameID);
    frameID++;
    delete undistImg;

}

void callback(const sensor_msgs::ImageConstPtr& img, const sensor_msgs::ImageConstPtr& img_right)
{
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    assert(cv_ptr->image.type() == CV_8U);
    assert(cv_ptr->image.channels() == 1);

    cv_bridge::CvImagePtr cv_ptr_right = cv_bridge::toCvCopy(img_right, sensor_msgs::image_encodings::MONO8);
    assert(cv_ptr_right->image.type() == CV_8U);
    assert(cv_ptr_right->image.channels() == 1);

    // HOTFIX for some NIIAS logs [cause of LeftImg (768, 1024), RightImg(772, 1032)]
    if (cv_ptr->image.cols != cv_ptr_right->image.cols)
    {
        if (cv_ptr->image.cols < cv_ptr_right->image.cols)  // Left img is smaller, than right one
        {
           /*cv::Mat tmp_right = cv_ptr_right->image(cv::Rect(0,0, cv_ptr->image.cols, cv_ptr->image.rows));
           cv_ptr_right->image = tmp_right.clone(); */
            cv_ptr_right->image(cv::Rect(0,0, cv_ptr->image.cols, cv_ptr->image.rows)).copyTo(cv_ptr_right->image);
        } else                                              // Right img is smaller, than left one
        {
            cv_ptr->image(cv::Rect(0,0, cv_ptr_right->image.cols, cv_ptr_right->image.rows)).copyTo(cv_ptr->image);
        }
    }


    if(setting_fullResetRequested)
    {
        std::vector<IOWrap::Output3DWrapper*> wraps = fullSystem->outputWrapper;
        delete fullSystem;
        for(IOWrap::Output3DWrapper* ow : wraps) ow->reset();
        fullSystem = new FullSystem();
        fullSystem->linearizeOperation=false;
        fullSystem->outputWrapper = wraps;
        if(undistorter->photometricUndist != 0)
            fullSystem->setGammaFunction(undistorter->photometricUndist->getG());
        setting_fullResetRequested=false;
    }

    MinimalImageB minImg((int)cv_ptr->image.cols, (int)cv_ptr->image.rows,(unsigned char*)cv_ptr->image.data);
    MinimalImageB minImg_right((int)cv_ptr_right->image.cols, (int)cv_ptr_right->image.rows,(unsigned char*)cv_ptr_right->image.data);
    ImageAndExposure* undistImg = undistorter->undistort<unsigned char>(&minImg, 1,0, 1.0f);
    ImageAndExposure* undistImg_right = undistorter->undistort<unsigned char>(&minImg_right, 1,0, 1.0f);

    fullSystem->addActiveFrame(undistImg, undistImg_right, frameID);
    frameID++;
    //printf("frameID: %d\n", frameID);
    //delete undistImg;
    //delete undistImg_right;
}

class PosePublisher : public dso::IOWrap::Output3DWrapper {
public:
  PosePublisher(ros::NodeHandle& nh)
    : pose_pub(nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("stereo_dso/pose", 10)), 
    odom_pub(nh.advertise<nav_msgs::Odometry>("stereo_dso/odometry", 10)),
    frame_number(0)
  {
      // ro = (x^2+y^2+z^2)^(1/2)   suggested whole standart deviation
      // Suggest, that x=y=z (deviation is equal in all sides)
      // ro = x*(3)^1/2 --> x = ro/(3)^(1/2)    standart deviation in every side
      // --> cov = ro^2     (cov = covariance)
      // pose_cov = std::pow(0.05, 2)/3;
      pose_cov = std::pow(5, 2)/3;
      orient_cov = std::pow(0.01, 2);
  }

  // frame->camToWorld - Mat 3x4, матрица преобразования в систему мировых координат
  void publishCamPose(FrameShell* frame, CalibHessian* HCalib) override {
    Eigen::Quaterniond quat(frame->camToWorld.rotationMatrix());
    Eigen::Vector3d trans = frame->camToWorld.translation();

    // Create Pose message
    geometry_msgs::PoseWithCovarianceStamped pose_msg;
    pose_msg.header.stamp = ros::Time::now();    // ros::Time(frame->timestamp)
    pose_msg.header.frame_id = "odom";
    pose_msg.header.seq = ++frame_number;

    // Position   
    pose_msg.pose.pose.position.x = trans.z();
    pose_msg.pose.pose.position.y = trans.x();
    pose_msg.pose.pose.position.z = trans.y();
    pose_msg.pose.pose.orientation.w = quat.w();
    pose_msg.pose.pose.orientation.x = quat.x();
    pose_msg.pose.pose.orientation.y = quat.y();
    pose_msg.pose.pose.orientation.z = quat.z(); 

    // // Original data (Previous variant)
    // pose_msg.pose.pose.position.x = trans.x();
    // pose_msg.pose.pose.position.y = trans.y();
    // pose_msg.pose.pose.position.z = trans.z();
    // pose_msg.pose.pose.orientation.w = quat.w();
    // pose_msg.pose.pose.orientation.x = quat.x();
    // pose_msg.pose.pose.orientation.y = quat.y();
    // pose_msg.pose.pose.orientation.z = quat.z();

    // IDEA: more accurate covariance estimate 
    // Covariance
    pose_msg.pose.covariance = boost::array<double, 36>({
        pose_cov, 0.,       0.,       0.,         0.,         0.,
        0.,       pose_cov, 0.,       0.,         0.,         0.,
        0.,       0.,       pose_cov, 0.,         0.,         0.,
        0.,       0.,       0.,       orient_cov, 0.,         0.,
        0.,       0.,       0.,       0.,         orient_cov, 0.,
        0.,       0.,       0.,       0.,         0.,         orient_cov
    });
    
    // Create Odometry message
    nav_msgs::Odometry odom_msg;
    odom_msg.pose = pose_msg.pose;
    odom_msg.header.stamp = ros::Time::now();    // ros::Time(frame->timestamp)
    odom_msg.header.frame_id = "odom";
    odom_msg.header.seq = ++frame_number;
    odom_msg.child_frame_id = "base_link";

    // Publish messages
    pose_pub.publish(pose_msg);
    odom_pub.publish(odom_msg);

    // Work with TF
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(trans.z(), trans.x(), trans.y()) );
    // // Original data
    // transform.setOrigin( tf::Vector3(trans.x(), trans.y(), trans.z()) );
    tf::Quaternion q;
    q[0] = quat.x();
    q[1] = quat.y();
    q[2] = quat.z();
    q[3] = quat.w();
    transform.setRotation(q);

    static tf::TransformBroadcaster tf_br;
    tf_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "tf/stereo_dso"));
  }

private:
  ros::Publisher pose_pub, odom_pub;
  int frame_number;
  float pose_cov, orient_cov;
};

class NavSatPublisher {
    ros::Publisher navsat_pub;
    ros::Subscriber navsat_sub;
    float lat_cov, long_cov, alt_cov;

public:
    NavSatPublisher(ros::NodeHandle& nh)
    {
        // Navigation data (longitude and latitude) --> ROUGH SUGGESTION
        // -------------------------------------------------------
        // In case, when longitude or latitude of 2 points are the same, distance between them:
        // 1 grad ~= 60 sea miles; 1 minute ~= 1 sea mile ~= 1852 metres
        // 1 grad ~= 111120 m
        // If suggest, that accuracy ~= 0.05m, than variance:
        double var = 0.05 / 111120.0;
        lat_cov = long_cov = alt_cov = float(std::pow(var, 2)/std::sqrt(3));   // distribute into 3 directions

        navsat_pub = nh.advertise<sensor_msgs::NavSatFix>("navigation_data/with_cov", 10);
        navsat_sub = nh.subscribe("navigation_data", 10, &NavSatPublisher::navsat_callback, this);
    }

    void navsat_callback(const sensor_msgs::NavSatFixPtr& nav_msg)
    {
        sensor_msgs::NavSatFix navCov_msg;

        navCov_msg.header = nav_msg->header;    // ros::Time(frame->timestamp)
        navCov_msg.latitude = nav_msg->latitude;
        navCov_msg.longitude = nav_msg->longitude;
        navCov_msg.altitude = nav_msg->altitude;

        navCov_msg.position_covariance_type = navCov_msg.COVARIANCE_TYPE_DIAGONAL_KNOWN;
        navCov_msg.position_covariance = boost::array<double, 9>({
                                                                    lat_cov, 0.,        0.,
                                                                    0.,      long_cov,  0.,
                                                                    0.,      0.,        alt_cov,
                                                                });

        navsat_pub.publish(navCov_msg);
    }
};


int main( int argc, char** argv )
{
    ros::init(argc, argv, "stereo_dso_ros");

    for(int i=1; i<argc;i++)
        parseArgument(argv[i]);

    /*
    //setting_desiredImmatureDensity = 1000;
    //setting_desiredPointDensity = 1200;
    //setting_minFrames = 5;
    //setting_maxFrames = 7;
    //setting_maxOptIterations=4;
    //setting_minOptIterations=1;
    //setting_logStuff = false;
    //setting_kfGlobalWeight = 1.3;


    //printf("MODE WITH CALIBRATION, but without exposure times!\n");
    //setting_photometricCalibration = 2;
    //setting_affineOptModeA = 0;
    //setting_affineOptModeB = 0;
    */

    undistorter = Undistort::getUndistorterForFile(calib, gammaFile, vignetteFile);

    setGlobalCalib(
            (int)undistorter->getSize()[0],
            (int)undistorter->getSize()[1],
            undistorter->getK().cast<float>());

    baseline = undistorter->getBl();
    //setBaseline();


    fullSystem = new FullSystem();
    fullSystem->linearizeOperation=false;


    if(!disableAllDisplay)
        fullSystem->outputWrapper.push_back(new IOWrap::PangolinDSOViewer(
                 (int)undistorter->getSize()[0],
                 (int)undistorter->getSize()[1]));


    // if(useSampleOutput)
    //     fullSystem->outputWrapper.push_back(new IOWrap::SampleOutputWrapper());


    if(undistorter->photometricUndist)
        fullSystem->setGammaFunction(undistorter->photometricUndist->getG());

    ros::NodeHandle nh;
    //ros::Subscriber imgSub = nh.subscribe("image", 1, &vidCb);
    /**********************************************************************/
    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/cam0/image_raw", 1);  // "/camera/left/image_raw"
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "/cam1/image_raw", 1); // "/camera/right/image_raw"
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub, right_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));
    /**********************************************************************/

    // Add pose publishing
    fullSystem->outputWrapper.push_back(new PosePublisher(nh));
//    NavSatPublisher navsatPub(nh);

    ros::spin();

    for(IOWrap::Output3DWrapper* ow : fullSystem->outputWrapper)
    {
        ow->join();
        delete ow;
    }

    fullSystem->printResult("~/Desktop/result.txt");
    delete undistorter;
    delete fullSystem;

    return 0;
}

