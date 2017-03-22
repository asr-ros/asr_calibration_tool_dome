/**

Copyright (c) 2016, Heller Florian, Stöckle Patrick, Wittenbeck Valerij
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include <ros/ros.h>
#include <asr_msgs/AsrObject.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <posedetection_msgs/Object6DPose.h>
#include <string>
#include <iostream>
#include <fstream>
#include <boost/thread.hpp>


class DataTracker {
public:
        ros::NodeHandle nh;

        // Subscriptions
        ros::Subscriber ptu_sub;
        ros::Subscriber fob_sub;
        ros::Subscriber cbd_sub;

        std::string ptu_topic, fob_topic, cbd_topic, output_file_path;

        double pan, tilt;
        geometry_msgs::Pose fob_pose;
        geometry_msgs::Pose cbd_pose;

        int data_counter;

        void ptuCB(const sensor_msgs::JointState& ptu_msg) {
            ROS_DEBUG("ptu Message recieved");
            pan = ptu_msg.position[0];
            tilt = ptu_msg.position[1];
        }

        void fobCB(const asr_msgs::AsrObject& fob_msg) {
            if (!fob_msg.identifier.empty() && strcmp(fob_msg.identifier.c_str(), "tracker_left_raw") == 0) {
                ROS_DEBUG("fob message recieved");
                geometry_msgs::PoseWithCovariance current_pose_with_c = fob_msg.sampledPoses.back();
                fob_pose = current_pose_with_c.pose;
            }
        }
        void cbdCB(const geometry_msgs::PoseStamped& cbd_msg) {
            ROS_DEBUG("cbd message recieved");
            ROS_INFO("cbd message recieved");
            cbd_pose = cbd_msg.pose;
        }

        DataTracker(ros::NodeHandle nh){
            this->nh = nh;
            nh.getParam("ptu_topic", ptu_topic);
            nh.getParam("fob_topic", fob_topic);
            nh.getParam("cbd_topic", cbd_topic);
            nh.getParam("output_file_path", output_file_path);

            ptu_sub = nh.subscribe(ptu_topic, 1, &DataTracker::ptuCB, this);
            fob_sub = nh.subscribe(fob_topic, 1, &DataTracker::fobCB, this);
            cbd_sub = nh.subscribe(cbd_topic, 1, &DataTracker::cbdCB, this);
            data_counter = 0;
            boost::thread trackerThread(spin, this);
            ros::spin();
        }
        static void spin(DataTracker* obj){
            ROS_INFO("Capturing started.");
            ROS_INFO("Enter 'e' to stop capturing.");
            while (std::cin.get() != 'e'){
               obj->write();
            }
            ROS_INFO("Capturing stopped.");
        }


        void write(){
            std::stringstream ss;
            ss << pan << "; " << tilt << "; ";
            ss << fob_pose.position.x << "; " << fob_pose.position.y << "; " << fob_pose.position.z << "; ";
            ss << fob_pose.orientation.w << "; " << fob_pose.orientation.x << "; " << fob_pose.orientation.y << "; " << fob_pose.orientation.z << "; ";
            ss << cbd_pose.position.x << "; " << cbd_pose.position.y << "; " << cbd_pose.position.z << std::endl;
            ROS_INFO("%s", ss.str().c_str());
            std::ofstream dataFile;
            dataFile.open (output_file_path.c_str(),std::ios::app);
            dataFile << ss.str();
            dataFile.close();
            ROS_INFO("data captured %d", data_counter);
            data_counter++;
        }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "calibration_tracker");

  ros::NodeHandle nh("~");
  new ::DataTracker(nh);
  //ros::spin();

  return 0;
}
