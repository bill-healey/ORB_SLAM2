/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<sstream>
#include<chrono>

#include<opencv2/opencv.hpp>

#include<System.h>
#include "zhelpers.h"
#include <stdint.h>

using namespace std;
void* context = NULL;
void* publisher = NULL;

void init_zmq() {
	context = zmq_init(1);

    publisher = zmq_socket(context, ZMQ_PUB);

	cout << "socket" << endl;

    //  Prevent publisher overflow from slow subscribers
    uint64_t hwm = 1;
    zmq_setsockopt(publisher, ZMQ_SNDHWM, &hwm, sizeof(hwm));

	cout << "setopt" << endl;

    int rc = zmq_bind(publisher, "tcp://*:5556");

	cout << "bind " << rc << endl;
}

void zmq_cleanup() {
    zmq_close(publisher);
    zmq_term(context);
	cout << "zmq cleaned up" << endl;
}

void zmq_send_const_string(const char* string) {
	int sent_bytes = zmq_send_const(publisher, string, strlen(string), 0);
	cout << "sent " << string << endl;
}

int main(int argc, char **argv) {
    std::string voc = "../../Vocabulary/ORBvoc.bin";
    std::string settings = "tiny6.yaml";
    //std::string settings = "iphone.yaml";
    cv::Mat frame;
    cv::Mat pose;
    cv::VideoCapture cap;

    int frames_since_last_send = 0;

	init_zmq();    
	//Sleep(10000);
	//zmq_send_const_string("pose|this is a test");

    bool open = cap.open(0);
    //superx
    //cap.set(4,704);
    //cap.set(5,480);

    //tiny6
    cap.set(4,720);
    cap.set(5,576);
    ORB_SLAM2::System SLAM(voc,settings,ORB_SLAM2::System::MONOCULAR,true);

    while(true) {
        bool frame_captured = cap.read(frame);
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        double tframe = 33.3;
        pose = SLAM.TrackMonocular(frame, tframe);
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        frames_since_last_send++;
        if (frames_since_last_send > 3) {
            std::stringstream ss;
            ss << "pose|" << pose;
            zmq_send_const_string(ss.str().c_str());
            cout << ss.str() << endl;
            frames_since_last_send = 0;
        }
        cout << "Frame time: " << ttrack << endl;
    }

    frame.release();
    cap.release();
    SLAM.Shutdown();
    //SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    return 0;
}

//Example pose
//  [0.97945923, 0.18689239, 0.075702704, -0.0023388132;
//  -0.20024681, 0.94562346, 0.2563152, 0.0025395327;
//  -0.023682896, -0.26620951, 0.96362418, 0.0046321955;
//   0, 0, 0, 1]
