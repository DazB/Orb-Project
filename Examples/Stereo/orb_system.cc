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
#include<iomanip>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>

#include <sys/socket.h>
#include <netinet/in.h>

using namespace std;

int s1; // Data socket

// Connects to a single client that sends the left and right stereo images for ORB SLAM to process
void connect_to_image_sender();

int main(int argc, char **argv) {

    // Read rectification parameters
    cv::FileStorage fsSettings("/home/daz/Project/Orb-Project/Examples/Stereo/ps_eye.yaml", cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        cerr << "ERROR: Wrong path to settings" << endl;
        return -1;
    }

    cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
    fsSettings["LEFT.K"] >> K_l;
    fsSettings["RIGHT.K"] >> K_r;

    fsSettings["LEFT.P"] >> P_l;
    fsSettings["RIGHT.P"] >> P_r;

    fsSettings["LEFT.R"] >> R_l;
    fsSettings["RIGHT.R"] >> R_r;

    fsSettings["LEFT.D"] >> D_l;
    fsSettings["RIGHT.D"] >> D_r;

    int rows_l = fsSettings["LEFT.height"];
    int cols_l = fsSettings["LEFT.width"];
    int rows_r = fsSettings["RIGHT.height"];
    int cols_r = fsSettings["RIGHT.width"];

    if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
       rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
    {
        cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
        return -1;
    }

    cv::Mat M1l,M2l,M1r,M2r;
    cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,M1l,M2l);
    cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,M1r,M2r);

    //const int nImages = vstrImageLeft.size();
    const int nImages = 1440; // TODO: change so not hardcode. Number of images sent

    // Get timestamp vector // TODO: sort this shit out. Either send with image, or store seperately?
    ifstream fTimes;
    vector<double> vTimeStamp;
    fTimes.open("/home/daz/capture/time.txt");
    vTimeStamp.reserve(5000);
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimeStamp.push_back(t/1e9); // Add time stamp to time stamp vector
        }
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM("/home/daz/Project/Orb-Project/Vocabulary/ORBvoc.txt","/home/daz/Project/Orb-Project/Examples/Stereo/ps_eye.yaml",ORB_SLAM2::System::STEREO,true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Images in the sequence (hardcoded known number): " << nImages << endl << endl;

    cout << "Waiting to connect to sender......" << endl;

    connect_to_image_sender(); // Connect to robot, so can send images through data socket s1

    int height = 480;
    int width = 640;
    cv::Mat imLeft, imRight, imLeftRect, imRightRect;

    imLeft = cv::Mat::zeros(height, width, CV_8UC3);
    imRight = cv::Mat::zeros(height, width, CV_8UC3);

    int  imgSize = imLeft.total()*imLeft.elemSize(); // Left and right images are of the same size
    uchar sockData[imgSize];
    ssize_t bytes;

    for (int numImages = 0; numImages < 1440; numImages++) { // TODO: change so can get number of images being sent

        //Receive data here
        for (int i = 0; i < imgSize; i += bytes) {
            if ((bytes = recv(s1, sockData + i, imgSize - i, 0)) == -1) {
                perror("Cannot bind a socket");
                exit(1);
            }
        }

        // Assign pixel value to img
        int ptr = 0;
        for (int i = 0; i < imLeft.rows; i++) {
            for (int j = 0; j < imLeft.cols; j++) {
                imLeft.at<cv::Vec3b>(i, j) = cv::Vec3b(sockData[ptr + 0], sockData[ptr + 1], sockData[ptr + 2]);
                ptr = ptr + 3;
            }
        }
        cout << "Received left" << endl;

//        cv::namedWindow("left", CV_WINDOW_AUTOSIZE);// Create a window for display.
//        cv::imshow("left", leftImg);
//        cv::waitKey(0);

        //Receive data here
        for (int i = 0; i < imgSize; i += bytes) {
            if ((bytes = recv(s1, sockData + i, imgSize - i, 0)) == -1) {
                perror("Cannot bind a socket");
                exit(1);
            }
        }

        // Assign pixel value to img
        ptr = 0;
        for (int i = 0; i < imRight.rows; i++) {
            for (int j = 0; j < imRight.cols; j++) {
                imRight.at<cv::Vec3b>(i, j) = cv::Vec3b(sockData[ptr + 0], sockData[ptr + 1], sockData[ptr + 2]);
                ptr = ptr + 3;
            }
        }
        cout << "Received right" << endl;
//        cv::namedWindow("right", CV_WINDOW_AUTOSIZE);// Create a window for display.
//        cv::imshow("right", rightImg);
//        cv::waitKey(0);

        // We have the images, now run the ORB-SLAM system
        cv::remap(imLeft, imLeftRect, M1l, M2l, cv::INTER_LINEAR);
        cv::remap(imRight, imRightRect, M1r, M2r, cv::INTER_LINEAR);

        double tframe = vTimeStamp[numImages];


#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the images to the SLAM system
        SLAM.TrackStereo(imLeftRect, imRightRect, tframe);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[numImages] = ttrack;

        // Wait to load the next frame
        double T = 0;
        if (numImages < nImages - 1)
            T = vTimeStamp[numImages + 1] - tframe;
        else if (numImages > 0)
            T = tframe - vTimeStamp[numImages - 1];

        if (ttrack < T)
            usleep((T - ttrack) * 1e6);
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
    return 0;

}

void connect_to_image_sender() {
    int listenPort = 1234;

    // Create a socket
    int s0 = socket(AF_INET, SOCK_STREAM, 0);
    if (s0 < 0) {
        perror("Cannot create a socket");
        exit(1);
    }

    // Fill in the address structure containing self address
    struct sockaddr_in myaddr;
    memset(&myaddr, 0, sizeof(struct sockaddr_in));
    myaddr.sin_family = AF_INET;
    myaddr.sin_port = htons(listenPort);        // Port to listen
    myaddr.sin_addr.s_addr = htonl(INADDR_ANY);

    // Bind a socket to the address
    int res = bind(s0, (struct sockaddr *) &myaddr, sizeof(myaddr));
    if (res < 0) {
        perror("Cannot bind a socket");
        exit(1);
    }

    // Set the "LINGER" timeout to zero, to close the listen socket
    // immediately at program termination.
    struct linger linger_opt = {1, 0}; // Linger active, timeout 0
    setsockopt(s0, SOL_SOCKET, SO_LINGER, &linger_opt, sizeof(linger_opt));

    // Now, listen for a connection
    res = listen(s0, 1);    // "1" is the maximal length of the queue
    if (res < 0) {
        perror("Cannot listen");
        exit(1);
    }

    // Accept a connection (the "accept" command waits for a connection with
    // no timeout limit...)
    struct sockaddr_in peeraddr;
    socklen_t peeraddr_len;
    s1 = accept(s0, (struct sockaddr *) &peeraddr, &peeraddr_len);
    if (s1 < 0) {
        perror("Cannot accept");
        exit(1);
    }

    // A connection is accepted. The new socket "s1" is created
    // for data input/output. The peeraddr structure is filled in with
    // the address of connected entity, print it.
    printf(
            "Connection from IP %d.%d.%d.%d, port %d\n",
            (ntohl(peeraddr.sin_addr.s_addr) >> 24) & 0xff, // High byte of address
            (ntohl(peeraddr.sin_addr.s_addr) >> 16) & 0xff, // . . .
            (ntohl(peeraddr.sin_addr.s_addr) >> 8) & 0xff,  // . . .
            ntohl(peeraddr.sin_addr.s_addr) & 0xff,         // Low byte of addr
            ntohs(peeraddr.sin_port)
    );

    res = close(s0);    // Close the listen socket
}


