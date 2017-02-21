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

int main(int argc, char **argv) {
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
    int s1 = accept(s0, (struct sockaddr *) &peeraddr, &peeraddr_len);
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

    int height = 480;
    int width = 640;
    cv::Mat leftImg, rightImg;
    leftImg = cv::Mat::zeros(height, width, CV_8UC3);
    rightImg = cv::Mat::zeros(height, width, CV_8UC3);

    int  imgSize = leftImg.total()*leftImg.elemSize(); // Left and right images are of the same size
    uchar sockData[imgSize];
    ssize_t bytes;

    for (int numImages = 0; numImages < 1440*2; numImages++) {

        //Receive data here
        for (int i = 0; i < imgSize; i += bytes) {
            if ((bytes = recv(s1, sockData + i, imgSize - i, 0)) == -1) {
                perror("Cannot bind a socket");
                exit(1);
            }
        }

        // Assign pixel value to img
        int ptr = 0;
        for (int i = 0; i < leftImg.rows; i++) {
            for (int j = 0; j < leftImg.cols; j++) {
                leftImg.at<cv::Vec3b>(i, j) = cv::Vec3b(sockData[ptr + 0], sockData[ptr + 1], sockData[ptr + 2]);
                ptr = ptr + 3;
            }
        }
        printf("Received left");
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
        for (int i = 0; i < rightImg.rows; i++) {
            for (int j = 0; j < rightImg.cols; j++) {
                rightImg.at<cv::Vec3b>(i, j) = cv::Vec3b(sockData[ptr + 0], sockData[ptr + 1], sockData[ptr + 2]);
                ptr = ptr + 3;
            }
        }
        printf("Received right");
//        cv::namedWindow("right", CV_WINDOW_AUTOSIZE);// Create a window for display.
//        cv::imshow("right", rightImg);
//        cv::waitKey(0);
    }
    return 0;

}


