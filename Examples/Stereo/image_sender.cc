/**
 * Author: Dariush Bahri
 * Connects to the external ORB system via TCP connection and sends each left and right stereo image seperately one by
 * one.
 */


#include<iostream>
#include<algorithm>
#include<fstream>
#include<iomanip>

#include<opencv2/core/core.hpp>

#include<System.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>


using namespace std;

void LoadImages(const string &strPathLeft, const string &strPathRight, const string &strPathTimes,
                vector<string> &vstrImageLeft, vector<string> &vstrImageRight, vector<double> &vTimeStamps);

int main(int argc, char **argv)
{

    // Retrieve paths to images
    vector<string> vstrImageLeft;
    vector<string> vstrImageRight;
    vector<double> vTimeStamp;
    LoadImages("/home/daz/capture/left", "/home/daz/capture/right", "/home/daz/capture/time.txt", vstrImageLeft, vstrImageRight, vTimeStamp);

    // Create a socket
    int s0 = socket(AF_INET, SOCK_STREAM, 0);
    if (s0 < 0) {
        perror("Cannot create a socket"); exit(1);
    }

    // Fill in the address of server
    struct sockaddr_in peeraddr;
    int peeraddr_len;
    memset(&peeraddr, 0, sizeof(peeraddr));
    char* peerHost = "localhost"; // TODO: ip of csteach0 = 144.32.125.156. Was localhost

    // Resolve the server address (convert from symbolic name to IP number)
    struct hostent *host = gethostbyname(peerHost);
    if (host == NULL) {
        perror("Cannot define host address"); exit(1);
    }
    peeraddr.sin_family = AF_INET;
    short peerPort = 1234;

    peeraddr.sin_port = htons(peerPort);

    // Print a resolved address of server (the first IP of the host)
    printf(
            "peer addr = %d.%d.%d.%d, port %d\n",
            host->h_addr_list[0][0] & 0xff,
            host->h_addr_list[0][1] & 0xff,
            host->h_addr_list[0][2] & 0xff,
            host->h_addr_list[0][3] & 0xff,
            (int) peerPort
    );

    // Write resolved IP address of a server to the address structure
    memmove(&(peeraddr.sin_addr.s_addr), host->h_addr_list[0], 4);

    // Connect to a remote server
    int res = connect(s0, (struct sockaddr*) &peeraddr, sizeof(peeraddr));
    if (res < 0) {
        perror("Cannot connect");
        exit(1);
    }
    printf("Connected. Send images.\n");
    const int nImages = vstrImageLeft.size();
    int leftImgSize, rightImgSize;
    cv::Mat imLeft, imRight, imLeftRect, imRightRect;

    for(int ni=0; ni<nImages; ni++) {
        // Read left and right images from file
        imLeft = cv::imread(vstrImageLeft[ni], CV_LOAD_IMAGE_UNCHANGED);
        imLeft = (imLeft.reshape(0, 1)); // to make it continuous
        leftImgSize = imLeft.total() * imLeft.elemSize();

        // Send left data here
        send(s0, imLeft.data, leftImgSize, 0);

        imRight = cv::imread(vstrImageRight[ni], CV_LOAD_IMAGE_UNCHANGED);
        imRight = (imLeft.reshape(0, 1));
        rightImgSize = imRight.total() * imRight.elemSize();

        // Send right data here
        send(s0, imRight.data, rightImgSize, 0);

    }
    close(s0);

    return 0;
}

void LoadImages(const string &strPathLeft, const string &strPathRight, const string &strPathTimes,
                vector<string> &vstrImageLeft, vector<string> &vstrImageRight, vector<double> &vTimeStamps)
{
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(5000);
    vstrImageLeft.reserve(5000);
    vstrImageRight.reserve(5000);
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            vstrImageLeft.push_back(strPathLeft + "/" + ss.str() + ".jpg");
            vstrImageRight.push_back(strPathRight + "/" + ss.str() + ".jpg");
            double t;
            ss >> t;
            vTimeStamps.push_back(t/1e9); // Add time stamp to time stamp vector

        }
    }
}