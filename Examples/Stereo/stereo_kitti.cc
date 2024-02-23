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

using namespace std;

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps);

// argc : num of argment
// argv : argument가 char형으로 저장되는 변수
// ** 이중 배열로써 표현
int main(int argc, char **argv)
{   
    // argc != 4인 경우 cerr를 통해 error message 출력
    if(argc != 4)
    {   
        // cerr : error를 출력
        cerr << endl << "Usage: ./stereo_kitti path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }
    
    // argv[0] : 실행 파일 | argv[1] : Vocabulary txt 파일 | argv[2] : yaml 파일 | argv[3] : data path
    
     // Retrieve paths to images
    vector<string> vstrImageLeft;   // left image path
    vector<string> vstrImageRight;  // right image path
    vector<double> vTimestamps;     // timestamps vector

    // timestamps 개수만큼 left image, right image 이미지를 불러옴
    LoadImages(string(argv[3]), vstrImageLeft, vstrImageRight, vTimestamps);

    // num of image
    const int nImages = vstrImageLeft.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,true);
            // System(Voc 경로, yaml 경로, sensor=stereo, viewer = true)

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;   


    // Main loop
    cv::Mat imLeft, imRight;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read left and right images from file (Rectificated image)
        imLeft = cv::imread(vstrImageLeft[ni],CV_LOAD_IMAGE_UNCHANGED);
        imRight = cv::imread(vstrImageRight[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni]; // ni번째 image의 timestamps

        // image가 없거나 경로가 잘못됐을 경우 error 출력
        if(imLeft.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(vstrImageLeft[ni]) << endl;
            return 1;
        }

// 일반적인 if문은 조건문이 false여도 컴파일은 되지만 #if문은 false인 경우 컴파일 자체가 되지 않음
// #if : 만약 ~ 라면
// #ifdef : 만약 ~가 정의되어 있다면 
// c++11 compile부터는 steady_clock, 이전은 monotonic_clock
#ifdef COMPILEDWITHC11 // CMakelist에 정의
        // steady_clock : 마지막 부팅 이후로 흘러간 시간 (초시계를 표현) (물리적 고정 시간)
        // system_clock : 컴퓨터 시스템 시간
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the images to the SLAM system
        SLAM.TrackStereo(imLeft,imRight,tframe);



#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        // differnce between t2 and t1
        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        // ni번째 t2-t1
        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;   // vTimestamps[ni+1] - vTimestamps[ni]
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];   // 마지막 ni의 T

        if(ttrack<T)
            usleep((T-ttrack)*1e6);    // micro sec동안 호출 프로세스의 실행을 일시 중지 sleep(1) = usleep(1000000)
    }

    // Stop all threads
    // 모든 thread가 끝나면 프로그램을 닫음
    SLAM.Shutdown();

    // Tracking time statistics
    // 이전 frame과 현재 frame사이의 시간 차이를 이용해 Tracking time을 구함
    sort(vTimesTrack.begin(),vTimesTrack.end());
    
    // totaltime = total tracking time
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    SLAM.SaveTrajectoryKITTI("CameraTrajectory.txt");

    return 0;
}


/**
 * @brief Load the left & right image and make timestamps vector 
 * @param strPathToSequence dataset 경로
 * @param vstrImageLeft 모든 left image path의 vector  
 * @param vstrImageRight 모든 right image path의 vector 
 * @param Timestamps timestamps vector
*/
void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps)
{
    ifstream fTimes; // input file stream
    string strPathTimeFile = strPathToSequence + "/times.txt"; // times.txt 파일 경로
    fTimes.open(strPathTimeFile.c_str()); // strPathTimeFile을 const char* 타입으로 변환 (open 함수에서 const char* 형식으로 받음)

    // end of file -> file 끝에 도달했을 경우 true return
    while(!fTimes.eof()) // eof를 사용하는 것은 파일 읽기의 안전 보장 x, txt 마지막에 공백이 있으면 null이 포함될 수 있음 -> size 달라짐
    { 
        string s;
        getline(fTimes,s); // std::string에 정의된 getline, 이렇게 하면 buf의 크기를 지정할 필요가 없음
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;  // ss에 담긴 내용 중에서 double형을 뽑아내는 것
            vTimestamps.push_back(t);
        }
    }
    
    // left & right image path
    string strPrefixLeft = strPathToSequence + "/image_0/";
    string strPrefixRight = strPathToSequence + "/image_1/";

    const int nTimes = vTimestamps.size(); // vTimestamps 길이
    // vTimestamps 길이와 동일하게 resize
    // reserve는 초기화 없이 공간만 할당
    // resize와 reserve를 하는 이유? -> 메모리 절약 + 시간 복잡도 감소
    vstrImageLeft.resize(nTimes); 
    vstrImageRight.resize(nTimes);


    // 총 000000 ~ nTimes 개수의 번호가 매겨짐
    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageLeft[i] = strPrefixLeft + ss.str() + ".png";
        vstrImageRight[i] = strPrefixRight + ss.str() + ".png";
    }
    // vstrImageLeft : 모든 left image path의 vector 
    // vstrImageright : 모든 right image path의 vector
}
