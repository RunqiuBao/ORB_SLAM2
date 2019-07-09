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
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

#include<System.h>
#include "MaskInfo.h"

#include <sys/stat.h>
#include <string>
#include <fstream>
#include <vector>
#include <sstream>

using namespace std;

vector<string> split(const string &s, char delim) {
    vector<string> elems;
    stringstream ss(s);
    string item;
    while (getline(ss, item, delim)) {
    if (!item.empty()) {
            elems.push_back(item);
        }
    }
    return elems;
}

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps, vector<string> &vImgNames)
{
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/mymav.txt";
    fTimes.open(strPathTimeFile.c_str());
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            
            std::string t2;
            ss >> t2;
            double t;
            ss >> t;
            //vTimestamps.push_back(t);
            vTimestamps.push_back(std::stod(s));//runqiu: here I modified the original stupid string to double method
            vImgNames.push_back(t2);
        }
    }

    string strPrefixLeft = strPathToSequence + "/left/undisR-1k-newcalib/";
    string strPrefixRight = strPathToSequence + "/right/undisR-1k-newcalib/";
    //string strPrefixLeft = strPathToSequence + "/left/datat2_1080r/";
    //string strPrefixRight = strPathToSequence + "/right/datat2_1080r/";
    //string strPrefixLeft = strPathToSequence + "/left/";
    //string strPrefixRight = strPathToSequence + "/right/";

    const int nTimes = vTimestamps.size();
    vstrImageLeft.resize(nTimes);
    vstrImageRight.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        //vstrImageLeft[i] = strPrefixLeft + ss.str() + ".png";
        //vstrImageRight[i] = strPrefixRight + ss.str() + ".png";
	    vstrImageLeft[i] = strPrefixLeft + vImgNames[i] + ".png";
        vstrImageRight[i] = strPrefixRight + vImgNames[i] + ".png";
    }
}

inline bool exists_test (const std::string& name) {
  struct stat buffer;   
  return (stat (name.c_str(), &buffer) == 0); 
}

inline bool exists_test0 (const std::string& name) {
    ifstream f(name.c_str());
    return f.good();
}

inline bool exists_test1 (const std::string& name) {
    if (FILE *file = fopen(name.c_str(), "r")) {
        fclose(file);
        return true;
    } else {
        return false;
    }   
}

void LoadMasks(const string &strPathToMask, vector<string> &vstrMaskLeft, vector<string> &vstrMaskedFrame, vector<string> &vImgNames)
{
    string strPrefixMaskLeft = strPathToMask + "/leftmask-nomargin/";
    string strPrefixMaskedFrame = strPathToMask + "/leftwithYOLO-nomargin/det_";
    string strPrefixMaskedFrame2 = strPathToMask + "/left/undisR-1k-newcalib/";
    const int nTimes = vImgNames.size();
    vstrMaskLeft.resize(nTimes);
    vstrMaskedFrame.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrMaskLeft[i] = strPrefixMaskLeft + vImgNames[i] + ".png";
        string maskedFrame=strPrefixMaskedFrame+vImgNames[i]+".png";
        if(exists_test1(maskedFrame))
            vstrMaskedFrame[i]=maskedFrame;
        else
        {
            maskedFrame=strPrefixMaskedFrame2+vImgNames[i]+".png";//runqiu: miss some frames that don't contain objects
            vstrMaskedFrame[i]=maskedFrame;
        }
        

    }
}

void LoadMasksPixel(const string &strPathToMask, vector<string> &vstrMaskLeft, vector<string> &vImgNames)
{
    string strPrefixMaskLeft = strPathToMask + "/left_maskrcnn/";

    const int nTimes = vImgNames.size();
    vstrMaskLeft.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrMaskLeft[i] = strPrefixMaskLeft + vImgNames[i] + ".png";
    }
}

void LoadMasksLabels(map<string, MaskSet> &masksSetDict, vector<string> masksText)
{
    for (std::vector<string>::iterator it=masksText.begin(); it != masksText.end(); ++it)
    {
        vector<string> oneFrameMasks = split(*it, ' ');
        int obsNb=(oneFrameMasks.size()-1)/5;//objects number
        MaskSet newObSet;//runqiu: masks set for one frame
        for(int i=0;i<obsNb;i++){
            string newlabel=oneFrameMasks.at(1+i);
            newObSet.labels.push_back(newlabel);
            bbox newbbox;
            newbbox.lux=stoi(oneFrameMasks.at(1+obsNb+i*4));
            newbbox.luy=stoi(oneFrameMasks.at(1+obsNb+i*4+1));
            newbbox.rlx=stoi(oneFrameMasks.at(1+obsNb+i*4+2));
            newbbox.rly=stoi(oneFrameMasks.at(1+obsNb+i*4+3));
            newObSet.bboxes.push_back(newbbox);
        }
        string frameName=oneFrameMasks.at(0);
        masksSetDict[frameName]=newObSet;
    }
}

int main(int argc, char **argv)
{
    if(argc != 4)
    {
       // cerr << endl << "Usage: ./stereo_kitti path_to_vocabulary path_to_settings path_to_sequence" << endl;
      //  return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageLeft;
    vector<string> vstrImageRight;
    vector<double> vTimestamps;
    vector<string> vImgNames;
    char *argvtemp[4]={" ","/media/chino/HD-PSFU3/testbar/ORB_SLAM2/Vocabulary/ORBvoc.bin","/media/chino/HD-PSFU3/testbar/ORB_SLAM2/Examples/Stereo/gopro34new.yaml","/media/chino/HD-PSFU3/testbar/slamDataset/stereo/mymav-stereo-do"};
 
    LoadImages(string(argvtemp[3]), vstrImageLeft, vstrImageRight, vTimestamps, vImgNames);//runqiu: vTimestamps is double numbers, vImgNames is exactly storing the image names
    char *strPathToMask="/media/chino/HD-PSFU3/testbar/slamDataset/stereo/mymav-stereo-do";//runqiu: add binary mask
    vector<string> vstrMaskLeft;
    vector<string> vstrMaskPixelLeft;
    vector<string> vstrMaskedFrame;
    LoadMasks(string(strPathToMask), vstrMaskLeft, vstrMaskedFrame, vImgNames);
    LoadMasksPixel(string(strPathToMask), vstrMaskPixelLeft, vImgNames);

    const int nImages = vstrImageLeft.size();

    //runqiu: read masks and semantic labels for every frame
    ifstream inFile("/media/chino/HD-PSFU3/testbar/slamDataset/stereo/mymav-stereo-do/masklist-nomargin.txt");
    vector<string> masksText;
	if(!inFile)
	{
		cout<<"Couldn't open the file"<<endl;
		exit(1);
	}
	string line;
	while( getline(inFile, line)  )
    {
        masksText.push_back(line);
    }
    map<string, MaskSet> masksSetDict;
    LoadMasksLabels(masksSetDict, masksText);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argvtemp[1],argvtemp[2],ORB_SLAM2::System::STEREO,true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;   

    // Main loop    
    cv::Mat imLeft, imRight;
    cv::Mat imMask;
    cv::Mat imMaskPixel;
    cv::Mat imMaskYolo;
    //cv::Mat imTf;//runqiu:get the pose of current frame
    // vector<std::string> tfRecordLine;
    int countP;
    countP=0;
    for(int ni=530; ni<nImages; ni++)
    {
        // Read left and right images from file
        imLeft = cv::imread(vstrImageLeft[ni],CV_LOAD_IMAGE_UNCHANGED);
        imRight = cv::imread(vstrImageRight[ni],CV_LOAD_IMAGE_UNCHANGED);
        //runqiu: read mask image from file
        imMask = cv::imread(vstrMaskLeft[0], CV_LOAD_IMAGE_UNCHANGED);
        imMaskPixel = cv::imread(vstrMaskPixelLeft[0], CV_LOAD_IMAGE_UNCHANGED);
        imMaskYolo = cv::imread(vstrMaskedFrame[ni], CV_LOAD_IMAGE_UNCHANGED);
        //cout << endl << std::to_string(ni)+" finish read!" << endl;
        double tframe = vTimestamps[ni];
        cout << vImgNames[ni] << endl << endl;

        int grayImgNum = 1;                                 //图像数
	    int grayChannels = 0 ;                              //需要计算的通道号 单通道只有 通道号为0
	    cv::Mat grayHist;                                   //灰度图输出直方图
	    const int grayHistDim = 1;                          //直方图维数
	    const int grayHistSize = 256 ;                      //直方图每一维度bin个数
	    float grayRanges[2] = { 0, 256 };                   //灰度值的统计范围
	    const float *grayHistRanges[1] = { grayRanges };    //灰度值统计范围指针
	    bool grayUniform = true;                            //是否均匀
	    bool grayAccumulate = false; 
        //计算灰度图像的直方图
	    cv::calcHist( &imMask, 
                  grayImgNum, 
                  &grayChannels, 
                  cv::Mat(), 
                  grayHist, 
                  grayHistDim, 
                  &grayHistSize, 
                  grayHistRanges, 
                  grayUniform, 
                  grayAccumulate );

	    float white_val = grayHist.at<float>(255);
        float black_val = grayHist.at<float>(0);
        float doi=white_val/(white_val+black_val);
        if(doi>0.2){//runqiu: set the threshold for switching to mask-rcnn
            imMask=imMaskPixel;//runqiu: hierarchical mask generation
            cout<<"Mask-RCNN:"<<vImgNames[ni]<<endl;
            countP++;
        }

        if(imLeft.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(vstrImageLeft[ni]) << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the images to the SLAM system
        //uchar thisone1=imMask.at<uchar>(22,424);
        //uchar thisone2=imMask.at<uchar>(24,814);
        //uchar thisone3=imMask.at<uchar>(322,816);
        //uchar thisone4=imMask.at<uchar>(322,425);
        //uchar mythreshold=255;
        //bool temp=(thisone1==mythreshold);
        MaskSet masksForThisFrame;
        masksForThisFrame=masksSetDict[vImgNames[ni]];//runqiu: need to see if empty
        SLAM.TrackStereo(imLeft,imRight,tframe,ni,imMask,imMaskPixel,imMaskYolo,masksForThisFrame);//runqiu: add binary mask
        //SLAM.openDOSwitch()
        //cv::Mat Rwc = imTf.rowRange(0,3).colRange(0,3).t();//rotation information
        //cv::Mat twc = -Rwc*imTf.rowRange(0,3).col(3);//translation information
        //vector<float> q = ORB_SLAM2::Converter::toQuaternion(Rwc);
        //vector<float> t = twc.cols;

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
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
    //SLAM.SaveTrajectoryKITTI("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryAndMap(SLAM.GetMap(), "KeyFrameTrajectory.txt", "MapPoints.txt");//runqiu:to save the point cloud map with trajectory
    SLAM.SaveTrajectoryKITTI("CameraTrajectory.txt");

    return 0;
    
}
