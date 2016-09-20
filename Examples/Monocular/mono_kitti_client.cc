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
#include<chrono>
#include<iomanip>
#include <grpc++/grpc++.h>

#include<opencv2/core/core.hpp>

#include "mono_kitti.grpc.pb.h"

#include"System.h"

using grpc::Channel;
using grpc::ClientContext;
using grpc::Status;
using mono_kitti::OrbSLAM;
using mono_kitti::NewSLAMRequest;
using mono_kitti::NewSLAMReturn;
using mono_kitti::SaveKeyFrameTrajectoryTUMRequest;
using mono_kitti::SaveKeyFrameTrajectoryTUMReturn;
using mono_kitti::ShutdownRequest;
using mono_kitti::ShutdownReturn;
using mono_kitti::TrackMonocularRequest;
using mono_kitti::TrackMonocularReturn;
using namespace std;

//enum ESensor { ROS = 0, RGBD = 1, STEREO = 2, MONOCULAR = 3};

void LoadImages(const string &strSequence, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);

class OrbSLAMClient {
 public:
  OrbSLAMClient(std::shared_ptr<Channel> channel) : stub_(OrbSLAM::NewStub(channel)) {};
  
  void NewSLAM(const string &strVocFile, const string &strSettingsFile, const mono_kitti::NewSLAMRequest_ESensor sensor, const bool bUseViewer) {

	NewSLAMRequest request;
	request.set_strvocfile(strVocFile);
	request.set_strsettingfile(strSettingsFile);
	request.set_sensor(sensor);
	request.set_buseviewer(bUseViewer);
	NewSLAMReturn reply;
  	ClientContext context;
  	stub_->NewSLAM(&context, request, &reply);
  }
  cv::Mat TrackMonocular(const cv::Mat &im, const double &timestamp) {
	// serialize cv::mat
	cv::Size size = im.size();
	std::vector<uchar> data(im.ptr(), im.ptr() + size.width * size.height* im.channels());
	std::string image(data.begin(), data.end());	
  	//uchar *data;
	//data = new uchar[size.width * size.height * im.channels()];
	//memcpy(data, im.ptr(), sizeof(uchar)*size.width * size.height * im.channels());
	//std::string iamge(data, data+ size.width * size.height * im.channels(), im.ptr() );
	TrackMonocularRequest request;
	request.set_im(image);
	request.set_im_width(size.width);
	request.set_im_height(size.height);
	request.set_im_channel(im.channels());
	request.set_im_type(im.type());
	TrackMonocularReturn reply;
 	ClientContext context;
	
	stub_->TrackMonocular(&context, request, &reply);

	// deserialize cv::mat
	std::string ret_im = reply.im();
	std::vector<uchar> ret_data(ret_im.begin(), ret_im.end());
	char *data_recv;
	data_recv = new char[ret_data.size()];	
	std::copy(ret_data.begin(), ret_data.end(), data_recv);	
	cv::Mat ret(reply.im_height(), reply.im_width(), reply.im_type(), data_recv );
	return ret;
  }
  void Shutdown() {
  	ShutdownRequest request;
	ShutdownReturn reply;
	ClientContext context;
	stub_->Shutdown(&context, request, &reply);
  }
  void SaveKeyFrameTrajectoryTUM(const string &filename) {
  	SaveKeyFrameTrajectoryTUMRequest request;
	request.set_filename(filename);
	SaveKeyFrameTrajectoryTUMReturn reply;
        ClientContext context;
	stub_->SaveKeyFrameTrajectoryTUM(&context, request, &reply);
  }
 private:
  std::unique_ptr<OrbSLAM::Stub> stub_;
};


int main(int argc, char **argv)
{
    if(argc != 4)
    {
        cerr << endl << "Usage: ./mono_kitti path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    LoadImages(string(argv[3]), vstrImageFilenames, vTimestamps);
    string strTime = string(argv[3])+"/time.txt";
    std::fstream fs;
    fs.open(strTime, std::fstream::in | std::fstream::out | std::fstream::trunc );

    int nImages = vstrImageFilenames.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    //ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);
    OrbSLAMClient SLAM(
	grpc::CreateChannel("localhost:50051",
			grpc::InsecureChannelCredentials())
    );
    SLAM.NewSLAM(argv[1], argv[2], mono_kitti::NewSLAMRequest_ESensor::NewSLAMRequest_ESensor_MONOCULAR, true);

 // this should be replaced by some constructive function
	// before that, the client should be connected to the grpc server


    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat im;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image from file
        im = cv::imread(vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(im.empty())
        {
            cerr << endl << "Failed to load image at: " << vstrImageFilenames[ni] << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
	cv::Mat ret = SLAM.TrackMonocular(im,tframe); 	// this is a grpc call
						// a new tpye of grpc call might be required to return the trajectory the previous
	cout << "shoot" << endl;
	cout << setprecision(7) << " " << ret.at<float>(0) << " " << ret.at<float>(1) << " " << ret.at<float>(2) << endl;

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;
        cout << ttrack << endl; 
 
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
    SLAM.Shutdown();   // this is another grpc call

    // Tracking time statistics
    //sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
	cout << vTimesTrack[ni] << endl;
        fs << vTimesTrack[ni] << endl; 
        totaltime+=vTimesTrack[ni];
    }
    fs.close();
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM(string(argv[3])+"/KeyFrameTrajectory.txt"); // this is a third grpc call

    return 0;
}

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
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
            vTimestamps.push_back(t);
        }
    }

    string strPrefixLeft = strPathToSequence + "/image_0/";

    const int nTimes = vTimestamps.size();
    vstrImageFilenames.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageFilenames[i] = strPrefixLeft + ss.str() + ".png";
    }
}
