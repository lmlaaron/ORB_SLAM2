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

#include<opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <boost/serialization/split_free.hpp>
#include <boost/serialization/vector.hpp>
#include <grpc/grpc.h>
#include <grpc++/server.h>
#include <grpc++/server_builder.h>
#include <grpc++/server_context.h>
#include <grpc++/security/server_credentials.h>

#include"System.h"
#include "mono_kitti.grpc.pb.h"

using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;
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

class OrbSLAMServiceImpl final : public OrbSLAM::Service {
 private:
  ORB_SLAM2::System* SLAM;
 public:
  explicit OrbSLAMServiceImpl() {
	SLAM = NULL;
  }
  Status NewSLAM(ServerContext* context,const NewSLAMRequest* request, NewSLAMReturn* reply) override {
	//char *strsettingfile_cstr = new char[300];
	//char *strvocfile_cstr = new char[300];
	//strcpy(str)
	if ( request->sensor() == mono_kitti::NewSLAMRequest_ESensor::NewSLAMRequest_ESensor_MONOCULAR   ) {
		SLAM = new ORB_SLAM2::System((request->strvocfile()), (request->strsettingfile()), ORB_SLAM2::System::eSensor::MONOCULAR, request->buseviewer());
	} else if ( request->sensor() == mono_kitti::NewSLAMRequest_ESensor::NewSLAMRequest_ESensor_RGBD ) {
		SLAM = new ORB_SLAM2::System((request->strvocfile()), (request->strsettingfile()), ORB_SLAM2::System::eSensor::RGBD, request->buseviewer());
	} else if ( request->sensor() == mono_kitti::NewSLAMRequest_ESensor::NewSLAMRequest_ESensor_STEREO ) {
		SLAM = new ORB_SLAM2::System((request->strvocfile()), (request->strsettingfile()), ORB_SLAM2::System::eSensor::STEREO, request->buseviewer());
	}
	reply->set_success(true);
	return Status::OK;
	}; 

  Status TrackMonocular(ServerContext* context, const TrackMonocularRequest* request, TrackMonocularReturn* reply) override {
	// de-serialize cv::mat
	std::string im = request->im();
	//std::vector<uchar> data(im.ptr(), im.ptr() + request->im_height() * request->im_width() * request->channels());
	std::vector<uchar> im_data(im.begin(), im.end());
	char *data_send;
	data_send = new char[ im_data.size()];	
	std::copy(im_data.begin(), im_data.end(), data_send);
	//strcpy(data_send, data_image.c_str());
	//data_send = new char[request->im_width() * request->im_height() *request->im_channel() ];	
	//strcpy(data_send, im.c_str());
	cv::Mat image(request->im_height(), request->im_width(), request->im_type(), data_send);
       	//cv::imshow("ORB-SLAM2: Current Frame",image); // for debug
        //cv::waitKey(1e3/30);

	cv::Mat ret = SLAM->TrackMonocular(image, request->timestamp()); 
	// serialize cv::mat
	cv::Size size = ret.size();
	std::vector<uchar> data(ret.ptr(), ret.ptr() + size.width * size.height* ret.channels());
	std::string ret_image(data.begin(), data.end());	
	reply->set_im(ret_image);
	reply->set_im_width(size.width);
	reply->set_im_height(size.height);
	reply->set_im_channel(ret.channels());
	reply->set_im_type(ret.type());
		
  	return Status::OK;
  }; 
  Status Shutdown(ServerContext* context, const ShutdownRequest* request, ShutdownReturn* reply) override {
	SLAM->Shutdown();
	reply->set_success(true);
  	return Status::OK;
  };

  Status SaveKeyFrameTrajectoryTUM(ServerContext* context, const SaveKeyFrameTrajectoryTUMRequest* request, SaveKeyFrameTrajectoryTUMReturn* reply) override {
	std::string filename = request->filename();
  	SLAM->SaveKeyFrameTrajectoryTUM(filename);
	reply->set_success(true);
	return Status::OK;
  };
};


void RunServer() {
  std::string server_address("0.0.0.0:50051");
  OrbSLAMServiceImpl service;

  ServerBuilder builder;
  // Listen on the given address without any authentication mechanism.
  builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
  // Register "service" as the instance through which we'll communicate with
  // clients. In this case it corresponds to an *synchronous* service.
  builder.RegisterService(&service);
  // Finally assemble the server.
  std::unique_ptr<Server> server(builder.BuildAndStart());
  std::cout << "Server listening on " << server_address << std::endl;

  // Wait for the server to shutdown. Note that some other thread must be
  // responsible for shutting down the server for this call to ever return.
  server->Wait();
}

int main(int argc, char ** argv) {
  RunServer();
  return 0;
}

