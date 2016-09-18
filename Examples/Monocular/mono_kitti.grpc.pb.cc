// Generated by the gRPC protobuf plugin.
// If you make any local change, they will be lost.
// source: mono_kitti.proto

#include "mono_kitti.pb.h"
#include "mono_kitti.grpc.pb.h"

#include <grpc++/impl/codegen/async_stream.h>
#include <grpc++/impl/codegen/async_unary_call.h>
#include <grpc++/impl/codegen/channel_interface.h>
#include <grpc++/impl/codegen/client_unary_call.h>
#include <grpc++/impl/codegen/method_handler_impl.h>
#include <grpc++/impl/codegen/rpc_service_method.h>
#include <grpc++/impl/codegen/service_type.h>
#include <grpc++/impl/codegen/sync_stream.h>
namespace mono_kitti {

static const char* OrbSLAM_method_names[] = {
  "/mono_kitti.OrbSLAM/NewSLAM",
  "/mono_kitti.OrbSLAM/TrackMonocular",
  "/mono_kitti.OrbSLAM/Shutdown",
  "/mono_kitti.OrbSLAM/SaveKeyFrameTrajectoryTUM",
};

std::unique_ptr< OrbSLAM::Stub> OrbSLAM::NewStub(const std::shared_ptr< ::grpc::ChannelInterface>& channel, const ::grpc::StubOptions& options) {
  std::unique_ptr< OrbSLAM::Stub> stub(new OrbSLAM::Stub(channel));
  return stub;
}

OrbSLAM::Stub::Stub(const std::shared_ptr< ::grpc::ChannelInterface>& channel)
  : channel_(channel), rpcmethod_NewSLAM_(OrbSLAM_method_names[0], ::grpc::RpcMethod::NORMAL_RPC, channel)
  , rpcmethod_TrackMonocular_(OrbSLAM_method_names[1], ::grpc::RpcMethod::NORMAL_RPC, channel)
  , rpcmethod_Shutdown_(OrbSLAM_method_names[2], ::grpc::RpcMethod::NORMAL_RPC, channel)
  , rpcmethod_SaveKeyFrameTrajectoryTUM_(OrbSLAM_method_names[3], ::grpc::RpcMethod::NORMAL_RPC, channel)
  {}

::grpc::Status OrbSLAM::Stub::NewSLAM(::grpc::ClientContext* context, const ::mono_kitti::NewSLAMRequest& request, ::mono_kitti::NewSLAMReturn* response) {
  return ::grpc::BlockingUnaryCall(channel_.get(), rpcmethod_NewSLAM_, context, request, response);
}

::grpc::ClientAsyncResponseReader< ::mono_kitti::NewSLAMReturn>* OrbSLAM::Stub::AsyncNewSLAMRaw(::grpc::ClientContext* context, const ::mono_kitti::NewSLAMRequest& request, ::grpc::CompletionQueue* cq) {
  return new ::grpc::ClientAsyncResponseReader< ::mono_kitti::NewSLAMReturn>(channel_.get(), cq, rpcmethod_NewSLAM_, context, request);
}

::grpc::Status OrbSLAM::Stub::TrackMonocular(::grpc::ClientContext* context, const ::mono_kitti::TrackMonocularRequest& request, ::mono_kitti::TrackMonocularReturn* response) {
  return ::grpc::BlockingUnaryCall(channel_.get(), rpcmethod_TrackMonocular_, context, request, response);
}

::grpc::ClientAsyncResponseReader< ::mono_kitti::TrackMonocularReturn>* OrbSLAM::Stub::AsyncTrackMonocularRaw(::grpc::ClientContext* context, const ::mono_kitti::TrackMonocularRequest& request, ::grpc::CompletionQueue* cq) {
  return new ::grpc::ClientAsyncResponseReader< ::mono_kitti::TrackMonocularReturn>(channel_.get(), cq, rpcmethod_TrackMonocular_, context, request);
}

::grpc::Status OrbSLAM::Stub::Shutdown(::grpc::ClientContext* context, const ::mono_kitti::ShutdownRequest& request, ::mono_kitti::ShutdownReturn* response) {
  return ::grpc::BlockingUnaryCall(channel_.get(), rpcmethod_Shutdown_, context, request, response);
}

::grpc::ClientAsyncResponseReader< ::mono_kitti::ShutdownReturn>* OrbSLAM::Stub::AsyncShutdownRaw(::grpc::ClientContext* context, const ::mono_kitti::ShutdownRequest& request, ::grpc::CompletionQueue* cq) {
  return new ::grpc::ClientAsyncResponseReader< ::mono_kitti::ShutdownReturn>(channel_.get(), cq, rpcmethod_Shutdown_, context, request);
}

::grpc::Status OrbSLAM::Stub::SaveKeyFrameTrajectoryTUM(::grpc::ClientContext* context, const ::mono_kitti::SaveKeyFrameTrajectoryTUMRequest& request, ::mono_kitti::SaveKeyFrameTrajectoryTUMReturn* response) {
  return ::grpc::BlockingUnaryCall(channel_.get(), rpcmethod_SaveKeyFrameTrajectoryTUM_, context, request, response);
}

::grpc::ClientAsyncResponseReader< ::mono_kitti::SaveKeyFrameTrajectoryTUMReturn>* OrbSLAM::Stub::AsyncSaveKeyFrameTrajectoryTUMRaw(::grpc::ClientContext* context, const ::mono_kitti::SaveKeyFrameTrajectoryTUMRequest& request, ::grpc::CompletionQueue* cq) {
  return new ::grpc::ClientAsyncResponseReader< ::mono_kitti::SaveKeyFrameTrajectoryTUMReturn>(channel_.get(), cq, rpcmethod_SaveKeyFrameTrajectoryTUM_, context, request);
}

OrbSLAM::Service::Service() {
  (void)OrbSLAM_method_names;
  AddMethod(new ::grpc::RpcServiceMethod(
      OrbSLAM_method_names[0],
      ::grpc::RpcMethod::NORMAL_RPC,
      new ::grpc::RpcMethodHandler< OrbSLAM::Service, ::mono_kitti::NewSLAMRequest, ::mono_kitti::NewSLAMReturn>(
          std::mem_fn(&OrbSLAM::Service::NewSLAM), this)));
  AddMethod(new ::grpc::RpcServiceMethod(
      OrbSLAM_method_names[1],
      ::grpc::RpcMethod::NORMAL_RPC,
      new ::grpc::RpcMethodHandler< OrbSLAM::Service, ::mono_kitti::TrackMonocularRequest, ::mono_kitti::TrackMonocularReturn>(
          std::mem_fn(&OrbSLAM::Service::TrackMonocular), this)));
  AddMethod(new ::grpc::RpcServiceMethod(
      OrbSLAM_method_names[2],
      ::grpc::RpcMethod::NORMAL_RPC,
      new ::grpc::RpcMethodHandler< OrbSLAM::Service, ::mono_kitti::ShutdownRequest, ::mono_kitti::ShutdownReturn>(
          std::mem_fn(&OrbSLAM::Service::Shutdown), this)));
  AddMethod(new ::grpc::RpcServiceMethod(
      OrbSLAM_method_names[3],
      ::grpc::RpcMethod::NORMAL_RPC,
      new ::grpc::RpcMethodHandler< OrbSLAM::Service, ::mono_kitti::SaveKeyFrameTrajectoryTUMRequest, ::mono_kitti::SaveKeyFrameTrajectoryTUMReturn>(
          std::mem_fn(&OrbSLAM::Service::SaveKeyFrameTrajectoryTUM), this)));
}

OrbSLAM::Service::~Service() {
}

::grpc::Status OrbSLAM::Service::NewSLAM(::grpc::ServerContext* context, const ::mono_kitti::NewSLAMRequest* request, ::mono_kitti::NewSLAMReturn* response) {
  (void) context;
  (void) request;
  (void) response;
  return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
}

::grpc::Status OrbSLAM::Service::TrackMonocular(::grpc::ServerContext* context, const ::mono_kitti::TrackMonocularRequest* request, ::mono_kitti::TrackMonocularReturn* response) {
  (void) context;
  (void) request;
  (void) response;
  return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
}

::grpc::Status OrbSLAM::Service::Shutdown(::grpc::ServerContext* context, const ::mono_kitti::ShutdownRequest* request, ::mono_kitti::ShutdownReturn* response) {
  (void) context;
  (void) request;
  (void) response;
  return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
}

::grpc::Status OrbSLAM::Service::SaveKeyFrameTrajectoryTUM(::grpc::ServerContext* context, const ::mono_kitti::SaveKeyFrameTrajectoryTUMRequest* request, ::mono_kitti::SaveKeyFrameTrajectoryTUMReturn* response) {
  (void) context;
  (void) request;
  (void) response;
  return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
}


}  // namespace mono_kitti
