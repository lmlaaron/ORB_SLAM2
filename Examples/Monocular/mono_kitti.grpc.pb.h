// Generated by the gRPC protobuf plugin.
// If you make any local change, they will be lost.
// source: mono_kitti.proto
// Original file comments:
// Copyright 2015, Google Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//     * Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above
// copyright notice, this list of conditions and the following disclaimer
// in the documentation and/or other materials provided with the
// distribution.
//     * Neither the name of Google Inc. nor the names of its
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
#ifndef GRPC_mono_5fkitti_2eproto__INCLUDED
#define GRPC_mono_5fkitti_2eproto__INCLUDED

#include "mono_kitti.pb.h"

#include <grpc++/impl/codegen/async_stream.h>
#include <grpc++/impl/codegen/async_unary_call.h>
#include <grpc++/impl/codegen/method_handler_impl.h>
#include <grpc++/impl/codegen/proto_utils.h>
#include <grpc++/impl/codegen/rpc_method.h>
#include <grpc++/impl/codegen/service_type.h>
#include <grpc++/impl/codegen/status.h>
#include <grpc++/impl/codegen/stub_options.h>
#include <grpc++/impl/codegen/sync_stream.h>

namespace grpc {
class CompletionQueue;
class Channel;
class RpcService;
class ServerCompletionQueue;
class ServerContext;
}  // namespace grpc

namespace mono_kitti {

class OrbSLAM GRPC_FINAL {
 public:
  class StubInterface {
   public:
    virtual ~StubInterface() {}
    virtual ::grpc::Status NewSLAM(::grpc::ClientContext* context, const ::mono_kitti::NewSLAMRequest& request, ::mono_kitti::NewSLAMReturn* response) = 0;
    std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::mono_kitti::NewSLAMReturn>> AsyncNewSLAM(::grpc::ClientContext* context, const ::mono_kitti::NewSLAMRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::mono_kitti::NewSLAMReturn>>(AsyncNewSLAMRaw(context, request, cq));
    }
    virtual ::grpc::Status TrackMonocular(::grpc::ClientContext* context, const ::mono_kitti::TrackMonocularRequest& request, ::mono_kitti::TrackMonocularReturn* response) = 0;
    std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::mono_kitti::TrackMonocularReturn>> AsyncTrackMonocular(::grpc::ClientContext* context, const ::mono_kitti::TrackMonocularRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::mono_kitti::TrackMonocularReturn>>(AsyncTrackMonocularRaw(context, request, cq));
    }
    virtual ::grpc::Status Shutdown(::grpc::ClientContext* context, const ::mono_kitti::ShutdownRequest& request, ::mono_kitti::ShutdownReturn* response) = 0;
    std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::mono_kitti::ShutdownReturn>> AsyncShutdown(::grpc::ClientContext* context, const ::mono_kitti::ShutdownRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::mono_kitti::ShutdownReturn>>(AsyncShutdownRaw(context, request, cq));
    }
    virtual ::grpc::Status SaveKeyFrameTrajectoryTUM(::grpc::ClientContext* context, const ::mono_kitti::SaveKeyFrameTrajectoryTUMRequest& request, ::mono_kitti::SaveKeyFrameTrajectoryTUMReturn* response) = 0;
    std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::mono_kitti::SaveKeyFrameTrajectoryTUMReturn>> AsyncSaveKeyFrameTrajectoryTUM(::grpc::ClientContext* context, const ::mono_kitti::SaveKeyFrameTrajectoryTUMRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::mono_kitti::SaveKeyFrameTrajectoryTUMReturn>>(AsyncSaveKeyFrameTrajectoryTUMRaw(context, request, cq));
    }
  private:
    virtual ::grpc::ClientAsyncResponseReaderInterface< ::mono_kitti::NewSLAMReturn>* AsyncNewSLAMRaw(::grpc::ClientContext* context, const ::mono_kitti::NewSLAMRequest& request, ::grpc::CompletionQueue* cq) = 0;
    virtual ::grpc::ClientAsyncResponseReaderInterface< ::mono_kitti::TrackMonocularReturn>* AsyncTrackMonocularRaw(::grpc::ClientContext* context, const ::mono_kitti::TrackMonocularRequest& request, ::grpc::CompletionQueue* cq) = 0;
    virtual ::grpc::ClientAsyncResponseReaderInterface< ::mono_kitti::ShutdownReturn>* AsyncShutdownRaw(::grpc::ClientContext* context, const ::mono_kitti::ShutdownRequest& request, ::grpc::CompletionQueue* cq) = 0;
    virtual ::grpc::ClientAsyncResponseReaderInterface< ::mono_kitti::SaveKeyFrameTrajectoryTUMReturn>* AsyncSaveKeyFrameTrajectoryTUMRaw(::grpc::ClientContext* context, const ::mono_kitti::SaveKeyFrameTrajectoryTUMRequest& request, ::grpc::CompletionQueue* cq) = 0;
  };
  class Stub GRPC_FINAL : public StubInterface {
   public:
    Stub(const std::shared_ptr< ::grpc::ChannelInterface>& channel);
    ::grpc::Status NewSLAM(::grpc::ClientContext* context, const ::mono_kitti::NewSLAMRequest& request, ::mono_kitti::NewSLAMReturn* response) GRPC_OVERRIDE;
    std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::mono_kitti::NewSLAMReturn>> AsyncNewSLAM(::grpc::ClientContext* context, const ::mono_kitti::NewSLAMRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::mono_kitti::NewSLAMReturn>>(AsyncNewSLAMRaw(context, request, cq));
    }
    ::grpc::Status TrackMonocular(::grpc::ClientContext* context, const ::mono_kitti::TrackMonocularRequest& request, ::mono_kitti::TrackMonocularReturn* response) GRPC_OVERRIDE;
    std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::mono_kitti::TrackMonocularReturn>> AsyncTrackMonocular(::grpc::ClientContext* context, const ::mono_kitti::TrackMonocularRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::mono_kitti::TrackMonocularReturn>>(AsyncTrackMonocularRaw(context, request, cq));
    }
    ::grpc::Status Shutdown(::grpc::ClientContext* context, const ::mono_kitti::ShutdownRequest& request, ::mono_kitti::ShutdownReturn* response) GRPC_OVERRIDE;
    std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::mono_kitti::ShutdownReturn>> AsyncShutdown(::grpc::ClientContext* context, const ::mono_kitti::ShutdownRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::mono_kitti::ShutdownReturn>>(AsyncShutdownRaw(context, request, cq));
    }
    ::grpc::Status SaveKeyFrameTrajectoryTUM(::grpc::ClientContext* context, const ::mono_kitti::SaveKeyFrameTrajectoryTUMRequest& request, ::mono_kitti::SaveKeyFrameTrajectoryTUMReturn* response) GRPC_OVERRIDE;
    std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::mono_kitti::SaveKeyFrameTrajectoryTUMReturn>> AsyncSaveKeyFrameTrajectoryTUM(::grpc::ClientContext* context, const ::mono_kitti::SaveKeyFrameTrajectoryTUMRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::mono_kitti::SaveKeyFrameTrajectoryTUMReturn>>(AsyncSaveKeyFrameTrajectoryTUMRaw(context, request, cq));
    }

   private:
    std::shared_ptr< ::grpc::ChannelInterface> channel_;
    ::grpc::ClientAsyncResponseReader< ::mono_kitti::NewSLAMReturn>* AsyncNewSLAMRaw(::grpc::ClientContext* context, const ::mono_kitti::NewSLAMRequest& request, ::grpc::CompletionQueue* cq) GRPC_OVERRIDE;
    ::grpc::ClientAsyncResponseReader< ::mono_kitti::TrackMonocularReturn>* AsyncTrackMonocularRaw(::grpc::ClientContext* context, const ::mono_kitti::TrackMonocularRequest& request, ::grpc::CompletionQueue* cq) GRPC_OVERRIDE;
    ::grpc::ClientAsyncResponseReader< ::mono_kitti::ShutdownReturn>* AsyncShutdownRaw(::grpc::ClientContext* context, const ::mono_kitti::ShutdownRequest& request, ::grpc::CompletionQueue* cq) GRPC_OVERRIDE;
    ::grpc::ClientAsyncResponseReader< ::mono_kitti::SaveKeyFrameTrajectoryTUMReturn>* AsyncSaveKeyFrameTrajectoryTUMRaw(::grpc::ClientContext* context, const ::mono_kitti::SaveKeyFrameTrajectoryTUMRequest& request, ::grpc::CompletionQueue* cq) GRPC_OVERRIDE;
    const ::grpc::RpcMethod rpcmethod_NewSLAM_;
    const ::grpc::RpcMethod rpcmethod_TrackMonocular_;
    const ::grpc::RpcMethod rpcmethod_Shutdown_;
    const ::grpc::RpcMethod rpcmethod_SaveKeyFrameTrajectoryTUM_;
  };
  static std::unique_ptr<Stub> NewStub(const std::shared_ptr< ::grpc::ChannelInterface>& channel, const ::grpc::StubOptions& options = ::grpc::StubOptions());

  class Service : public ::grpc::Service {
   public:
    Service();
    virtual ~Service();
    virtual ::grpc::Status NewSLAM(::grpc::ServerContext* context, const ::mono_kitti::NewSLAMRequest* request, ::mono_kitti::NewSLAMReturn* response);
    virtual ::grpc::Status TrackMonocular(::grpc::ServerContext* context, const ::mono_kitti::TrackMonocularRequest* request, ::mono_kitti::TrackMonocularReturn* response);
    virtual ::grpc::Status Shutdown(::grpc::ServerContext* context, const ::mono_kitti::ShutdownRequest* request, ::mono_kitti::ShutdownReturn* response);
    virtual ::grpc::Status SaveKeyFrameTrajectoryTUM(::grpc::ServerContext* context, const ::mono_kitti::SaveKeyFrameTrajectoryTUMRequest* request, ::mono_kitti::SaveKeyFrameTrajectoryTUMReturn* response);
  };
  template <class BaseClass>
  class WithAsyncMethod_NewSLAM : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service *service) {}
   public:
    WithAsyncMethod_NewSLAM() {
      ::grpc::Service::MarkMethodAsync(0);
    }
    ~WithAsyncMethod_NewSLAM() GRPC_OVERRIDE {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status NewSLAM(::grpc::ServerContext* context, const ::mono_kitti::NewSLAMRequest* request, ::mono_kitti::NewSLAMReturn* response) GRPC_FINAL GRPC_OVERRIDE {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    void RequestNewSLAM(::grpc::ServerContext* context, ::mono_kitti::NewSLAMRequest* request, ::grpc::ServerAsyncResponseWriter< ::mono_kitti::NewSLAMReturn>* response, ::grpc::CompletionQueue* new_call_cq, ::grpc::ServerCompletionQueue* notification_cq, void *tag) {
      ::grpc::Service::RequestAsyncUnary(0, context, request, response, new_call_cq, notification_cq, tag);
    }
  };
  template <class BaseClass>
  class WithAsyncMethod_TrackMonocular : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service *service) {}
   public:
    WithAsyncMethod_TrackMonocular() {
      ::grpc::Service::MarkMethodAsync(1);
    }
    ~WithAsyncMethod_TrackMonocular() GRPC_OVERRIDE {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status TrackMonocular(::grpc::ServerContext* context, const ::mono_kitti::TrackMonocularRequest* request, ::mono_kitti::TrackMonocularReturn* response) GRPC_FINAL GRPC_OVERRIDE {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    void RequestTrackMonocular(::grpc::ServerContext* context, ::mono_kitti::TrackMonocularRequest* request, ::grpc::ServerAsyncResponseWriter< ::mono_kitti::TrackMonocularReturn>* response, ::grpc::CompletionQueue* new_call_cq, ::grpc::ServerCompletionQueue* notification_cq, void *tag) {
      ::grpc::Service::RequestAsyncUnary(1, context, request, response, new_call_cq, notification_cq, tag);
    }
  };
  template <class BaseClass>
  class WithAsyncMethod_Shutdown : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service *service) {}
   public:
    WithAsyncMethod_Shutdown() {
      ::grpc::Service::MarkMethodAsync(2);
    }
    ~WithAsyncMethod_Shutdown() GRPC_OVERRIDE {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status Shutdown(::grpc::ServerContext* context, const ::mono_kitti::ShutdownRequest* request, ::mono_kitti::ShutdownReturn* response) GRPC_FINAL GRPC_OVERRIDE {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    void RequestShutdown(::grpc::ServerContext* context, ::mono_kitti::ShutdownRequest* request, ::grpc::ServerAsyncResponseWriter< ::mono_kitti::ShutdownReturn>* response, ::grpc::CompletionQueue* new_call_cq, ::grpc::ServerCompletionQueue* notification_cq, void *tag) {
      ::grpc::Service::RequestAsyncUnary(2, context, request, response, new_call_cq, notification_cq, tag);
    }
  };
  template <class BaseClass>
  class WithAsyncMethod_SaveKeyFrameTrajectoryTUM : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service *service) {}
   public:
    WithAsyncMethod_SaveKeyFrameTrajectoryTUM() {
      ::grpc::Service::MarkMethodAsync(3);
    }
    ~WithAsyncMethod_SaveKeyFrameTrajectoryTUM() GRPC_OVERRIDE {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status SaveKeyFrameTrajectoryTUM(::grpc::ServerContext* context, const ::mono_kitti::SaveKeyFrameTrajectoryTUMRequest* request, ::mono_kitti::SaveKeyFrameTrajectoryTUMReturn* response) GRPC_FINAL GRPC_OVERRIDE {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    void RequestSaveKeyFrameTrajectoryTUM(::grpc::ServerContext* context, ::mono_kitti::SaveKeyFrameTrajectoryTUMRequest* request, ::grpc::ServerAsyncResponseWriter< ::mono_kitti::SaveKeyFrameTrajectoryTUMReturn>* response, ::grpc::CompletionQueue* new_call_cq, ::grpc::ServerCompletionQueue* notification_cq, void *tag) {
      ::grpc::Service::RequestAsyncUnary(3, context, request, response, new_call_cq, notification_cq, tag);
    }
  };
  typedef WithAsyncMethod_NewSLAM<WithAsyncMethod_TrackMonocular<WithAsyncMethod_Shutdown<WithAsyncMethod_SaveKeyFrameTrajectoryTUM<Service > > > > AsyncService;
  template <class BaseClass>
  class WithGenericMethod_NewSLAM : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service *service) {}
   public:
    WithGenericMethod_NewSLAM() {
      ::grpc::Service::MarkMethodGeneric(0);
    }
    ~WithGenericMethod_NewSLAM() GRPC_OVERRIDE {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status NewSLAM(::grpc::ServerContext* context, const ::mono_kitti::NewSLAMRequest* request, ::mono_kitti::NewSLAMReturn* response) GRPC_FINAL GRPC_OVERRIDE {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
  };
  template <class BaseClass>
  class WithGenericMethod_TrackMonocular : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service *service) {}
   public:
    WithGenericMethod_TrackMonocular() {
      ::grpc::Service::MarkMethodGeneric(1);
    }
    ~WithGenericMethod_TrackMonocular() GRPC_OVERRIDE {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status TrackMonocular(::grpc::ServerContext* context, const ::mono_kitti::TrackMonocularRequest* request, ::mono_kitti::TrackMonocularReturn* response) GRPC_FINAL GRPC_OVERRIDE {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
  };
  template <class BaseClass>
  class WithGenericMethod_Shutdown : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service *service) {}
   public:
    WithGenericMethod_Shutdown() {
      ::grpc::Service::MarkMethodGeneric(2);
    }
    ~WithGenericMethod_Shutdown() GRPC_OVERRIDE {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status Shutdown(::grpc::ServerContext* context, const ::mono_kitti::ShutdownRequest* request, ::mono_kitti::ShutdownReturn* response) GRPC_FINAL GRPC_OVERRIDE {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
  };
  template <class BaseClass>
  class WithGenericMethod_SaveKeyFrameTrajectoryTUM : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service *service) {}
   public:
    WithGenericMethod_SaveKeyFrameTrajectoryTUM() {
      ::grpc::Service::MarkMethodGeneric(3);
    }
    ~WithGenericMethod_SaveKeyFrameTrajectoryTUM() GRPC_OVERRIDE {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status SaveKeyFrameTrajectoryTUM(::grpc::ServerContext* context, const ::mono_kitti::SaveKeyFrameTrajectoryTUMRequest* request, ::mono_kitti::SaveKeyFrameTrajectoryTUMReturn* response) GRPC_FINAL GRPC_OVERRIDE {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
  };
  template <class BaseClass>
  class WithStreamedUnaryMethod_NewSLAM : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service *service) {}
   public:
    WithStreamedUnaryMethod_NewSLAM() {
      ::grpc::Service::MarkMethodStreamedUnary(0,
        new ::grpc::StreamedUnaryHandler< ::mono_kitti::NewSLAMRequest, ::mono_kitti::NewSLAMReturn>(std::bind(&WithStreamedUnaryMethod_NewSLAM<BaseClass>::StreamedNewSLAM, this, std::placeholders::_1, std::placeholders::_2)));
    }
    ~WithStreamedUnaryMethod_NewSLAM() GRPC_OVERRIDE {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable regular version of this method
    ::grpc::Status NewSLAM(::grpc::ServerContext* context, const ::mono_kitti::NewSLAMRequest* request, ::mono_kitti::NewSLAMReturn* response) GRPC_FINAL GRPC_OVERRIDE {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    // replace default version of method with streamed unary
    virtual ::grpc::Status StreamedNewSLAM(::grpc::ServerContext* context, ::grpc::ServerUnaryStreamer< ::mono_kitti::NewSLAMRequest,::mono_kitti::NewSLAMReturn>* server_unary_streamer) = 0;
  };
  template <class BaseClass>
  class WithStreamedUnaryMethod_TrackMonocular : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service *service) {}
   public:
    WithStreamedUnaryMethod_TrackMonocular() {
      ::grpc::Service::MarkMethodStreamedUnary(1,
        new ::grpc::StreamedUnaryHandler< ::mono_kitti::TrackMonocularRequest, ::mono_kitti::TrackMonocularReturn>(std::bind(&WithStreamedUnaryMethod_TrackMonocular<BaseClass>::StreamedTrackMonocular, this, std::placeholders::_1, std::placeholders::_2)));
    }
    ~WithStreamedUnaryMethod_TrackMonocular() GRPC_OVERRIDE {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable regular version of this method
    ::grpc::Status TrackMonocular(::grpc::ServerContext* context, const ::mono_kitti::TrackMonocularRequest* request, ::mono_kitti::TrackMonocularReturn* response) GRPC_FINAL GRPC_OVERRIDE {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    // replace default version of method with streamed unary
    virtual ::grpc::Status StreamedTrackMonocular(::grpc::ServerContext* context, ::grpc::ServerUnaryStreamer< ::mono_kitti::TrackMonocularRequest,::mono_kitti::TrackMonocularReturn>* server_unary_streamer) = 0;
  };
  template <class BaseClass>
  class WithStreamedUnaryMethod_Shutdown : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service *service) {}
   public:
    WithStreamedUnaryMethod_Shutdown() {
      ::grpc::Service::MarkMethodStreamedUnary(2,
        new ::grpc::StreamedUnaryHandler< ::mono_kitti::ShutdownRequest, ::mono_kitti::ShutdownReturn>(std::bind(&WithStreamedUnaryMethod_Shutdown<BaseClass>::StreamedShutdown, this, std::placeholders::_1, std::placeholders::_2)));
    }
    ~WithStreamedUnaryMethod_Shutdown() GRPC_OVERRIDE {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable regular version of this method
    ::grpc::Status Shutdown(::grpc::ServerContext* context, const ::mono_kitti::ShutdownRequest* request, ::mono_kitti::ShutdownReturn* response) GRPC_FINAL GRPC_OVERRIDE {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    // replace default version of method with streamed unary
    virtual ::grpc::Status StreamedShutdown(::grpc::ServerContext* context, ::grpc::ServerUnaryStreamer< ::mono_kitti::ShutdownRequest,::mono_kitti::ShutdownReturn>* server_unary_streamer) = 0;
  };
  template <class BaseClass>
  class WithStreamedUnaryMethod_SaveKeyFrameTrajectoryTUM : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service *service) {}
   public:
    WithStreamedUnaryMethod_SaveKeyFrameTrajectoryTUM() {
      ::grpc::Service::MarkMethodStreamedUnary(3,
        new ::grpc::StreamedUnaryHandler< ::mono_kitti::SaveKeyFrameTrajectoryTUMRequest, ::mono_kitti::SaveKeyFrameTrajectoryTUMReturn>(std::bind(&WithStreamedUnaryMethod_SaveKeyFrameTrajectoryTUM<BaseClass>::StreamedSaveKeyFrameTrajectoryTUM, this, std::placeholders::_1, std::placeholders::_2)));
    }
    ~WithStreamedUnaryMethod_SaveKeyFrameTrajectoryTUM() GRPC_OVERRIDE {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable regular version of this method
    ::grpc::Status SaveKeyFrameTrajectoryTUM(::grpc::ServerContext* context, const ::mono_kitti::SaveKeyFrameTrajectoryTUMRequest* request, ::mono_kitti::SaveKeyFrameTrajectoryTUMReturn* response) GRPC_FINAL GRPC_OVERRIDE {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    // replace default version of method with streamed unary
    virtual ::grpc::Status StreamedSaveKeyFrameTrajectoryTUM(::grpc::ServerContext* context, ::grpc::ServerUnaryStreamer< ::mono_kitti::SaveKeyFrameTrajectoryTUMRequest,::mono_kitti::SaveKeyFrameTrajectoryTUMReturn>* server_unary_streamer) = 0;
  };
  typedef WithStreamedUnaryMethod_NewSLAM<WithStreamedUnaryMethod_TrackMonocular<WithStreamedUnaryMethod_Shutdown<WithStreamedUnaryMethod_SaveKeyFrameTrajectoryTUM<Service > > > > StreamedUnaryService;
};

}  // namespace mono_kitti


#endif  // GRPC_mono_5fkitti_2eproto__INCLUDED
