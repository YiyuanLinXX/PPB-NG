#include "ppbng_thermal/spinnaker_a6701_backend.hpp"

#include <Spinnaker.h>
#include <SpinGenApi/SpinnakerGenApi.h>
#include <cstring>
#include <exception>
#include <functional>
#include <limits>
#include <utility>

namespace ppbng_thermal
{
using namespace Spinnaker;
using namespace Spinnaker::GenApi;

namespace
{
Status preflight(std::chrono::milliseconds timeout,const StopToken&stop)
{if(stop.stop_requested())return{ErrorCode::cancelled,"stop requested"};if(timeout.count()<=0)return{ErrorCode::timeout,"finite positive timeout required"};return{};}
std::string exception_detail(const Exception&e){return e.what()?e.what():"Spinnaker exception";}
bool read_string(INodeMap&map,const char*name,std::string&value)
{CStringPtr node=map.GetNode(name);if(!IsReadable(node))return false;value=node->GetValue().c_str();return true;}
bool read_enum(INodeMap&map,const char*name,std::string&value)
{CEnumerationPtr node=map.GetNode(name);if(!IsReadable(node))return false;CEnumEntryPtr entry=node->GetCurrentEntry();if(!IsReadable(entry))return false;value=entry->GetSymbolic().c_str();return true;}
bool read_integer(INodeMap&map,const char*name,std::size_t&value)
{CIntegerPtr node=map.GetNode(name);if(!IsReadable(node))return false;auto raw=node->GetValue();if(raw<0)return false;value=static_cast<std::size_t>(raw);return true;}
bool read_bool(INodeMap&map,const char*name,bool&value)
{CBooleanPtr node=map.GetNode(name);if(!IsReadable(node))return false;value=node->GetValue();return true;}
class ImageReleaseGuard
{
public:
  explicit ImageReleaseGuard(ImagePtr image):image_(std::move(image)){}
  ~ImageReleaseGuard(){if(armed_&&image_){try{image_->Release();}catch(...){}}}
  ImageReleaseGuard(const ImageReleaseGuard&)=delete;
  ImageReleaseGuard&operator=(const ImageReleaseGuard&)=delete;
  void dismiss()noexcept{armed_=false;}
private:
  ImagePtr image_;
  bool armed_{true};
};
Status set_enum_transactional(INodeMap&map,const char*name,const std::string&target,
  std::vector<std::function<void()>>&rollback)
{
  CEnumerationPtr node=map.GetNode(name);if(!IsReadable(node)||!IsWritable(node))return{ErrorCode::invalid_configuration,std::string(name)+" missing/read-only"};
  CEnumEntryPtr old_entry=node->GetCurrentEntry();if(!IsReadable(old_entry))return{ErrorCode::invalid_configuration,std::string(name)+" current entry unreadable"};
  const auto old_value=old_entry->GetValue();CEnumEntryPtr desired=node->GetEntryByName(target.c_str());if(!IsReadable(desired))return{ErrorCode::invalid_configuration,std::string(name)+" value unavailable: "+target};
  node->SetIntValue(desired->GetValue());rollback.emplace_back([node,old_value](){try{node->SetIntValue(old_value);}catch(...){}});
  std::string actual;if(!read_enum(map,name,actual)||actual!=target)return{ErrorCode::readback_mismatch,std::string(name)+" readback mismatch"};return{};
}
Status set_integer_transactional(INodeMap&map,const char*name,std::size_t target,
  std::vector<std::function<void()>>&rollback)
{
  CIntegerPtr node=map.GetNode(name);if(!IsReadable(node)||!IsWritable(node))return{ErrorCode::invalid_configuration,std::string(name)+" missing/read-only"};auto old=node->GetValue();if(target>static_cast<std::size_t>((std::numeric_limits<int64_t>::max)()))return{ErrorCode::invalid_configuration,std::string(name)+" overflow"};node->SetValue(static_cast<int64_t>(target));rollback.emplace_back([node,old](){try{node->SetValue(old);}catch(...){}});std::size_t actual=0;if(!read_integer(map,name,actual)||actual!=target)return{ErrorCode::readback_mismatch,std::string(name)+" readback mismatch"};return{};
}
}

struct SpinnakerA6701Backend::Impl
{
  explicit Impl(SpinnakerA6701Identity i):identity(std::move(i)){}
  SpinnakerA6701Identity identity;SystemPtr system;CameraPtr camera;LifecycleState state{LifecycleState::idle};
  ThermalConfiguration requested{};std::uint64_t segment{0};
  void release_camera()noexcept
  {
    if(camera&&camera->IsInitialized()){
      // EndAcquisition is the normal path.  This device-side stop is an
      // idempotent safety net for partial starts and exception paths where
      // the local lifecycle state may not have advanced yet.
      try{
        CCommandPtr stop=camera->GetNodeMap().GetNode("AcquisitionStop");
        if(IsWritable(stop))stop->Execute();
      }catch(...){}
      try{camera->DeInit();}catch(...){}
    }
    camera=nullptr;
  }
  void release_all()noexcept{release_camera();try{if(system)system->ReleaseInstance();}catch(...){}system=nullptr;}
};

SpinnakerA6701Backend::SpinnakerA6701Backend(SpinnakerA6701Identity i):impl_(std::make_unique<Impl>(std::move(i))){}
SpinnakerA6701Backend::~SpinnakerA6701Backend(){stop(std::chrono::milliseconds(100));impl_->release_all();}
LifecycleState SpinnakerA6701Backend::state()const noexcept{return impl_->state;}

Status SpinnakerA6701Backend::discover(std::chrono::milliseconds timeout,const StopToken&stop,std::vector<std::string>&ids)
{
  auto s=preflight(timeout,stop);if(!s.ok())return s;if(impl_->identity.device_id.empty())return{ErrorCode::invalid_configuration,"configured device identity is empty"};
  try{if(!impl_->system)impl_->system=System::GetInstance();CameraList cameras=impl_->system->GetCameras();CameraPtr match;for(unsigned int i=0;i<cameras.GetSize();++i){if(stop.stop_requested()){cameras.Clear();return{ErrorCode::cancelled,"stop requested"};}auto candidate=cameras.GetByIndex(i);std::string id;if(read_string(candidate->GetTLDeviceNodeMap(),"DeviceID",id)&&id==impl_->identity.device_id){if(match){cameras.Clear();return{ErrorCode::invalid_configuration,"configured identity is not unique"};}match=candidate;}}
    cameras.Clear();if(!match)return{ErrorCode::not_found,"configured A6701 identity not found"};impl_->camera=match;ids={impl_->identity.device_id};impl_->state=LifecycleState::discovered;return{};
  }catch(const Exception&e){return{ErrorCode::io_error,exception_detail(e)};}
}

Status SpinnakerA6701Backend::open(const std::string&id,std::chrono::milliseconds timeout,const StopToken&stop)
{
  auto s=preflight(timeout,stop);if(!s.ok())return s;if(impl_->state!=LifecycleState::discovered||id!=impl_->identity.device_id||!impl_->camera)return{ErrorCode::invalid_state,"exact configured device must be discovered before open"};
  try{impl_->camera->Init();auto&map=impl_->camera->GetNodeMap();std::string model;if(!read_string(map,"CameraModel",model)||model!=impl_->identity.expected_model){impl_->release_camera();return{ErrorCode::invalid_configuration,"CameraModel is missing or is not A6701"};}std::string access;if(!read_enum(map,"GevCCP",access)||access!="ControlAccess"){impl_->release_camera();return{ErrorCode::access_denied,"GevCCP ControlAccess required"};}impl_->state=LifecycleState::open;return{};}catch(const Exception&e){impl_->release_camera();return{ErrorCode::io_error,exception_detail(e)};}
}

Status SpinnakerA6701Backend::configure(const ThermalConfiguration&c,std::chrono::milliseconds timeout,const StopToken&stop)
{
  auto s=preflight(timeout,stop);if(!s.ok())return s;if(impl_->state!=LifecycleState::open)return{ErrorCode::invalid_state,"open before configure"};
  std::vector<std::function<void()>> rollback;auto undo=[&](){for(auto i=rollback.rbegin();i!=rollback.rend();++i)(*i)();};
  try{auto&map=impl_->camera->GetNodeMap();
    for(auto step:{std::pair<const char*,std::size_t>{"Width",c.width},{"Height",c.transport_height}}){s=set_integer_transactional(map,step.first,step.second,rollback);if(!s.ok()){undo();return s;}}
    const std::pair<const char*,const std::string*> enums[]={{"PixelFormat",&c.pixel_format},{"IRFormat",&c.ir_format},{"FrameSyncSource",&c.frame_sync_source},{"FrameSyncMode",&c.frame_sync_mode},{"FrameSyncPolarity",&c.frame_sync_polarity}};
    for(const auto&step:enums){s=set_enum_transactional(map,step.first,*step.second,rollback);if(!s.ok()){undo();return s;}}
    auto&stream=impl_->camera->GetTLStreamNodeMap();CBooleanPtr resend=stream.GetNode("StreamPacketResendEnable");CEnumerationPtr handling=stream.GetNode("StreamBufferHandlingMode");if(!IsReadable(resend)||!IsReadable(handling)){undo();return{ErrorCode::invalid_configuration,"stream resend/buffer nodes unreadable"};}
    bool correction=false;if(!read_bool(map,"CorrectionDigitalEnabled",correction)){undo();return{ErrorCode::invalid_configuration,"CorrectionDigitalEnabled unreadable"};}
    const auto previous_requested=impl_->requested;impl_->requested=c;ThermalReadback r;s=readback(timeout,stop,r);if(!s.ok()||configuration_hash(c)!=configuration_hash(r)){impl_->requested=previous_requested;undo();return s.ok()?Status{ErrorCode::readback_mismatch,"transaction readback hash mismatch"}:s;}s=validate_a6701_contract(r);if(!s.ok()){impl_->requested=previous_requested;undo();return s;}impl_->state=LifecycleState::configured;return{};
  }catch(const Exception&e){undo();return{ErrorCode::io_error,exception_detail(e)};}
}

Status SpinnakerA6701Backend::readback(std::chrono::milliseconds timeout,const StopToken&stop,ThermalReadback&r)
{
  auto s=preflight(timeout,stop);if(!s.ok())return s;if(!impl_->camera||!impl_->camera->IsInitialized())return{ErrorCode::invalid_state,"camera not initialized"};
  try{auto&map=impl_->camera->GetNodeMap();if(!read_integer(map,"Width",r.width)||!read_integer(map,"Height",r.transport_height)||!read_integer(map,"PayloadSize",r.payload_bytes)||!read_enum(map,"PixelFormat",r.pixel_format)||!read_enum(map,"IRFormat",r.ir_format)||!read_enum(map,"FrameSyncSource",r.frame_sync_source)||!read_enum(map,"FrameSyncMode",r.frame_sync_mode)||!read_enum(map,"FrameSyncPolarity",r.frame_sync_polarity)||!read_bool(map,"Ready",r.ready)||!read_bool(map,"FPACold",r.fpa_cold))return{ErrorCode::invalid_configuration,"required A6701 readback node unavailable"};r.image_height=512;r.row_stride_bytes=1280;r.require_ready=impl_->requested.require_ready;r.require_fpa_cold=impl_->requested.require_fpa_cold;std::string access;r.access=read_enum(map,"GevCCP",access)&&access=="ControlAccess"?AccessMode::control:AccessMode::read_only;r.frame_sync_evidence=FrameSyncEvidence::unverified;return{};}catch(const Exception&e){return{ErrorCode::io_error,exception_detail(e)};}
}

Status SpinnakerA6701Backend::arm(std::chrono::milliseconds timeout,const StopToken&stop)
{auto s=preflight(timeout,stop);if(!s.ok())return s;if(impl_->state!=LifecycleState::configured)return{ErrorCode::invalid_state,"configure before arm"};ThermalReadback r;s=readback(timeout,stop,r);if(!s.ok())return s;s=validate_a6701_contract(r);if(!s.ok())return s;if(configuration_hash(impl_->requested)!=configuration_hash(r))return{ErrorCode::readback_mismatch,"pre-arm readback changed"};try{impl_->camera->BeginAcquisition();impl_->state=LifecycleState::armed;return{};}catch(const Exception&e){return{ErrorCode::io_error,exception_detail(e)};}}

FrameResult SpinnakerA6701Backend::next_frame(std::chrono::milliseconds timeout,const StopToken&stop)
{
  auto s=preflight(timeout,stop);if(!s.ok())return{s,{}};if(impl_->state!=LifecycleState::armed&&impl_->state!=LifecycleState::streaming)return{{ErrorCode::invalid_state,"arm before stream"},{}};
  try{
    auto image=impl_->camera->GetNextImage(static_cast<std::uint64_t>(timeout.count()));
    if(!image)return{{ErrorCode::io_error,"Spinnaker returned a null image"},{}};
    const auto host_receive_monotonic=std::chrono::steady_clock::now();
    ImageReleaseGuard release_guard(image);
    if(stop.stop_requested())return{{ErrorCode::cancelled,"stop requested"},{}};
    impl_->state=LifecycleState::streaming;
    auto storage=std::make_shared<std::vector<std::byte>>(image->GetBufferSize());
    std::memcpy(storage->data(),image->GetData(),storage->size());
    const bool geometry=image->GetWidth()==640&&image->GetHeight()==513&&
      image->GetStride()==1280&&storage->size()==656640;
    const bool complete=!image->IsIncomplete();
    FrameInfo info{image->GetFrameID(),impl_->segment,storage->size(),complete&&geometry,
      image->GetTimeStamp(),host_receive_monotonic};
    FrameLease lease(info,storage,[image](){try{image->Release();}catch(...){}});
    release_guard.dismiss();
    if(!complete)return{{ErrorCode::incomplete_frame,
      Image::GetImageStatusDescription(image->GetImageStatus())},std::move(lease)};
    if(!geometry)return{{ErrorCode::invalid_configuration,
      "frame geometry/payload changed"},std::move(lease)};
    return{{},std::move(lease)};
  }catch(const Exception&e){
    if(e.GetError()==SPINNAKER_ERR_TIMEOUT)return{{ErrorCode::timeout,exception_detail(e)},{}};
    return{{ErrorCode::io_error,exception_detail(e)},{}};
  }catch(const std::exception&e){return{{ErrorCode::io_error,e.what()},{}};}
}

Status SpinnakerA6701Backend::recover(std::chrono::milliseconds timeout,const StopToken&stop)
{auto s=preflight(timeout,stop);if(!s.ok())return s;this->stop(timeout);impl_->release_camera();++impl_->segment;impl_->state=LifecycleState::idle;std::vector<std::string>ids;s=discover(timeout,stop,ids);if(!s.ok())return s;return open(ids.front(),timeout,stop);}
Status SpinnakerA6701Backend::stop(std::chrono::milliseconds timeout)noexcept
{if(timeout.count()<=0)return{ErrorCode::timeout,"finite positive timeout required"};try{if(impl_->camera&&impl_->camera->IsInitialized()&&(impl_->state==LifecycleState::armed||impl_->state==LifecycleState::streaming))impl_->camera->EndAcquisition();impl_->release_all();impl_->state=LifecycleState::stopped;return{};}catch(const Exception&e){impl_->release_all();impl_->state=LifecycleState::faulted;return{ErrorCode::io_error,exception_detail(e)};}catch(...){impl_->release_all();impl_->state=LifecycleState::faulted;return{ErrorCode::io_error,"unknown stop failure"};}}
} // namespace ppbng_thermal
