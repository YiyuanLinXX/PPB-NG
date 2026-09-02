#include "ppbng_thermal/runtime_contract.hpp"
#include <chrono>
namespace ppbng_thermal {
RuntimeFrameDescriptor describe_runtime_frame(const FrameInfo&i,const ThermalReadback&r){
 RuntimeFrameDescriptor d;d.transport_width=640;d.transport_height=513;d.row_stride_bytes=1280;d.payload_size_bytes=i.payload_bytes;d.image_y=1;d.image_width=640;d.image_height=512;d.image_data_offset_bytes=1280;d.image_data_length_bytes=655360;d.auxiliary_row_count=1;d.auxiliary_data_offset_bytes=0;d.auxiliary_data_length_bytes=1280;d.auxiliary_type="FLIR_A6701_OPAQUE_FRAME_METADATA_ROW";d.raw_payload_preserved=true;d.complete=i.complete;d.sdk_frame_counter=i.frame_id;d.camera_timestamp_ns=i.camera_timestamp_ns;d.host_receive_monotonic_ns=static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(i.host_receive_monotonic.time_since_epoch()).count());
 const bool readback_ok=r.width==640&&r.transport_height==513&&r.image_height==512&&r.row_stride_bytes==1280&&r.payload_bytes==656640;
 if(!readback_ok)d.validation_detail="A6701 readback differs from 640x513 transport contract";else if(i.payload_bytes!=656640)d.validation_detail="A6701 frame is not the full 656640-byte payload";else if(!i.complete)d.validation_detail="incomplete A6701 frame";else d.validation_detail="full 640x513 payload preserved; first row remains opaque";
 return d;
}
Status validate_inert_arm_preflight(bool authorized,bool prepared,const std::string&id,const std::string&model,const std::string&channel,const ThermalConfiguration&c,std::size_t capacity,std::uint64_t wait){
 if(!authorized)return {ErrorCode::invalid_state,"hardware authorization is false"};if(!prepared)return {ErrorCode::invalid_state,"session is not prepared"};if(id.empty()||model.empty()||channel.empty())return {ErrorCode::invalid_configuration,"exact identity/model and trigger channel are required"};if(!capacity||!wait)return {ErrorCode::invalid_configuration,"positive association bounds are required"};if(c.frame_sync_mode.empty()||c.frame_sync_polarity.empty())return {ErrorCode::invalid_configuration,"external frame-sync configuration is required"};return {ErrorCode::none,"inert preflight passed without SDK access"};
}
} // namespace ppbng_thermal
