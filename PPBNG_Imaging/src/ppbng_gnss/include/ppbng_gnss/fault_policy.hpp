#pragma once
#include <cstdint>
namespace ppbng_gnss {
enum class ProductionFaultKind {storage_create,storage_write,storage_flush_close,device_transport,
  device_timeout,integrity,timing};
struct FaultPolicy {const char* code;std::uint8_t severity;bool causes_global_stop;};
constexpr FaultPolicy fault_policy(ProductionFaultKind kind) noexcept {
 switch(kind){
  case ProductionFaultKind::storage_create:return {"GNSS_STORAGE_CREATE_FAILED",2U,true};
  case ProductionFaultKind::storage_write:return {"GNSS_STORAGE_WRITE_FAILED",3U,true};
  case ProductionFaultKind::storage_flush_close:return {"GNSS_STORAGE_FINALIZE_FAILED",3U,true};
  case ProductionFaultKind::device_transport:return {"GNSS_DEVICE_OR_TRANSPORT_FAULT",2U,false};
  case ProductionFaultKind::device_timeout:return {"GNSS_DEVICE_TIMEOUT",2U,false};
  case ProductionFaultKind::integrity:return {"GNSS_INTEGRITY_FAULT",3U,true};
  case ProductionFaultKind::timing:return {"GNSS_TIMING_FAULT",3U,true};
 }
 return {"GNSS_UNKNOWN_FAULT",2U,false};
}
}
