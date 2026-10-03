#include "ppbng_thermal/recovery_supervisor.hpp"
#include <algorithm>
#include <limits>
#include <stdexcept>
namespace ppbng_thermal {
RecoverySupervisor::RecoverySupervisor(RecoveryPolicy p):policy_(p){if(p.consecutive_timeout_threshold==0||p.maximum_attempts==0||p.initial_backoff.count()<=0||p.maximum_backoff<p.initial_backoff)throw std::invalid_argument("invalid recovery policy");}
bool RecoverySupervisor::observe_timeout(){if(consecutive_timeouts_!=std::numeric_limits<std::size_t>::max())++consecutive_timeouts_;return consecutive_timeouts_>=policy_.consecutive_timeout_threshold;}
void RecoverySupervisor::observe_frame(){consecutive_timeouts_=0;}
void RecoverySupervisor::begin_episode(){consecutive_timeouts_=0;attempts_=0;}
RecoveryAttempt RecoverySupervisor::next_attempt(){if(attempts_>=policy_.maximum_attempts)return{false,true,attempts_,std::chrono::milliseconds{0},"recovery attempts exhausted"};++attempts_;auto d=policy_.initial_backoff;for(std::size_t i=1;i<attempts_&&d<policy_.maximum_backoff;++i)d=std::min(policy_.maximum_backoff,d*2);return{true,false,attempts_,d,"recovery attempt permitted"};}
void RecoverySupervisor::recovered(){attempts_=0;consecutive_timeouts_=0;}
std::size_t RecoverySupervisor::consecutive_timeouts()const noexcept{return consecutive_timeouts_;}
}
