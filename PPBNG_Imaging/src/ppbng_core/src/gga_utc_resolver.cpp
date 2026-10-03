#include "ppbng_core/gga_utc_resolver.hpp"
#include <array>
#include <charconv>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
namespace ppbng_core {namespace {constexpr std::int64_t second=1000000000LL,day=86400LL*second;std::optional<std::int64_t>tod(std::string_view s){if(s.size()<6)return{};int h=0,m=0;double sec=0;auto a=std::from_chars(s.data(),s.data()+2,h);auto b=std::from_chars(s.data()+2,s.data()+4,m);try{sec=std::stod(std::string(s.substr(4)));}catch(...){return{};}if(a.ec!=std::errc{}||b.ec!=std::errc{}||h<0||h>23||m<0||m>59||!std::isfinite(sec)||sec<0||sec>=60)return{};return(static_cast<std::int64_t>(h)*3600+static_cast<std::int64_t>(m)*60)*second+static_cast<std::int64_t>(std::llround(sec*second));}std::uint64_t diff(std::int64_t a,std::int64_t b){return a>=b?static_cast<std::uint64_t>(a-b):static_cast<std::uint64_t>(b-a);}}
GgaUtcResolver::GgaUtcResolver(std::uint64_t age,std::uint64_t residual):max_age_(age),max_residual_(residual){if(!age||!residual||residual>=static_cast<std::uint64_t>(day/2))throw std::invalid_argument("invalid GGA UTC resolver bounds");}
bool GgaUtcResolver::observe_anchor(const GnssUtcAnchor&a){
 if(!a.sequence||!a.connection_epoch||!a.host_monotonic_ns||a.utc_nanoseconds<=0)return false;
 if(!anchors_.empty()){
  const auto&last=anchors_.back();
  if(a.connection_epoch<last.connection_epoch)return false;
  if(a.connection_epoch>last.connection_epoch)anchors_.clear();
  else if(a.host_monotonic_ns<=last.host_monotonic_ns||a.sequence<=last.sequence)return false;
 }
 anchors_.push_back(a);if(anchors_.size()>capacity_)anchors_.pop_front();return true;
}
std::optional<ResolvedGgaUtc> GgaUtcResolver::resolve(std::string_view text,std::uint64_t host,std::uint64_t epoch)const{
 auto time=tod(text);if(!time||anchors_.empty())return{};
 std::optional<ResolvedGgaUtc> best;std::optional<std::int64_t> unique_utc;
 for(const auto&anchor:anchors_){
  if(anchor.connection_epoch!=epoch)continue;
  const auto hd=host>=anchor.host_monotonic_ns?host-anchor.host_monotonic_ns:anchor.host_monotonic_ns-host;
  if(hd>max_age_||hd>static_cast<std::uint64_t>((std::numeric_limits<std::int64_t>::max)()))continue;
  const auto signed_delta=host>=anchor.host_monotonic_ns?static_cast<std::int64_t>(hd):-static_cast<std::int64_t>(hd);
  if((signed_delta>0&&anchor.utc_nanoseconds>(std::numeric_limits<std::int64_t>::max)()-signed_delta)||
     (signed_delta<0&&anchor.utc_nanoseconds<(std::numeric_limits<std::int64_t>::min)()-signed_delta))continue;
  const auto expected=anchor.utc_nanoseconds+signed_delta;
  const auto base=(anchor.utc_nanoseconds/day)*day;
  std::array<std::int64_t,3>c{base-day+*time,base+*time,base+day+*time};
  std::size_t candidate=0;for(std::size_t i=1;i<c.size();++i)if(diff(c[i],expected)<diff(c[candidate],expected))candidate=i;
  const auto residual=diff(c[candidate],expected);if(residual>max_residual_)continue;
  for(std::size_t i=0;i<c.size();++i)if(i!=candidate&&diff(c[i],expected)<=max_residual_)return{};
  if(unique_utc&&*unique_utc!=c[candidate])return{};
  unique_utc=c[candidate];
  if(!best||residual<best->clock_residual_ns||(residual==best->clock_residual_ns&&hd<best->host_delta_ns))
   best=ResolvedGgaUtc{c[candidate],anchor.sequence,hd,residual};
 }
 return best;
}
void GgaUtcResolver::reset(){anchors_.clear();}
}
