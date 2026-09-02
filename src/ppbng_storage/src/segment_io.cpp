#include "ppbng_storage/segment_io.hpp"
#include <algorithm>
#include <array>
#include <cstring>
#include <system_error>
#include <type_traits>
#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif

namespace ppbng_storage
{
namespace
{
constexpr std::size_t kFileHeaderSize=16,kRecordHeaderSize=72,kTrailerSize=24;
const std::array<std::byte,8> kFileMagic{std::byte{'P'},std::byte{'P'},std::byte{'B'},std::byte{'N'},std::byte{'G'},std::byte{'S'},std::byte{'G'},std::byte{'1'}};
constexpr std::array<std::uint32_t,256> make_crc32_table() noexcept
{
  std::array<std::uint32_t,256> table{};
  for(std::size_t value=0;value<table.size();++value){
    auto remainder=static_cast<std::uint32_t>(value);
    for(unsigned bit=0;bit<8U;++bit){
      remainder=(remainder>>1U)^(0xedb88320U&static_cast<std::uint32_t>(
        -static_cast<std::int32_t>(remainder&1U)));
    }
    table[value]=remainder;
  }
  return table;
}
constexpr auto kCrc32Table=make_crc32_table();
template<class T>void put(std::byte *p,T v){using U=typename std::make_unsigned<T>::type;U u=static_cast<U>(v);for(std::size_t i=0;i<sizeof(T);++i)p[i]=std::byte((u>>(i*8))&0xffU);}
template<class T>T get(const std::byte*p){using U=typename std::make_unsigned<T>::type;U u=0;for(std::size_t i=0;i<sizeof(T);++i)u|=static_cast<U>(std::to_integer<unsigned char>(p[i]))<<(i*8);return static_cast<T>(u);}
bool safe_relative(const std::filesystem::path&p){if(p.empty()||p.is_absolute()||p.has_root_name()||p.has_root_directory())return false;auto i=p.begin();if(i==p.end()||*i!="segments")return false;for(const auto&c:p)if(c.empty()||c=="."||c=="..")return false;return true;}
bool beneath(const std::filesystem::path&c,const std::filesystem::path&r){auto ci=c.begin();for(auto ri=r.begin();ri!=r.end();++ri,++ci)if(ci==c.end()||*ci!=*ri)return false;return true;}
SegmentStatus resolve(const std::filesystem::path&root,const std::filesystem::path&rel,std::filesystem::path&out)
{if(!safe_relative(rel))return{SegmentIoError::unsafe_path,"segment must be beneath segments/",0};std::error_code e;auto cr=std::filesystem::weakly_canonical(std::filesystem::absolute(root,e),e);if(e||!std::filesystem::is_directory(cr,e))return{SegmentIoError::unsafe_path,"invalid session root",0};out=(cr/rel).lexically_normal();auto parent=std::filesystem::weakly_canonical(out.parent_path(),e);if(e||!std::filesystem::is_directory(parent,e)||!beneath(parent,cr))return{SegmentIoError::unsafe_path,"segment parent escaped session root",0};return{};}
std::array<std::byte,kFileHeaderSize> file_header(){std::array<std::byte,kFileHeaderSize>b{};std::copy(kFileMagic.begin(),kFileMagic.end(),b.begin());put<std::uint16_t>(b.data()+8,1);put<std::uint16_t>(b.data()+10,0);put<std::uint32_t>(b.data()+12,kFileHeaderSize);return b;}
std::array<std::byte,kRecordHeaderSize> record_header(const FrameEnvelope&e,std::uint64_t n,std::uint32_t crc)
{std::array<std::byte,kRecordHeaderSize>b{};std::memcpy(b.data(),"FRM1",4);put<std::uint16_t>(b.data()+4,1);put<std::uint16_t>(b.data()+6,kRecordHeaderSize);put<std::uint64_t>(b.data()+8,e.sample_id);put<std::uint64_t>(b.data()+16,e.trigger_id);put<std::uint64_t>(b.data()+24,e.pps_sequence);put<std::int64_t>(b.data()+32,e.utc_nanoseconds);b[40]=std::byte(static_cast<unsigned char>(e.time_quality));put<std::uint64_t>(b.data()+48,n);put<std::uint32_t>(b.data()+56,crc);put<std::uint64_t>(b.data()+60,kRecordHeaderSize+n+kTrailerSize);put<std::uint32_t>(b.data()+68,payload_crc32(b.data(),68));return b;}
std::array<std::byte,kTrailerSize> trailer(std::uint64_t total,std::uint32_t crc)
{std::array<std::byte,kTrailerSize>b{};std::memcpy(b.data(),"CMIT",4);put<std::uint16_t>(b.data()+4,1);put<std::uint16_t>(b.data()+6,kTrailerSize);put<std::uint64_t>(b.data()+8,total);put<std::uint32_t>(b.data()+16,crc);put<std::uint32_t>(b.data()+20,payload_crc32(b.data(),20));return b;}
}

std::uint32_t payload_crc32(const std::byte*d,std::size_t n)noexcept
{
  std::uint32_t crc=0xffffffffU;
  for(std::size_t index=0;index<n;++index){
    const auto byte=std::to_integer<unsigned char>(d[index]);
    crc=(crc>>8U)^kCrc32Table[(crc^byte)&0xffU];
  }
  return ~crc;
}
SegmentWriter::SegmentWriter(std::filesystem::path p,SegmentWriterOptions o):path_(std::move(p)),options_(o){}
SegmentWriter::~SegmentWriter(){if(stream_.is_open())stream_.close();}
std::pair<SegmentStatus,std::unique_ptr<SegmentWriter>> SegmentWriter::create(const std::filesystem::path&r,const std::filesystem::path&rel,SegmentWriterOptions o)
{std::filesystem::path p;auto s=resolve(r,rel,p);if(!s.ok())return{s,nullptr};auto w=std::unique_ptr<SegmentWriter>(new SegmentWriter(p,o));s=w->open_exclusive();if(!s.ok())return{s,nullptr};return{s,std::move(w)};}
SegmentStatus SegmentWriter::open_exclusive()
{
#ifdef _WIN32
  HANDLE reservation=CreateFileW(path_.c_str(),GENERIC_WRITE,0,nullptr,CREATE_NEW,FILE_ATTRIBUTE_NORMAL,nullptr);
  if(reservation==INVALID_HANDLE_VALUE){auto e=GetLastError();return{e==ERROR_FILE_EXISTS?SegmentIoError::already_exists:SegmentIoError::open_failed,"exclusive create failed",0};}
  CloseHandle(reservation);
#else
  std::error_code e;if(std::filesystem::exists(path_,e))return{SegmentIoError::already_exists,"segment exists",0};
#endif
  stream_.open(path_,std::ios::binary|std::ios::out|std::ios::trunc);if(!stream_)return{SegmentIoError::open_failed,"open failed",0};auto h=file_header();auto s=write_bytes(h.data(),h.size());if(s.ok()){committed_bytes_=physical_bytes_;stream_.flush();}return s;
}
SegmentStatus SegmentWriter::write_bytes(const std::byte*d,std::size_t n)
{if(n>std::numeric_limits<std::uint64_t>::max()-physical_bytes_)return{SegmentIoError::format_error,"file offset overflow",physical_bytes_};std::size_t allowed=n;if(physical_bytes_>=options_.fail_after_total_bytes)allowed=0;else allowed=static_cast<std::size_t>(std::min<std::uint64_t>(n,options_.fail_after_total_bytes-physical_bytes_));if(allowed){stream_.write(reinterpret_cast<const char*>(d),static_cast<std::streamsize>(allowed));physical_bytes_+=allowed;}if(allowed!=n||!stream_)return{SegmentIoError::write_failed,"injected or physical write failure",physical_bytes_};return{};}
SegmentStatus SegmentWriter::append(const FrameEnvelope&e,const std::byte*p,std::size_t n)
{if(!p&&n)return{SegmentIoError::format_error,"null payload",physical_bytes_};if(n>std::numeric_limits<std::uint64_t>::max()-kRecordHeaderSize-kTrailerSize)return{SegmentIoError::format_error,"record size overflow",physical_bytes_};auto crc=payload_crc32(p,n);auto h=record_header(e,n,crc);auto t=trailer(h.size()+n+kTrailerSize,crc);auto s=write_bytes(h.data(),h.size());if(s.ok())s=write_bytes(p,n);if(s.ok())s=write_bytes(t.data(),t.size());if(!s.ok())return s;committed_bytes_=physical_bytes_;++committed_records_;last_sample_id_=e.sample_id;return{};}
SegmentStatus SegmentWriter::flush(){stream_.flush();return stream_?SegmentStatus{}:SegmentStatus{SegmentIoError::flush_failed,"flush failed",physical_bytes_};}
std::pair<SegmentStatus,SegmentCheckpoint> SegmentWriter::checkpoint(){auto s=flush();return{s,{committed_records_,committed_bytes_,last_sample_id_}};}

std::pair<SegmentStatus,std::unique_ptr<SegmentReader>> SegmentReader::open(const std::filesystem::path&root,const std::filesystem::path&relative){std::filesystem::path p;auto s=resolve(root,relative,p);if(!s.ok())return{s,nullptr};auto r=std::unique_ptr<SegmentReader>(new SegmentReader(p));s=r->open_file();if(!s.ok())return{s,nullptr};return{s,std::move(r)};}
SegmentStatus SegmentReader::open_file(){stream_.open(path_,std::ios::binary);if(!stream_)return{SegmentIoError::open_failed,"open failed",0};std::array<std::byte,kFileHeaderSize>b{};stream_.read(reinterpret_cast<char*>(b.data()),b.size());if(stream_.gcount()!=static_cast<std::streamsize>(b.size())||!std::equal(kFileMagic.begin(),kFileMagic.end(),b.begin())||get<std::uint16_t>(b.data()+8)!=1||get<std::uint32_t>(b.data()+12)!=kFileHeaderSize)return{SegmentIoError::invalid_file_header,"invalid segment header",0};offset_=last_committed_offset_=kFileHeaderSize;return{};}
SegmentStatus SegmentReader::next(SegmentFrame&f)
{std::array<std::byte,kRecordHeaderSize>h{};stream_.read(reinterpret_cast<char*>(h.data()),h.size());auto got=stream_.gcount();if(got==0&&stream_.eof())return{SegmentIoError::end_of_file,"",offset_};if(got!=static_cast<std::streamsize>(h.size()))return{SegmentIoError::truncated_tail,"partial record header",offset_};if(std::memcmp(h.data(),"FRM1",4)||get<std::uint16_t>(h.data()+4)!=1||get<std::uint16_t>(h.data()+6)!=kRecordHeaderSize)return{SegmentIoError::format_error,"record header",offset_};if(payload_crc32(h.data(),68)!=get<std::uint32_t>(h.data()+68))return{SegmentIoError::checksum_error,"header checksum",offset_};auto n=get<std::uint64_t>(h.data()+48),total=get<std::uint64_t>(h.data()+60);auto quality=std::to_integer<unsigned char>(h[40]);if(quality>static_cast<unsigned char>(TimeQuality::locked)||n>static_cast<std::uint64_t>(std::numeric_limits<std::streamsize>::max())||n>std::numeric_limits<std::size_t>::max()||n>std::numeric_limits<std::uint64_t>::max()-kRecordHeaderSize-kTrailerSize||total!=kRecordHeaderSize+n+kTrailerSize)return{SegmentIoError::format_error,"record length or time quality",offset_};f.payload.resize(static_cast<std::size_t>(n));stream_.read(reinterpret_cast<char*>(f.payload.data()),static_cast<std::streamsize>(n));if(stream_.gcount()!=static_cast<std::streamsize>(n))return{SegmentIoError::truncated_tail,"partial payload",offset_};std::array<std::byte,kTrailerSize>t{};stream_.read(reinterpret_cast<char*>(t.data()),t.size());if(stream_.gcount()!=static_cast<std::streamsize>(t.size()))return{SegmentIoError::truncated_tail,"missing commit trailer",offset_};auto crc=get<std::uint32_t>(h.data()+56);if(std::memcmp(t.data(),"CMIT",4)||get<std::uint64_t>(t.data()+8)!=total||get<std::uint32_t>(t.data()+16)!=crc||payload_crc32(t.data(),20)!=get<std::uint32_t>(t.data()+20))return{SegmentIoError::checksum_error,"commit trailer checksum",offset_};if(payload_crc32(f.payload.data(),f.payload.size())!=crc)return{SegmentIoError::checksum_error,"payload checksum",offset_};f.record_offset=offset_;f.envelope={get<std::uint64_t>(h.data()+8),get<std::uint64_t>(h.data()+16),get<std::uint64_t>(h.data()+24),get<std::int64_t>(h.data()+32),static_cast<TimeQuality>(quality)};offset_+=total;last_committed_offset_=offset_;return{};}
SegmentStatus SegmentReader::scan_to_end(){SegmentFrame f;for(;;){auto s=next(f);if(s.error==SegmentIoError::end_of_file)return{};if(!s.ok())return s;}}
SegmentStatus recover_truncated_segment(const std::filesystem::path&r,const std::filesystem::path&rel,std::uint64_t&size)
{std::filesystem::path p;auto s=resolve(r,rel,p);if(!s.ok())return s;auto opened=SegmentReader::open(r,rel);if(!opened.first.ok())return opened.first;s=opened.second->scan_to_end();size=opened.second->last_committed_offset();if(s.ok())return{};if(s.error!=SegmentIoError::truncated_tail)return s;std::error_code e;std::filesystem::resize_file(p,size,e);if(e)return{SegmentIoError::write_failed,e.message(),size};return{};}
bool SegmentRolloverPolicy::should_rollover(std::uint64_t current,std::uint64_t payload)const noexcept{if(payload>std::numeric_limits<std::uint64_t>::max()-record_overhead_bytes())return true;auto next=payload+record_overhead_bytes();return current>maximum_bytes_||next>maximum_bytes_-std::min(current,maximum_bytes_);}
} // namespace ppbng_storage
