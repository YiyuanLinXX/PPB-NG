# Describes a raw frame without transporting the raw image through ROS.
# Transport geometry is deliberately separate from the validated image region.
# For an A6701 payload reported as 640x513 with a 640x512 radiometric image,
# transport_height is 513 while image_height is 512. The remaining bytes are
# described as auxiliary data and are never silently discarded.

uint8 BYTE_ORDER_UNKNOWN=0
uint8 BYTE_ORDER_LITTLE_ENDIAN=1
uint8 BYTE_ORDER_BIG_ENDIAN=2

uint8 GEOMETRY_SOURCE_UNVERIFIED=0
uint8 GEOMETRY_SOURCE_CAMERA_READBACK=1
uint8 GEOMETRY_SOURCE_VENDOR_DOCUMENTED=2

ppbng_interfaces/SampleStamp stamp

uint32 transport_width
uint32 transport_height
uint32 row_stride_bytes
uint64 payload_size_bytes
string pixel_format
uint8 byte_order

uint32 image_x
uint32 image_y
uint32 image_width
uint32 image_height
uint64 image_data_offset_bytes
uint64 image_data_length_bytes

uint32 auxiliary_row_count
uint64 auxiliary_data_offset_bytes
uint64 auxiliary_data_length_bytes
string auxiliary_type
string auxiliary_schema_version
uint8 geometry_source

bool raw_payload_preserved
bool complete
string validation_detail

# Per-frame camera settings. For Spinnaker RGB cameras, exposure/gain/black
# level come from the image's hardware Chunk Data. White-balance ratios are
# read immediately after receipt because this camera does not expose them as
# image chunks; settings_source makes that distinction explicit.
bool chunk_data_valid
bool chunk_frame_id_valid
uint64 chunk_frame_id
bool chunk_timestamp_valid
uint64 chunk_timestamp
bool exposure_time_valid
float64 exposure_time_us
bool gain_valid
float64 gain_db
bool black_level_valid
float64 black_level
bool white_balance_red_valid
float64 white_balance_red
bool white_balance_blue_valid
float64 white_balance_blue
string exposure_auto
string gain_auto
string balance_white_auto
string settings_source
