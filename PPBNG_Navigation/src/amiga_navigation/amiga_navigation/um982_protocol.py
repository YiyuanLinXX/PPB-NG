"""Pure parsing and geometry helpers for the UM982 driver."""

from dataclasses import dataclass
import math

from pyproj import Geod


WGS84_GEOD = Geod(ellps='WGS84')


@dataclass(frozen=True)
class GgaFix:
    latitude: float
    longitude: float
    altitude_m: float
    quality: int
    differential_age_sec: float | None
    raw_sentence: str


@dataclass(frozen=True)
class HeadingSolution:
    solution_status: str
    position_type: str
    baseline_m: float
    heading_deg: float
    pitch_deg: float
    heading_stddev_deg: float
    pitch_stddev_deg: float
    satellites_tracked: int
    satellites_used: int


def parse_gga(sentence: str) -> GgaFix:
    """Parse a GGA sentence emitted by the master antenna."""
    text = sentence.strip()
    if not text.startswith('$') or '*' not in text:
        raise ValueError('incomplete NMEA sentence')
    payload, checksum_text = text[1:].rsplit('*', 1)
    checksum = 0
    for character in payload:
        checksum ^= ord(character)
    if checksum != int(checksum_text[:2], 16):
        raise ValueError('NMEA checksum mismatch')

    fields = payload.split(',')
    if len(fields) < 15 or not fields[0].endswith('GGA'):
        raise ValueError('not a GGA sentence')

    latitude = _nmea_coordinate(fields[2], fields[3], 2)
    longitude = _nmea_coordinate(fields[4], fields[5], 3)
    age_text = fields[13]
    differential_age = float(age_text) if age_text else None
    altitude = float(fields[9])
    finite_values = [latitude, longitude, altitude]
    if differential_age is not None:
        finite_values.append(differential_age)
    if not all(math.isfinite(value) for value in finite_values):
        raise ValueError('GGA contains NaN or infinity')
    return GgaFix(
        latitude=latitude,
        longitude=longitude,
        altitude_m=altitude,
        quality=int(fields[6]),
        differential_age_sec=differential_age,
        raw_sentence=sentence,
    )


def _nmea_coordinate(value: str, hemisphere: str, degree_digits: int) -> float:
    if not value or len(value) <= degree_digits:
        raise ValueError('empty NMEA coordinate')
    degrees = float(value[:degree_digits])
    minutes = float(value[degree_digits:])
    coordinate = degrees + minutes / 60.0
    if hemisphere in ('S', 'W'):
        coordinate = -coordinate
    elif hemisphere not in ('N', 'E'):
        raise ValueError(f'invalid hemisphere {hemisphere!r}')
    return coordinate


def parse_uniheadinga(sentence: str) -> HeadingSolution:
    """Parse the ASCII body of a UNIHEADINGA message."""
    text = sentence.strip()
    if not text.startswith('#UNIHEADINGA,'):
        raise ValueError('not a UNIHEADINGA message')
    if ';' not in text or '*' not in text:
        raise ValueError('incomplete UNIHEADINGA message')

    crc_payload, crc_text = text[1:].rsplit('*', 1)
    if _unicore_crc32(crc_payload.encode('ascii')) != int(crc_text[:8], 16):
        raise ValueError('UNIHEADINGA CRC mismatch')

    body = crc_payload.split(';', 1)[1]
    fields = body.split(',')
    if len(fields) < 14:
        raise ValueError(
            f'UNIHEADINGA body has {len(fields)} fields; expected at least 14'
        )

    baseline_m = float(fields[2])
    heading_deg = float(fields[3])
    pitch_deg = float(fields[4])
    heading_stddev_deg = float(fields[6])
    pitch_stddev_deg = float(fields[7])
    if not all(math.isfinite(value) for value in (
        baseline_m,
        heading_deg,
        pitch_deg,
        heading_stddev_deg,
        pitch_stddev_deg,
    )):
        raise ValueError('UNIHEADINGA contains NaN or infinity')

    return HeadingSolution(
        solution_status=fields[0].strip(),
        position_type=fields[1].strip(),
        baseline_m=baseline_m,
        heading_deg=heading_deg % 360.0,
        pitch_deg=pitch_deg,
        heading_stddev_deg=heading_stddev_deg,
        pitch_stddev_deg=pitch_stddev_deg,
        satellites_tracked=int(fields[9]),
        satellites_used=int(fields[10]),
    )


def _unicore_crc32(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc = ((crc >> 8) & 0x00FFFFFF) ^ _crc32_value(
            (crc ^ byte) & 0xFF
        )
    return crc


def _crc32_value(value: int) -> int:
    crc = value
    for _ in range(8):
        if crc & 1:
            crc = (crc >> 1) ^ 0xEDB88320
        else:
            crc >>= 1
    return crc


def midpoint_from_master(
    latitude: float,
    longitude: float,
    altitude_m: float,
    heading_deg: float,
    pitch_deg: float,
    baseline_m: float,
) -> tuple[float, float, float]:
    """Move from ANT1 to the 3-D midpoint of the ANT1-to-ANT2 baseline."""
    half_length = baseline_m / 2.0
    pitch_rad = math.radians(pitch_deg)
    horizontal_distance = half_length * math.cos(pitch_rad)
    vertical_distance = half_length * math.sin(pitch_rad)
    midpoint_lon, midpoint_lat, _ = WGS84_GEOD.fwd(
        longitude,
        latitude,
        heading_deg,
        horizontal_distance,
    )
    return midpoint_lat, midpoint_lon, altitude_m + vertical_distance


def heading_to_enu_yaw(heading_deg: float) -> float:
    """Convert true heading clockwise from north to ROS ENU yaw."""
    yaw = math.radians(90.0 - heading_deg)
    return (yaw + math.pi) % (2.0 * math.pi) - math.pi


def robot_heading_from_baseline(
    baseline_heading_deg: float,
    antenna_baseline_angle_deg: float,
    calibration_offset_deg: float = 0.0,
) -> float:
    """Convert ANT1-to-ANT2 heading into robot-forward true heading.

    ``antenna_baseline_angle_deg`` is the ANT1-to-ANT2 direction relative to
    robot forward, with positive angles clockwise when viewed from above.
    """
    return (
        baseline_heading_deg
        - antenna_baseline_angle_deg
        + calibration_offset_deg
    ) % 360.0
