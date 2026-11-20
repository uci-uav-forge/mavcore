import math
import struct
from dataclasses import dataclass
from enum import Enum, IntEnum

import pymavlink.dialects.v20.all as dialect

# Longest param_id. A 16 character name has no NULL terminator.
PARAM_ID_LEN = 16
# PARAM_VALUE.param_index is a uint16, the vehicle sends -1 (65535) when it
# reports a value that was not requested by index (after a set or read by name).
NO_PARAM_INDEX = 65535


class MAVParamType(IntEnum):
    UNKNOWN = -1
    UINT8 = 1
    INT8 = 2
    UINT16 = 3
    INT16 = 4
    UINT32 = 5
    INT32 = 6
    UINT64 = 7
    INT64 = 8
    REAL32 = 9
    REAL64 = 10


class MAVParamError(IntEnum):
    UNKNOWN = -1
    NO_ERROR = 0
    DOES_NOT_EXIST = 1
    VALUE_OUT_OF_RANGE = 2
    PERMISSION_DENIED = 3
    COMPONENT_NOT_FOUND = 4
    READ_ONLY = 5


class ParamEncoding(Enum):
    """
    How a parameter value is packed into the float param_value field.

    C_CAST: the value is cast to a float (ArduPilot). Integers over 24 bits lose precision.
    BYTEWISE: the value's bytes are copied into the float (PX4).
    """

    C_CAST = "c_cast"
    BYTEWISE = "bytewise"

    @classmethod
    def from_capabilities(cls, capabilities: int) -> "ParamEncoding":
        """
        Picks the encoding from AUTOPILOT_VERSION.capabilities. Neither ArduPilot nor PX4 sets
        its bit, so this only detects BYTEWISE when it is advertised and defaults to C_CAST.
        """
        if capabilities & dialect.MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_BYTEWISE:
            return cls.BYTEWISE
        return cls.C_CAST


_INT_FORMATS = {
    MAVParamType.UINT8: "<B",
    MAVParamType.INT8: "<b",
    MAVParamType.UINT16: "<H",
    MAVParamType.INT16: "<h",
    MAVParamType.UINT32: "<I",
    MAVParamType.INT32: "<i",
}


def to_param_type(param_type: int) -> MAVParamType:
    try:
        return MAVParamType(param_type)
    except ValueError:
        return MAVParamType.UNKNOWN


def _to_float32(value: float) -> float:
    return struct.unpack("<f", struct.pack("<f", value))[0]


def encode_param_value(
    value: int | float, param_type: MAVParamType, encoding: ParamEncoding
) -> float:
    """
    Packs value into the float sent in PARAM_SET. Raises ValueError if value cannot be sent
    exactly as param_type.
    """
    if param_type == MAVParamType.REAL32:
        if not math.isfinite(value):
            raise ValueError(f"Parameter value must be finite, got {value}")
        try:
            return _to_float32(float(value))
        except OverflowError:
            raise ValueError(f"{value} is out of range for REAL32") from None

    if param_type not in _INT_FORMATS:
        raise ValueError(
            f"{param_type.name} parameters are not supported, the parameter protocol "
            "only carries 32-bit values"
        )
    if isinstance(value, float) and not value.is_integer():
        raise ValueError(f"{param_type.name} parameter value must be an integer")
    value = int(value)
    try:
        packed = struct.pack(_INT_FORMATS[param_type], value)
    except struct.error:
        raise ValueError(f"{value} is out of range for {param_type.name}") from None

    if encoding == ParamEncoding.C_CAST:
        if _to_float32(value) != value:
            raise ValueError(
                f"{value} cannot be sent exactly with C_CAST encoding (over 24 bits)"
            )
        return float(value)

    # Python floats are doubles, so int bit patterns that are signalling NaNs in float32
    # come back quieted. This affects some values when sending or receiving BYTEWISE ints.
    return struct.unpack("<f", packed.ljust(4, b"\x00"))[0]


def decode_param_value(
    raw: float, param_type: MAVParamType, encoding: ParamEncoding
) -> int | float:
    """
    Unpacks the float received in PARAM_VALUE into the parameter's real type.
    Returns raw if the type is a float or not a supported 32-bit type.
    """
    if param_type not in _INT_FORMATS:
        return raw
    if encoding == ParamEncoding.C_CAST:
        return int(raw) if math.isfinite(raw) else raw
    size = struct.calcsize(_INT_FORMATS[param_type])
    return struct.unpack(_INT_FORMATS[param_type], struct.pack("<f", raw)[:size])[0]


def param_values_match(
    sent: int | float, received: int | float, param_type: MAVParamType
) -> bool:
    """
    Whether a PARAM_VALUE received after a PARAM_SET confirms the set. Floats are compared
    within 0.01%: ArduPilot reports the default value when a set lands within 0.01% of it.
    """
    if param_type in _INT_FORMATS:
        return sent == received
    sent = _to_float32(float(sent))
    return sent == received or math.isclose(sent, received, rel_tol=1e-4)


def _encode_param_id(param_id: str) -> bytes:
    encoded = param_id.encode("ascii")
    if not 0 < len(encoded) <= PARAM_ID_LEN:
        raise ValueError(
            f"param_id must be 1 to {PARAM_ID_LEN} ASCII characters, got '{param_id}'"
        )
    return encoded


def _decode_param_id(param_id: bytes | str) -> str:
    if isinstance(param_id, bytes):
        return param_id.split(b"\x00", 1)[0].decode("ascii", errors="replace")
    return param_id.split("\x00", 1)[0]


@dataclass(frozen=True)
class Param:
    """
    A parameter value reported by the vehicle.

    index: -1 if the vehicle did not report it (after a set or a read by name)
    count: total parameters the component has
    """

    name: str
    value: int | float
    param_type: MAVParamType
    index: int
    count: int
    system: int
    component: int

    @classmethod
    def from_msg(cls, msg, encoding: ParamEncoding) -> "Param":
        param_type = to_param_type(msg.param_type)
        return cls(
            name=_decode_param_id(msg.param_id),
            value=decode_param_value(msg.param_value, param_type, encoding),
            param_type=param_type,
            index=-1 if msg.param_index == NO_PARAM_INDEX else msg.param_index,
            count=msg.param_count,
            system=msg.get_srcSystem(),
            component=msg.get_srcComponent(),
        )
