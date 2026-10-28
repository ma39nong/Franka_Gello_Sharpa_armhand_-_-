"""Read ROS Image metadata from CDR1 without copying the pixel sequence."""

from dataclasses import dataclass
import struct


@dataclass(frozen=True)
class ImageMetadata:
    header_time_ns: int | None
    height: int
    width: int
    encoding: str
    is_bigendian: int
    step: int
    data: memoryview


def read_image_metadata(serialized: bytes) -> ImageMetadata | None:
    """Return a pixel view; unsupported CDR variants use the ROS decoder.

    Every length is bounds-checked, including the pixel sequence. A truncated
    image must still fail validation even though no pixel values are decoded.
    """
    data = memoryview(serialized)
    if len(data) < 4:
        raise ValueError("truncated Image CDR encapsulation")
    representation = bytes(data[:2])
    if representation not in (b"\x00\x00", b"\x00\x01"):
        return None
    endian = "<" if representation == b"\x00\x01" else ">"
    offset = 4

    def uint32(signed: bool = False) -> int:
        nonlocal offset
        offset = (offset + 3) & ~3
        if offset + 4 > len(data):
            raise ValueError("truncated Image CDR integer")
        value = struct.unpack_from(endian + ("i" if signed else "I"), data, offset)[0]
        offset += 4
        return value

    def string() -> str:
        nonlocal offset
        size = uint32()
        if size == 0 or offset + size > len(data) or data[offset + size - 1] != 0:
            raise ValueError("invalid Image CDR string length or terminator")
        value = bytes(data[offset:offset + size - 1]).decode("utf-8")
        offset += size
        return value

    sec = uint32(signed=True)
    nanosec = uint32()
    string()  # header.frame_id (variable length affects subsequent alignment)
    height = uint32()
    width = uint32()
    encoding = string()
    if offset >= len(data):
        raise ValueError("truncated Image CDR endianness")
    is_bigendian = int(data[offset])
    offset += 1
    step = uint32()
    pixel_bytes = uint32()
    if pixel_bytes > len(data) - offset:
        raise ValueError("truncated Image CDR pixel sequence")
    timestamp = sec * 1_000_000_000 + nanosec
    return ImageMetadata(
        header_time_ns=timestamp if timestamp > 0 else None,
        height=height,
        width=width,
        encoding=encoding,
        is_bigendian=is_bigendian,
        step=step,
        data=data[offset:offset + pixel_bytes],
    )
