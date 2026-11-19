"""
Minimal CDR (Common Data Representation) reader for ROS2 message payloads
received over zenoh. Covers the primitives and sequence/array patterns used
by msg_decoders.py - not a full IDL-driven deserializer, since there is no
ROS2 install here to introspect message definitions from.
"""

import struct


class CDRReader:
    def __init__(self, data):
        self.data = memoryview(data)

        if len(data) < 4:
            raise ValueError("CDR payload too short")

        identifier = bytes(self.data[:2])

        if identifier == b"\x00\x01":
            self.endian = "<"
        elif identifier == b"\x00\x00":
            self.endian = ">"
        else:
            raise ValueError(f"Unsupported CDR encapsulation: {identifier.hex()}")

        self.pos = 4
        self.base = 4

    def align(self, size):
        relative = self.pos - self.base
        self.pos += (-relative) % size

    def _read(self, fmt, alignment):
        self.align(alignment)
        size = struct.calcsize(fmt)

        if self.pos + size > len(self.data):
            raise ValueError("Truncated CDR payload")

        value = struct.unpack_from(self.endian + fmt, self.data, self.pos)[0]
        self.pos += size
        return value

    def bool(self):
        return bool(self._read("B", 1))

    def u8(self):
        return self._read("B", 1)

    def i8(self):
        return self._read("b", 1)

    def u16(self):
        return self._read("H", 2)

    def i16(self):
        return self._read("h", 2)

    def u32(self):
        return self._read("I", 4)

    def i32(self):
        return self._read("i", 4)

    def u64(self):
        return self._read("Q", 8)

    def i64(self):
        return self._read("q", 8)

    def f32(self):
        return self._read("f", 4)

    def f64(self):
        return self._read("d", 8)

    def string(self):
        length = self.u32()

        if length < 1 or self.pos + length > len(self.data):
            raise ValueError("Invalid CDR string")

        result = bytes(self.data[self.pos:self.pos + length])
        self.pos += length
        return result.rstrip(b"\x00").decode("utf-8", errors="replace")

    def byte_array(self):
        length = self.u32()

        if self.pos + length > len(self.data):
            raise ValueError("Invalid CDR byte array")

        result = bytes(self.data[self.pos:self.pos + length])
        self.pos += length
        return result

    def _numeric_sequence(self, item_fmt, alignment):
        count = self.u32()
        self.align(alignment)
        fmt = f"{count}{item_fmt}"
        size = struct.calcsize(fmt)

        if self.pos + size > len(self.data):
            raise ValueError("Truncated CDR payload")

        values = list(struct.unpack_from(self.endian + fmt, self.data, self.pos))
        self.pos += size
        return values

    def f64_sequence(self):
        return self._numeric_sequence("d", 8)

    def f32_sequence(self):
        return self._numeric_sequence("f", 4)

    def i32_sequence(self):
        return self._numeric_sequence("i", 4)

    def string_sequence(self):
        count = self.u32()
        return [self.string() for _ in range(count)]
