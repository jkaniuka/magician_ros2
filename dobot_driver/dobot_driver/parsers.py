import struct

# This file contains each parser for the communication specification v1.1.5
# Note that long exposure to the code may and likely will cause a quick and sudden death
# It is advised that you do not put yourself to risk and you should therefore close this
# file as soon as possible

# The rationale around implementing the parser this way is due to it being concise,
# somewhat readable (given you have access to the API reference) and scalable -
# with the ability to easily add or change parsers in the future

# Format:
# key = message id according to API reference
# value[0] = parser for response to getters (direction=in, rw=0, isQueued=0)
# value[1] = parser for response to setters (direction=in, rw=1, isQueued=0)
# value[2] = parser for response to setters (direction=in, rw=1, isQueued=1)
# value[3] = parser for request to setters (direction=out, rw=1, isQueued=0/1)

parsers = {
    # Device information
    0: [lambda x: ''.join(map(chr, x)), None, None, lambda x: list(x[0].encode('ascii')) + [0x00]],
    1: [lambda x: ''.join(map(chr, x)), None, None, lambda x: list(x[0].encode('ascii')) + [0x00]],
    2: [lambda x: x[0:3], None, None, None],
    3: [lambda x: struct.unpack('<B', bytearray(x))[0], None, None, lambda x: list(struct.pack('<BB', *x))],
    4: [lambda x: struct.unpack('<L', bytearray(x))[0], None, None, None],
    5: [lambda x: struct.unpack('<' + 'L' * 3, bytearray(x)), None, None, None],
    # Real-time pose
    10: [lambda x: struct.unpack('<' + 'f' * 8, bytearray(x)), None, None, None],
    11: [None, None, None, lambda x: list(struct.pack('<Bff', *x))],
    13: [lambda x: struct.unpack('<f', bytearray(x))[0], None, None, None],
    # Alarm
    20: [lambda x: struct.unpack('<' + 'B' * 16, bytearray(x)), None, None, None], #BUG
    21: [None, None, None, None],
    # Homing function
    30: [lambda x: struct.unpack('<' + 'f' * 4, bytearray(x)), None, lambda x: struct.unpack('<Q', bytearray(x))[0], lambda x: list(struct.pack('<' + 'f' * 4, *x))],
    31: [None, None, lambda x: struct.unpack('<Q', bytearray(x))[0], lambda x: list(struct.pack('<f', *x))],
    32: [lambda x: struct.unpack('<Bf', bytearray(x)), None, lambda x: struct.unpack('<Q', bytearray(x))[0], lambda x: list(struct.pack('<Bf', *x))],
    # Handhold teaching
    40: [lambda x: x[0], None, None, lambda x: list(struct.pack('<B', *x))],
    41: [lambda x: x[0] == 1, None, None, lambda x: list(struct.pack('<B', *x))],
    42: [lambda x: x[0] == 1, None, None, None],
    # End effector
    60: [None, None, None, lambda x: list(struct.pack('<' + 'f' * 3, *x))], #BUG in original lib version was 4 instead of 3 !
    61: [lambda x: (x[0] == 1, x[1] == 2), None, lambda x: struct.unpack('<Q', bytearray(x))[0], lambda x: list(struct.pack('<BB', *x))],
    62: [lambda x: (x[0] == 1, x[1] == 2), None, lambda x: struct.unpack('<Q', bytearray(x))[0], lambda x: list(struct.pack('<BB', *x))],
    63: [lambda x: (x[0] == 1, x[1] == 2), None, lambda x: struct.unpack('<Q', bytearray(x))[0], lambda x: list(struct.pack('<BB', *x))],
    # JOG
    70: [lambda x: struct.unpack('<' + 'f' * 8, bytearray(x)), None, None, lambda x: list(struct.pack('<' + 'f' * 8, *x))],
    71: [lambda x: struct.unpack('<' + 'f' * 8, bytearray(x)), None, lambda x: struct.unpack('<Q', bytearray(x))[0], lambda x: list(struct.pack('<' + 'f' * 8, *x))],
    72: [lambda x: struct.unpack('<' + 'f' * 2, bytearray(x)), None, lambda x: struct.unpack('<Q', bytearray(x))[0], lambda x: list(struct.pack('<' + 'f' * 2, *x))],
    73: [None, None, lambda x: struct.unpack('<Q', bytearray(x))[0], lambda x: list(struct.pack('<' + 'B' * 2, *x))],
    74: [lambda x: struct.unpack('<' + 'f' * 2, bytearray(x)), None, lambda x: struct.unpack('<Q', bytearray(x))[0], lambda x: list(struct.pack('<' + 'f' * 2, *x))],
    # PTP
    80: [None, None, lambda x: struct.unpack('<Q', bytearray(x))[0], lambda x: list(struct.pack('<' + 'f' * 8, *x))],
    81: [None, None, lambda x: struct.unpack('<Q', bytearray(x))[0], lambda x: list(struct.pack('<' + 'f' * 4, *x))],
    82: [lambda x: struct.unpack('<' + 'f' * 2, bytearray(x)), None, lambda x: struct.unpack('<Q', bytearray(x))[0], lambda x: list(struct.pack('<' + 'f' * 2, *x))],
    83: [None, None, lambda x: struct.unpack('<Q', bytearray(x))[0], lambda x: list(struct.pack('<' + 'f' * 2, *x))],
    84: [None, None, lambda x: struct.unpack('<Q', bytearray(x))[0], lambda x: list(struct.pack('<Bffff', *x))],
    85: [lambda x: struct.unpack('<' + 'f' * 2, bytearray(x)), None, lambda x: struct.unpack('<Q', bytearray(x))[0], lambda x: list(struct.pack('<' + 'f' * 2, *x))],
    86: [None, None, lambda x: struct.unpack('<Q', bytearray(x))[0], lambda x: list(struct.pack('<Bfffff', *x))],
    87: [lambda x: struct.unpack('<' + 'f' * 3, bytearray(x)), None, lambda x: struct.unpack('<Q', bytearray(x))[0], lambda x: list(struct.pack('<' + 'f' * 3, *x))],
    88: [None, None, lambda x: struct.unpack('<Q', bytearray(x))[0], lambda x: list(struct.pack('<Bffff', *x))],
    89: [None, None, lambda x: struct.unpack('<Q', bytearray(x))[0], lambda x: list(struct.pack('<Bfffff', *x))],
    # Continuous path
    90: [lambda x: struct.unpack('<fffB', bytearray(x)), None, lambda x: struct.unpack('<Q', bytearray(x))[0], lambda x: list(struct.pack('<fffB', *x))],
    91: [None, None, lambda x: struct.unpack('<Q', bytearray(x))[0], lambda x: list(struct.pack('<Bffff', *x))],
    92: [None, None, lambda x: struct.unpack('<Q', bytearray(x))[0], lambda x: list(struct.pack('<Bffff', *x))],
    # Arc
    100: [lambda x: struct.unpack('<' + 'f' * 4, bytearray(x)), None, lambda x: struct.unpack('<Q', bytearray(x))[0], lambda x: list(struct.pack('<' + 'f' * 4, *x))],
    101: [None, None, lambda x: struct.unpack('<Q', bytearray(x))[0], lambda x: list(struct.pack('<' + 'f' * 8, *x))],
    # Wait
    110: [None, None, lambda x: struct.unpack('<Q', bytearray(x))[0], lambda x: list(struct.pack('<L', *x))],
    # Triggers
    120: [None, None, lambda x: struct.unpack('<Q', bytearray(x))[0], lambda x: list(struct.pack('<BBBH', *x))],
    # EIO
    130: [lambda x: struct.unpack('<B' * 2, bytearray(x)), None, lambda x: struct.unpack('<Q', bytearray(x))[0], lambda x: list(struct.pack('<' + 'B' * 2, *x))],
    131: [lambda x: struct.unpack('<B' * 2, bytearray(x)), None, lambda x: struct.unpack('<Q', bytearray(x))[0], lambda x: list(struct.pack('<' + 'B' * 2, *x))],
    132: [lambda x: struct.unpack('<Bff', bytearray(x)), None, lambda x: struct.unpack('<Q', bytearray(x))[0], lambda x: list(struct.pack('<Bff', *x))],
    133: [lambda x: struct.unpack('<B' * 2, bytearray(x)), None, None, None],
    134: [lambda x: struct.unpack('<BH', bytearray(x)), None, None, None],
    135: [None, None, lambda x: struct.unpack('<Q', bytearray(x))[0], lambda x: list(struct.pack('<BBf', *x))],
    137: [lambda x: struct.unpack('<B' * 3, bytearray(x)), None, lambda x: struct.unpack('<Q', bytearray(x))[0], lambda x: list(struct.pack('<' + 'B' * 3, *x))],
    138: [lambda x: struct.unpack('<B', bytearray(x)), None, lambda x: struct.unpack('<Q', bytearray(x))[0], lambda x: list(struct.pack('<' + 'B' * 3, *x))],
    # Calibration
    140: [lambda x: struct.unpack('<' + 'f' * 2, bytearray(x)), None, None, lambda x: list(struct.pack('<' + 'f' * 2, *x))],
    # Wifi
    150: [lambda x: x[0] == 1, None, None, lambda x: list(struct.pack('<B', *x))],
    151: [lambda x: ''.join(map(chr, x)), None, None, lambda x: list(x[0].encode('ascii')) + [0x00]],
    152: [lambda x: ''.join(map(chr, x)), None, None, lambda x: list(x[0].encode('ascii')) + [0x00]],
    153: [lambda x: struct.unpack('<B' * 5, bytearray(x)), None, None, lambda x: list(struct.pack('<B' * 5, *x))],
    154: [lambda x: struct.unpack('<B' * 4, bytearray(x)), None, None, lambda x: list(struct.pack('<B' * 4, *x))],
    155: [lambda x: struct.unpack('<B' * 4, bytearray(x)), None, None, lambda x: list(struct.pack('<B' * 4, *x))],
    156: [lambda x: struct.unpack('<B' * 4, bytearray(x)), None, None, lambda x: list(struct.pack('<B' * 4, *x))],
    157: [lambda x: x[0] == 1, None, None, None],
    # Losing-step detection
    170: [None, None, None, lambda x: list(struct.pack('<f', *x))],
    171: [None, None, lambda x: struct.unpack('<Q', bytearray(x))[0], None],
    # Queued execution control
    240: [None, None, None, None],
    241: [None, None, None, None],
    242: [None, None, None, None],
    243: [None, None, None, lambda x: list(struct.pack('<L' * 2, *x))],
    244: [None, None, None, None],
    245: [None, None, None, None],
    246: [lambda x: struct.unpack('<Q', bytearray(x))[0], None, None, None]
}
