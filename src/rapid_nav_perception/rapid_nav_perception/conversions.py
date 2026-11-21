"""
Conversions between sensor_msgs/Image fields and numpy arrays.

No ROS imports: the functions work on any object with the sensor_msgs/Image fields.
"""

import numpy as np

COLOUR_ENCODINGS = ('rgb8', 'bgr8')
FLOAT_ENCODINGS = ('32FC1', '32FC2')


def _layout(encoding):
    """Return (numpy dtype, channels) of an encoding, or raise ValueError."""
    if encoding in COLOUR_ENCODINGS:
        return np.dtype(np.uint8), 3
    if encoding == '32FC1':
        return np.dtype(np.float32), 1
    if encoding == '32FC2':
        return np.dtype(np.float32), 2
    raise ValueError('unsupported image encoding: {!r}'.format(encoding))


def image_to_array(msg):
    """
    Return the image of a message as a numpy array.

    rgb8 gives (H, W, 3) uint8 RGB; bgr8 is returned as RGB too. 32FC1 gives (H, W) float32, 32FC2
    gives (H, W, 2) float32. The row padding (step) and the byte order (is_bigendian) are honoured.
    """
    dtype, channels = _layout(msg.encoding)
    height, width = int(msg.height), int(msg.width)
    row_bytes = width * channels * dtype.itemsize
    step = int(msg.step)
    if step < row_bytes:
        raise ValueError('step {} is smaller than a row of {} bytes'.format(step, row_bytes))
    buffer = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    if buffer.size < step * height:
        raise ValueError('data has {} bytes, need {}'.format(buffer.size, step * height))
    rows = buffer[:step * height].reshape(height, step)[:, :row_bytes]
    order = '>' if msg.is_bigendian else '<'
    wire = np.dtype(order + dtype.char)
    array = np.ascontiguousarray(rows).view(wire).reshape(height, width, channels)
    array = array.astype(dtype)
    if msg.encoding == 'bgr8':
        array = array[..., ::-1]
    if channels == 1:
        return array[..., 0]
    return array


def array_to_image_fields(array, encoding):
    """
    Return (height, width, encoding, step, data) for a message, from an array.

    The inverse of image_to_array: rgb8 and bgr8 take RGB arrays (bgr8 is reordered), 32FC1 takes
    (H, W) or (H, W, 1), 32FC2 takes (H, W, 2). The data is little-endian and unpadded (step = row
    bytes).
    """
    dtype, channels = _layout(encoding)
    values = np.asarray(array)
    if values.ndim == 2 and channels == 1:
        values = values[..., np.newaxis]
    if values.ndim != 3 or values.shape[2] != channels:
        raise ValueError('array shape {} does not fit encoding {}'.format(values.shape, encoding))
    if encoding == 'bgr8':
        values = values[..., ::-1]
    height, width = values.shape[0], values.shape[1]
    wire = np.dtype('<f4') if dtype.kind == 'f' else np.dtype(np.uint8)
    data = np.ascontiguousarray(values, dtype=wire).tobytes()
    step = width * channels * dtype.itemsize
    return (height, width, encoding, step, data)
