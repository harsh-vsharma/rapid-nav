"""Tests of the image conversions, with a stand-in for sensor_msgs/Image."""

import numpy as np
import pytest

from rapid_nav_perception.conversions import array_to_image_fields, image_to_array


class FakeImage:
    """Carries the sensor_msgs/Image fields."""

    def __init__(self, height, width, encoding, step, data, is_bigendian=0):
        """Store the fields."""
        self.height = height
        self.width = width
        self.encoding = encoding
        self.step = step
        self.data = data
        self.is_bigendian = is_bigendian


def as_message(array, encoding):
    """Return a FakeImage made from an array."""
    height, width, enc, step, data = array_to_image_fields(array, encoding)
    return FakeImage(height, width, enc, step, data)


def test_round_trip_rgb8():
    """rgb8 survives a round trip unchanged."""
    rng = np.random.default_rng(1)
    array = rng.integers(0, 256, size=(5, 7, 3), dtype=np.uint8)
    back = image_to_array(as_message(array, 'rgb8'))
    assert back.dtype == np.uint8
    assert np.array_equal(back, array)


def test_round_trip_bgr8_comes_back_as_rgb():
    """bgr8 is reordered on the way in and out, so the RGB array survives."""
    array = np.zeros((2, 2, 3), dtype=np.uint8)
    array[..., 0] = 200
    message = as_message(array, 'bgr8')
    assert message.data[0] == 0  # the first byte is blue
    assert np.array_equal(image_to_array(message), array)


def test_round_trip_32fc1():
    """Depth survives a round trip as float32 with shape (H, W)."""
    array = np.linspace(0.5, 9.5, 12, dtype=np.float32).reshape(3, 4)
    back = image_to_array(as_message(array, '32FC1'))
    assert back.shape == (3, 4)
    assert np.allclose(back, array)


def test_round_trip_32fc2():
    """Flow survives a round trip as float32 with shape (H, W, 2)."""
    rng = np.random.default_rng(2)
    array = rng.normal(size=(4, 5, 2)).astype(np.float32)
    back = image_to_array(as_message(array, '32FC2'))
    assert back.shape == (4, 5, 2)
    assert np.allclose(back, array)


def test_padded_step_is_honoured():
    """Bytes past the end of each row (the padding) are ignored."""
    array = np.arange(2 * 3 * 3, dtype=np.uint8).reshape(2, 3, 3)
    row_bytes = 9
    step = 12
    padded = bytearray()
    for row in array.reshape(2, row_bytes):
        padded += bytes(row) + b'\xff' * (step - row_bytes)
    message = FakeImage(2, 3, 'rgb8', step, bytes(padded))
    assert np.array_equal(image_to_array(message), array)


def test_big_endian_floats_are_read_correctly():
    """is_bigendian selects the byte order of float data."""
    values = np.array([[1.5, -2.25]], dtype='>f4')
    message = FakeImage(1, 2, '32FC1', 8, values.tobytes(), is_bigendian=1)
    assert np.allclose(image_to_array(message), [[1.5, -2.25]])


def test_unknown_encoding_raises():
    """An encoding outside the four supported ones is an error."""
    message = FakeImage(1, 1, 'mono16', 2, b'\x00\x00')
    with pytest.raises(ValueError):
        image_to_array(message)
    with pytest.raises(ValueError):
        array_to_image_fields(np.zeros((1, 1)), 'mono16')
