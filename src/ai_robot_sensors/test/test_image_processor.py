import pytest
from sensor_msgs.msg import Image

from ai_robot_sensors.image_processor import rgb8_to_mono8


def image(width, height, step, data, encoding='rgb8'):
    message = Image()
    message.width = width
    message.height = height
    message.step = step
    message.encoding = encoding
    message.header.frame_id = 'camera_optical_link'
    message.header.stamp.sec = 12
    message.data = bytes(data)
    return message


def test_rgb8_to_mono8_preserves_geometry_header_and_handles_padding():
    source = image(2, 1, 8, [255, 0, 0, 0, 255, 0, 99, 99])

    result = rgb8_to_mono8(source)

    assert result.encoding == 'mono8'
    assert (result.width, result.height, result.step) == (2, 1, 2)
    assert list(result.data) == [76, 149]
    assert result.header.frame_id == 'camera_optical_link'
    assert result.header.stamp.sec == 12


@pytest.mark.parametrize('encoding', ['mono8', 'bgr8'])
def test_rgb8_to_mono8_rejects_wrong_encoding(encoding):
    with pytest.raises(ValueError, match='expected rgb8'):
        rgb8_to_mono8(image(1, 1, 3, [0, 0, 0], encoding))


def test_rgb8_to_mono8_rejects_short_data():
    with pytest.raises(ValueError, match='shorter'):
        rgb8_to_mono8(image(2, 1, 6, [0, 0, 0]))
