import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import pytest
from sensor_msgs.msg import Imu, LaserScan

from ai_robot_sensors.fault_injector import inject_fault


def test_drop_models_sensor_and_command_interruption():
    assert inject_fault(LaserScan(), 'scan', 'drop') is None
    assert inject_fault(Twist(), 'twist', 'drop') is None


def test_zero_stamp_and_bad_frame_do_not_mutate_source():
    source = LaserScan()
    source.header.stamp.sec = 10
    source.header.frame_id = 'laser_link'
    zero_stamp = inject_fault(source, 'scan', 'zero_stamp')
    bad_frame = inject_fault(source, 'scan', 'bad_frame', 'invalid_laser')
    assert (zero_stamp.header.stamp.sec, zero_stamp.header.stamp.nanosec) == (0, 0)
    assert bad_frame.header.frame_id == 'invalid_laser'
    assert source.header.stamp.sec == 10
    assert source.header.frame_id == 'laser_link'


@pytest.mark.parametrize(
    ('message', 'message_type', 'invalid_value'),
    [
        (LaserScan(ranges=[1.0]), 'scan', lambda item: item.ranges[0]),
        (Imu(), 'imu', lambda item: item.angular_velocity.x),
        (Odometry(), 'odometry', lambda item: item.pose.covariance[0]),
        (Twist(), 'twist', lambda item: item.linear.x),
    ],
)
def test_nonfinite_data_covers_sensor_fusion_and_command_inputs(
        message, message_type, invalid_value):
    output = inject_fault(message, message_type, 'nonfinite_data')
    assert math.isnan(invalid_value(output))


def test_invalid_combinations_are_rejected():
    with pytest.raises(ValueError, match='requires a message with a header'):
        inject_fault(Twist(), 'twist', 'bad_frame')
    with pytest.raises(ValueError, match='unsupported fault_mode'):
        inject_fault(Imu(), 'imu', 'unknown')
