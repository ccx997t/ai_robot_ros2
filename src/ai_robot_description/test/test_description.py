from pathlib import Path
import xml.etree.ElementTree as ET

import xacro


XACRO_FILE = Path(__file__).parents[1] / 'urdf' / 'ai_robot.urdf.xacro'


def expanded_robot():
    return ET.fromstring(xacro.process_file(str(XACRO_FILE)).toxml())


def test_model_has_required_links_and_joints():
    robot = expanded_robot()
    links = {element.attrib['name'] for element in robot.findall('link')}
    joints = {element.attrib['name']: element for element in robot.findall('joint')}

    assert {'base_footprint', 'base_link', 'left_wheel_link', 'right_wheel_link',
            'caster_link', 'sensor_link'} <= links
    assert joints['left_wheel_joint'].attrib['type'] == 'continuous'
    assert joints['right_wheel_joint'].attrib['type'] == 'continuous'
    assert joints['sensor_joint'].find('parent').attrib['link'] == 'base_link'


def test_physical_links_have_positive_mass_and_collision():
    robot = expanded_robot()
    physical_links = {'base_link', 'left_wheel_link', 'right_wheel_link', 'caster_link'}
    for link in robot.findall('link'):
        if link.attrib['name'] not in physical_links:
            continue
        assert link.find('collision') is not None
        mass = float(link.find('inertial/mass').attrib['value'])
        assert mass > 0.0


def test_wheel_geometry_contract():
    robot = expanded_robot()
    joints = {element.attrib['name']: element for element in robot.findall('joint')}
    left_y = float(joints['left_wheel_joint'].find('origin').attrib['xyz'].split()[1])
    right_y = float(joints['right_wheel_joint'].find('origin').attrib['xyz'].split()[1])
    assert left_y == 0.17
    assert right_y == -0.17
    assert joints['left_wheel_joint'].find('axis').attrib['xyz'] == '0 1 0'
    assert joints['right_wheel_joint'].find('axis').attrib['xyz'] == '0 1 0'
