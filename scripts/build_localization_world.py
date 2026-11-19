#!/usr/bin/env python3
"""Build a self-contained BLE/IMU world from the robot template."""
import json
from pathlib import Path
import xml.etree.ElementTree as ET
ROOT=Path(__file__).resolve().parents[1]
c=json.loads((ROOT/'config/localization.json').read_text())
tree=ET.parse(ROOT/'gazebo/robot_base.sdf'); world=tree.getroot().find('world')
world.set('name','localization_lab')
for ns in world.findall('.//namespace'): ns.text='/localization_demo'
robot=world.find("model[@name='localization_robot']"); robot.set('name','localization_robot')
# Put the native sensor at the robot center with aligned body axes.
base=robot.find("link[@name='base_link']")
sensor=ET.SubElement(base,'sensor',name='imu_sensor',type='imu')
for tag,value in [('always_on','true'),('update_rate','50'),('pose','0 0 0 0 0 0')]: ET.SubElement(sensor,tag).text=value
imu=ET.SubElement(sensor,'imu')
for kind,std in [('angular_velocity',c['imu']['gyro_sigma_radps']),('linear_acceleration',c['imu']['accel_sigma_mps2'])]:
    parent=ET.SubElement(imu,kind)
    for axis in ['x','y','z']:
        noise=ET.SubElement(ET.SubElement(parent,axis),'noise',type='gaussian')
        ET.SubElement(noise,'mean').text='0'; ET.SubElement(noise,'stddev').text=str(std)
plugin=ET.SubElement(sensor,'plugin',name='native_imu',filename='libgazebo_ros_imu_sensor.so')
ros=ET.SubElement(plugin,'ros'); ET.SubElement(ros,'namespace').text='/localization_demo'
ET.SubElement(ros,'remapping').text='~/out:=imu_native'
ET.SubElement(plugin,'frame_name').text='base_link'
ET.SubElement(plugin,'initial_orientation_as_reference').text='false'
for b in c['beacons']:
    model=ET.SubElement(world,'model',name='beacon_'+b['id']); ET.SubElement(model,'static').text='true'
    ET.SubElement(model,'pose').text=' '.join(map(str,b['xyz']))+' 0 0 0'
    link=ET.SubElement(model,'link',name='beacon')
    for kind in ['visual','collision']:
        item=ET.SubElement(link,kind,name=kind); geometry=ET.SubElement(item,'geometry')
        ET.SubElement(ET.SubElement(geometry,'sphere'),'radius').text='.12'
        if kind=='visual':
            material=ET.SubElement(item,'material'); ET.SubElement(material,'ambient').text='0.1 0.7 0.4 1'; ET.SubElement(material,'diffuse').text='0.1 0.7 0.4 1'
# Floor grid in RViz provides the surveyed room frame. No downloaded models.
ET.indent(tree,space='  '); tree.write(ROOT/'gazebo/localization_lab.world',encoding='unicode',xml_declaration=True)
print(ROOT/'gazebo/localization_lab.world')
