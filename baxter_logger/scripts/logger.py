#!/usr/bin/env python

import rospy
import rospkg
from lxml import etree

def go():
	print('We are here to make some noise!')



rospack = rospkg.RosPack()
path=rospack.get_path('baxter_logger')

print('The path is '+path)


root = etree.Element('config')
objects = etree.SubElement(root, 'objects')
etree.SubElement(objects,'types').text='2'

storage=etree.SubElement(root,'storage_containers')

etree.SubElement(storage,'number').text='2'

container1=etree.SubElement(storage,'container',id='1')
pose1=etree.SubElement(container1,'pose')

position1=etree.SubElement(pose1,'position')
etree.SubElement(position1,'x').text='1.2'
etree.SubElement(position1,'y').text='1'
etree.SubElement(position1,'z').text='2'

orientation1=etree.SubElement(container1,'orientation')
etree.SubElement(orientation1,'x').text='0.7'
etree.SubElement(orientation1,'y').text='0.98'
etree.SubElement(orientation1,'z').text='0.02'
etree.SubElement(orientation1,'w').text='0.001'


container2=etree.SubElement(storage,'container',id='2')
pose2=etree.SubElement(container2,'pose')

position2=etree.SubElement(pose2,'position')
etree.SubElement(position2,'x').text='1.9'
etree.SubElement(position2,'y').text='10'
etree.SubElement(position2,'z').text='19'

orientation2=etree.SubElement(container2,'orientation')
etree.SubElement(orientation2,'x').text='0.7'
etree.SubElement(orientation2,'y').text='0.98'
etree.SubElement(orientation2,'z').text='0.02'
etree.SubElement(orientation2,'w').text='0.001'



go()

tree = etree.ElementTree(root)
tree.write(path+'/config_result/config.xml')



