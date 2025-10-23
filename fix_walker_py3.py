#!/usr/bin/env python3
import re

walker_path = '/home/ramsesmc1906/ros-darwin/src/darwin_gazebo/scripts/walker.py'

with open(walker_path, 'r') as f:
    content = f.read()

# Fix print statements
content = content.replace('print j,"p",self.pfn[j],"a",self.afn[j]', 'print(j,"p",self.pfn[j],"a",self.afn[j])')
content = content.replace('print "cmdvel",msg', 'print("cmdvel",msg)')

with open(walker_path, 'w') as f:
    f.write(content)

print("Fixed walker.py for Python3")
