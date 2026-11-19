#!/usr/bin/env python3
"""Read-only runtime inventory. Use the ROS interpreter inside devbox."""
import argparse
import importlib.util
import json
from pathlib import Path
import platform
import shutil
import subprocess
import sys
p=argparse.ArgumentParser(); p.add_argument('--output',type=Path); a=p.parse_args()
result={'python':sys.version,'executable':sys.executable,'platform':platform.platform(),
        'os_release':Path('/etc/os-release').read_text(),
        'modules':{m:importlib.util.find_spec(m) is not None for m in ['numpy','matplotlib','rclpy']},
        'commands':{m:shutil.which(m) for m in ['gzserver','gzclient','ros2','rviz2','distrobox']}}
if shutil.which('gzserver'):
    r=subprocess.run(['gzserver','--version'],capture_output=True,text=True,timeout=10)
    result['gazebo_version']=r.stdout+r.stderr
print(json.dumps(result,indent=2))
if a.output:
    a.output.parent.mkdir(parents=True,exist_ok=True); a.output.write_text(json.dumps(result,indent=2)+'\n')
