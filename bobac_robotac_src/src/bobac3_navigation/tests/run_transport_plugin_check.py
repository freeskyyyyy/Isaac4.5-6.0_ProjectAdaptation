"""Compile and run installed Jazzy algorithms without creating ROS nodes."""
import argparse
from pathlib import Path
import subprocess

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output', type=Path, default=Path('/tmp/bobac-nav-c/transport_plugin_check'))
args = parser.parse_args()
args.output.parent.mkdir(parents=True, exist_ok=True)
source = Path(__file__).with_name('transport_plugin_check.cpp')
command = ['g++', '-std=c++17', '-O0', str(source), '-o', str(args.output), '-I/opt/ros/jazzy/include']
command += ['-I'+str(p) for p in Path('/opt/ros/jazzy/include').iterdir() if p.is_dir()]
command += ['-I/usr/include/eigen3']
command += ['-I'+str(p) for p in Path('/usr/include').glob('ompl-*')]
command += ['-L/opt/ros/jazzy/lib', '-Wl,-rpath,/opt/ros/jazzy/lib', '-lnav2_smac_planner',
            '-lmppi_critics', '-lnav2_costmap_2d_core', '-lrclcpp', '-pthread']
subprocess.run(command, check=True)
subprocess.run([str(args.output)], check=True)
