import subprocess
import os
import argparse
import time
import signal
import shlex
# subprocess.call(["roscore"], shell=True)

# subprocess.call(["rosrun","nodelet","nodelet","standalone",
# "velodyne_pointcloud/CloudNodelet"])

cwd = os.getcwd()

parser = argparse.ArgumentParser(description='ROSBAG File: Cooking ROSBAGs')
parser.add_argument(
	'bag',
	type=str,
	help='Path to rosbag file'
)

args = parser.parse_args()
rosbag_file = args.bag

# subprocess.call(["rosrun rosbag record -O pointcloud.bag /velodyne_points"], shell =True)
#
# subprocess.call(["rosbag", "play", "6_f.bag"])
cmd_1 = "roscore"
cmd_2 = "rosrun nodelet nodelet standalone velodyne_ pointcloud/CloudNodelet"
cmd_3 = "rosrun rosbag record -O out_" + rosbag_file +" /velodyne_points"
cmd_3_l = ["x-term", "-e", "rosrun","rosbag","record","-O",("out_" + rosbag_file), "/velodyne_points"]
cmd_4 = "rosbag play " + rosbag_file
cmd_4_l = "rosbag","play", rosbag_file
# p1 = subprocess.Popen("exec " + cmd_1, shell=True)
# os.killpg(os.getpgid(pro.pid), signal.SIGTERM)
# time.sleep(3)
# p2 = subprocess.Popen("exec " + cmd_2, shell=False)
# time.sleep(3)
# process = subprocess.Popen(shlex.split("""gnome-terminal -x '"ls"'"""), shell=True,stdout=subprocess.PIPE)
# process.wait()
# print (process.returncode)
# p3 = subprocess.Popen(shlex.split("""x-terminal-emulator -e 'bash -c "rosrun rosbag record -O out_ """ + rosbag_file + """ /velodyne_points"'"""), stdout=subprocess.PIPE, shell=True)
# run3 = subprocess.call(cmd_3_l, shell=False)
# time.sleep(3)
# p4 = subprocess.Popen("exec " + cmd_4)
# run_4 = subprocess.Popen(cmd_4_l, shell=False)
# time.sleep(3)
# out1 = p1.communicate()[0]
# out3 = p3.communicate()[0]
# p1.kill()
# p2.kill()
# p3.kill()
# p4.kill()
try:
	# profile = subprocess.call(["source","~/.profile"], shell=True)
	profile = subprocess.Popen("source ~/.profile", shell=True)
	profile.wait()
except SyntaxError:
	print "source wasn't loaded this time"
print "here1"
# target_term = subprocess.call(["target_term", "-set", "1"], shell=True)
target_term = subprocess.Popen("target_term -set 2", shell=True)
print "here2"
# target_term.wait()
print "here3"
time.sleep(5)
run_roscore = subprocess.Popen("target_term -run 1 ls", shell=True)
run_roscore.wait()
time.sleep(3)
# end_roscore = subprocess.Popen("target_term -run 2 ls", shell=True)





print "ended"
