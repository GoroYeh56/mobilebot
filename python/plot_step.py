import sys
import os
import numpy as np
import matplotlib.pyplot as plt
sys.path.append("lcmtypes")
import lcm
from lcmtypes import mbot_wheel_ctrl_t

if len(sys.argv) < 2:
    sys.stderr.write("usage: plot_step.py <logfile>")
    sys.exit(1)

file = sys.argv[1]
log = lcm.EventLog(file,"r")

data = np.empty((0,7), dtype=float)
init = 0
for event in log:
    if event.channel == "MBOT_WHEEL_CTRL":
        msg = mbot_wheel_ctrl_t.decode(event.data)
        if init==0:
            start_utime = msg.utime
            init = 1
        data = np.append(data, np.array([[ \
            (msg.utime-start_utime)/1.0E6, \
            msg.left_motor_pwm_cmd, \
            msg.right_motor_pwm_cmd, \
            msg.left_motor_vel_cmd, \
            msg.right_motor_vel_cmd, \
            msg.left_motor_vel, \
            msg.right_motor_vel
            ]]), axis=0)
cmd = data[:,3]
start = np.where(cmd>=0.1)[0][0]-5
end = start + 500 #plot 10s worth of data
p1 = plt.plot(data[start:end,0], data[start:end,3], 'g', label="vel cmd")
p2 = plt.plot(data[start:end,0], data[start:end,1], 'r', label="pwm cmd")
p3 = plt.plot(data[start:end,0], data[start:end,5], 'b', label="velocity")
plt.legend(loc="upper right")
filename, file_extension = os.path.splitext(file)
plt.savefig(filename + '.png')
plt.show()
