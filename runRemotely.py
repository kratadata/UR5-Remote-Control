import sys
sys.path.append('')
import URBasic
import time
import math
from min_jerk_planner_translation import PathPlanTranslation

ROBOT_IP = '192.168.19.1'
ACCELERATION = 0.9  # Robot acceleration value
VELOCITY = 0.8  # Robot speed value

start_pose = [-0.18507570121045797, -0.43755157063468963, 0.21101969081827837, -0.06998478570599498, -3.0949971695297402, 0.10056260631290592]
desired_pose = [-0.41227681851594755, -0.553539320093064, 0.07077025734923525, -0.06990025901302169, -3.0949715741835195, 0.10065200008528846]
orientation_const = start_pose[3:]
trajectory_time = 8  # time of min_jerk trajectory
dt = 1/500  # 500 Hz    # frequency
planner = PathPlanTranslation(start_pose, desired_pose, trajectory_time)


def set_lookorigin():
    """
    Return Value:
        orig: math3D Transform Object
            characterises location and rotation of the new coordinate system in reference to the base coordinate system

    """
    position = robot.get_actual_tcp_pose()
    orig = m3d.Transform(position)
    return orig


print("initialising robot")
robotModel = URBasic.robotModel.RobotModel()
print("initialising model robot")
robot = URBasic.urScriptExt.UrScriptExt(host=ROBOT_IP, robotModel=robotModel, conf_filename='rtdeConfigurationDefault.xml')

robot.reset_error()
print("robot initialised")
time.sleep(1)

robot_startposition = robot.get_actual_tcp_pose()
robot.movej(q=start_pose, a= ACCELERATION, v= VELOCITY )
origin = set_lookorigin()

robot.init_realtime_control()  # starts the realtime control loop on the Universal-Robot Controller
time.sleep(1) # just a short wait to make sure everything is initialised
t_current = 0
t_start = time.time()

try:
    print("starting loop")
    while time.time() - t_start < trajectory_time:
        t_init = time.time()
        t_prev = t_current
        t_current = time.time() - t_start
        if t_current <= trajectory_time:
            # ------------------ impedance -----------------------
            [position_ref, lin_vel_ref, acceleration_ref] = planner.trajectory_planning(t_current)
            pose = position_ref.tolist() + orientation_const
            robot.servoj(pose)

    print("exiting loop")

except KeyboardInterrupt:
    print("closing robot connection")
    robot.close()

except:
    robot.close()