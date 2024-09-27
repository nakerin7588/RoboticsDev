from math import pi
import roboticstoolbox as rtb
from spatialmath import SE3

mdh = [[0.0, 0.0, 0.2, pi/2.0], [0.0, pi/2.0, 0.12, pi/2.0], [0.25, 0.0, -0.1, -pi/2.0]]
revjoint = [] # Create revolute joint
for i, data in enumerate(mdh):
    revjoint.append(rtb.RevoluteMDH(a=data[0], alpha=data[1], d=data[2], offset=data[3])) # Append revolute joint
robot = rtb.DHRobot(
    [
        revjoint[0],
        revjoint[1],
        revjoint[2]
    ]
    ,tool = SE3.Rx(-pi/2) * SE3.Tz(0.28)
    , name="3R robot"
)
# Show robot
print(robot)
robot.plot([0.0001, 0.0001, 0.0001], block=True)