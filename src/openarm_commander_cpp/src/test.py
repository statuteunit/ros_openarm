import pybullet as p
import pybullet_data
import os
from ament_index_python.packages import get_package_share_directory

# 连接物理服务器
physicsClient = p.connect(p.GUI)

# 设置内置模型路径
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# 1. 加载地面 (URDF)
planeId = p.loadURDF("plane.urdf")

description_pkg_share = get_package_share_directory('openarm_description')
urdf = os.path.join(description_pkg_share, 'urdf', 'robot', 'openarm_bimanual_cam.urdf')
# 2. 加载机器人 (URDF)
startPos = [0, 0, 0]
startOrientation = p.getQuaternionFromEuler([0, 0, 0])
robotId = p.loadURDF(urdf, startPos, startOrientation)

# 3. 加载物体 (SDF)
# objIds = p.loadSDF("my_object.sdf")

# 保持仿真运行
while True:
    p.stepSimulation()