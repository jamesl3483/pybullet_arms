import pybullet as p
import pybullet_data
import numpy as np
from ..mano_model import ManoModel

# Connect to PyBullet
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.resetSimulation()
p.setGravity(0, 0, -9.81)

# Load ground plane
plane_id = p.loadURDF("plane.urdf")

# Load a cylinder
cylinder_id = p.createCollisionShape(p.GEOM_CYLINDER, radius=0.05, height=0.2)
cylinder_visual_id = p.createVisualShape(p.GEOM_CYLINDER, radius=0.05, length=0.2, rgbaColor=[1, 0, 0, 1])
cylinder_body_id = p.createMultiBody(baseCollisionShapeIndex=cylinder_id, baseVisualShapeIndex=cylinder_visual_id,
                                     basePosition=[0, 0, 0.1])

# Initialize MANO hand model
mano = ManoModel(left_hand=False)
mano_hand =

# Simulation parameters
steps = 1000

for step in range(steps):
    p.stepSimulation()
    p.setRealTimeSimulation(1)

# Keep simulation window open
input("Press Enter to exit...")
p.disconnect()
