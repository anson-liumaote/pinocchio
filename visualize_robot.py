import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer
import os
import numpy as np

# Load the URDF model
model_path = os.path.dirname(os.path.abspath(__file__))
urdf_path = os.path.join(model_path, "bi_urdf/urdf/bi_urdf.urdf")
model, collision_model, visual_model = pin.buildModelsFromUrdf(urdf_path, model_path)

# Create data required for the algorithms
data = model.createData()

# Create the visualize
viz = MeshcatVisualizer(model, collision_model, visual_model)

# Load the robot model in the visualizer
viz.initViewer()
viz.loadViewerModel()

# Display the robot at its initial configuration
q0 = pin.neutral(model)
# q0 =  np.array([0., 0., 0., 0., 0., 0., 1., 
#                 1.27, -2.0, 1., 0.,
#                 1.27, -2.0, 1., 0.,])
# q0 = pin.utils.zero(model.nq)1
viz.display(q0)
print(q0)

input("Press Enter to exit...")  # Keep the visualizer open
