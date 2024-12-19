import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper
import os
import sympy as sp

# Define the path to your URDF model
urdf_path = "inverted_pendulum.urdf"  # Update this path
model_dir = os.path.dirname(urdf_path)

# Load the URDF model
robot = RobotWrapper.BuildFromURDF(urdf_path, [model_dir])

# Define symbolic variables for state and input
q, dq, u = sp.symbols('q dq u')
m, l, I, g = sp.symbols('m l I g')

# Define the state vector
x = sp.Matrix([q, dq])

# Dynamics equations
ddq = (1/I) * (u - m * g * l * sp.sin(q))

# State space representation
dx = sp.Matrix([dq, ddq])

# Compute the Jacobians to get A and B matrices
A = dx.jacobian(x)
B = dx.jacobian(sp.Matrix([u]))

# Substitute numerical values for parameters (if needed)
parameters = {m: 1.0, l: 1.0, I: 1.0, g: 9.81}
A_num = A.subs(parameters)
B_num = B.subs(parameters)

# Evaluate at the upright equilibrium point (q = 0, dq = 0)
equilibrium_point = {q: 0, dq: 0}

A_eq = A_num.subs(equilibrium_point)
B_eq = B_num.subs(equilibrium_point)

print("A matrix:")
print(A_eq)

print("\nB matrix:")
print(B_eq)
