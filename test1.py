import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper
import numpy as np
import matplotlib.pyplot as plt


def load_model():
    # 載入機器人模型
    urdf_path = "wheel_leg_description/urdf/wheel_leg.urdf"
    # robot = RobotWrapper.BuildFromURDF(urdf_path)
    model = pin.buildModelFromUrdf(urdf_path)
    data = model.createData()

    # 初始化位置和速度
    # q = pin.neutral(model)
    q = pin.utils.zero(model.nq)
    q[0] = 10
    q[2] = 10
    # q[1] = 2
    # q = pin.randomConfiguration(model)
    v = pin.utils.zero(model.nv)
    a = pin.utils.zero(model.nv)

    # 計算質量矩陣和重力項
    M = pin.crba(model, data, q)
    g = pin.computeGeneralizedGravity(model, data, q)


    print('inertia', model.inertias[0])
    print('inertia', model.inertias[1])
    print('inertia', model.inertias[2])
    print("q", q)
    print("v", v)
    print("M", M)
    print("g", g)

    # 計算雅可比矩陣
    J = pin.computeJointJacobians(model, data, q)
    print("J", J)

    # 計算科里奧利項
    C = pin.computeCoriolisMatrix(model, data, q, v)
    print("C", C)

    # 初始化A和B矩陣
    nv = model.nv
    nu = model.nq

    print(nv, nu)
    com = pin.centerOfMass(model, data)
    print(com)
    tau = pin.rnea(model, data, q, v, a)
    print('tau', tau)

    for name, oMi in zip(model.names, data.oMi):
        print(("{:<24} : {: .2f} {: .2f} {: .2f}".format(name, *oMi.translation.T.flat)))
    


if __name__=="__main__":
    load_model()