import pinocchio as pin
# from pinocchio.robot_wrapper import RobotWrapper
import numpy as np
import matplotlib.pyplot as plt
import numpy as np

class loadRobotModel():
    def __init__(self, urdf_path):
        # 載入機器人模型
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()
        self.pos = pin.neutral(self.model)

    def calculateCom(self, plot=False):

        # Compute forward kinematics
        pin.forwardKinematics(self.model, self.data, self.pos)
        # print('q', self.pos)
        # Update frame placements
        pin.updateFramePlacements(self.model, self.data)

        com = pin.centerOfMass(self.model, self.data)
        plt.plot(com[0], com[2], marker = 'x')

        # J = pin.computeJointJacobians(self.model, self.data, self.pos)
        # print("J", J)

        
        com_lenth_vector = com - self.data.oMi[-1].translation
        com_lenth = (com_lenth_vector[0]**2+com_lenth_vector[2]**2)**(1/2)
        


        for name, oMi, inertia in zip(self.model.names, self.data.oMi, self.model.inertias):
            # print('trans', oMi.translation)
            if name=='universe':
                continue
            world_com = oMi.act(inertia.lever)  # Transform local CoM to world frame
            plt.scatter(oMi.translation[0], oMi.translation[2], label=name)
            plt.scatter(world_com[0], world_com[2], marker='*', label=name) 
            print('joint:', name, oMi.translation[0], oMi.translation[2]) 
            print('link_com:',name, world_com[0], world_com[2]) 
        
        if plot:
            plt.axis('equal')
            plt.legend()
            plt.show()
        
        return com, com_lenth

    def getOmi(self):
        for name, oMi in zip(self.model.names, self.data.oMi):
            if name=='L_calf2wheel' or name=='R_calf2wheel':
                return oMi.translation[0]
    
    def inverse_kinematics(self, x, y, L1=0.215, L2=0.215):
        # 計算 d    
        d = np.sqrt(x**2 + y**2)
        # 計算 theta2
        cos_theta2 = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)
        theta2 = np.arccos(cos_theta2)
        
        # 計算 theta1
        theta1 = np.arctan2(y, x) - np.arctan2(L2 * np.sin(theta2), L1 + L2 * np.cos(theta2))
    
        return theta1, theta2
    

    # def calculateInertia(self):
    #     body_inertia = 0
    #     wheel_inertia = 0
    #     wheel_joint = self.data.oMi[-1].translation
    #     for name, inertia, oMi in zip(self.model.names, self.model.inertias, self.data.oMi):
    #         if name=='wheel_joint_right' or name=='wheel_joint_left':
    #             wheel_inertia = inertia.inertia[1,1]
    #             link_com = oMi.translation + inertia.lever
    #         else:
    #             link_com = oMi.translation + inertia.lever
    #             dis = wheel_joint - link_com
    #             dis_xz = (dis[0]**2+dis[2]**2)**(1/2)
    #             print(inertia.inertia[1,1])
    #             print(inertia.mass)
    #             body_inertia += inertia.inertia[1,1] + inertia.mass*dis_xz**2

    #         plt.scatter(oMi.translation[0], oMi.translation[2]) 
    #         plt.scatter(link_com[0], link_com[2], marker='*')
            
        
    #     plt.axis('equal')
    #     plt.show()
    #     print("I", body_inertia, wheel_inertia)
    #     return body_inertia, wheel_inertia
    
    def calculateMass(self):
        body_mass = 0
        for name, inertia, oMi in zip(self.model.names, self.model.inertias, self.data.oMi):
                print(name, inertia.mass)
                body_mass += inertia.mass
        return body_mass
    

if __name__=="__main__":
    # urdf_path = "base_link_description/urdf/base_link.urdf"
    urdf_path = "crazydog/urdf/robotv2_6dof.urdf"
    robot = loadRobotModel(urdf_path=urdf_path)
    robot.pos = np.array([0., 0., 0., 0., 0., 0., 1., 
                            1.5, -2.3, 1., 0.,
                            1.5, -2.3, 1., 0.,])

    com, com_lenth = robot.calculateCom(plot=True)
    print('com', com, com_lenth)
    body_mass = robot.calculateMass()
    print('mass', body_mass)
    # position = [-0.0719649575357234, -0.13633336470173232]
    position = [-0.05882993992272667, -0.10034980934134337]
    theta1, theta2 = robot.inverse_kinematics(position[0]-(0.0014000000000026006), position[1]-(0.0646505434669769))
    robot.pos = np.array([0., 0., 0., 0., 0., 0., 1.,
                            -(theta1+1.57), -theta2, 1., 0.,
                            -(theta1+1.57), -theta2, 1., 0.])
    print(robot.pos)
    com1, com_lenth = robot.calculateCom(plot=True)
    print('com1', com1, com_lenth)
    print('mass:', robot.calculateMass())
    while 1:
        com = robot.calculateCom(plot=False)
        oMi = robot.getOmi()
        diff = oMi - com[0]
        print('diff', diff)
        if abs(diff[0])<0.00001:
            break
        position[0] -= diff[0]/2
        theta1, theta2 = robot.inverse_kinematics(position[0]-(0.0014000000000026006), position[1]-(0.0646505434669769))
        print('theta', theta1, theta2)
        robot.pos = np.array([0., 0., 0., 0., 0., 0., 1.,
                            -(theta1+1.57), -theta2, 1., 0.,
                            -(theta1+1.57), -theta2, 1., 0.])

    com4, com_lenth = robot.calculateCom(plot=True)
    print('final com', com4)
    print('final com length:', com_lenth)
    print('final angle:', -(theta1+1.57), -theta2)