import math
import time
import matplotlib.pyplot as plt
import numpy as np
from numpy.linalg import inv, eig
import pinocchio as pin
import urdf_loader

class InvertedPendulumLQR:
    # def __init__(self, hip, knee, l_bar=3.0, M=0.48, m=2*(0.06801+0.07172)+0.45376, g=9.8, Q=None, R=None, delta_t=1/50, sim_time=15.0, show_animation=True):
    def __init__(self,l_bar=None, M=0.28, m=None, g=9.8, Q=None, R=None, delta_t=1/150, sim_time=3.0, show_animation=True):    
    # transform isaac sim angle to com.py angle
        urdf_path = "crazydog/urdf/robotv2_6dof.urdf"
        robot = urdf_loader.loadRobotModel(urdf_path=urdf_path)
        robot.pos = np.array([0., 0., 0., 0., 0., 0., 1., 
                                1.321, -2.134, 1., 0.,
                                1.321, -2.134, 1., 0.,])
        self.com, self.l_bar = robot.calculateCom(plot=False)
        print(self.l_bar)
        self.M = M  # mass of the cart [kg]self.R = R if R is not None else np.diag([0.1])  # input cost matrix
        self.m = robot.calculateMass()  # mass of the pendulum [kg]
        self.g = g  # gravity [m/s^2]
        self.nx = 4  # number of states
        self.nu = 1  # number of inputs
        self.wheel_r = 0.07
        self.Q = Q if Q is not None else np.diag([0, 1.0, 1.0, 1.0])  # state cost matrix , best in IsaacSim
        self.R = R if R is not None else np.diag([1e-6])  # input cost matrix

        # self.Q = Q if Q is not None else np.diag([0.1, 0.001, 30.0, 0.0])  # state cost matrix
        # self.R = R if R is not None else np.diag([0.001])  # input cost matrix

        self.delta_t = delta_t  # time tick [s]
        self.sim_time = sim_time  # simulation time [s]

        self.show_animation = show_animation

        self.A, self.B = self.get_model_matrix()
        self.K, _, _ = self.dlqr(self.A, self.B, self.Q, self.R)
        print('mass', self.m)
        print('K,', self.K)

    def main(self):
        x0 = np.array([
            [0.0],
            [0.0],
            [-0.1],
            [0.0]
        ])

        X = np.copy(x0)
        time_elapsed = 0.0

        time_list = []
        x00_list = []
        x10_list = []
        x20_list = []
        x30_list = []
        u_list = []
        while self.sim_time > time_elapsed:
        # while True:
            time_elapsed += self.delta_t
            
            # calculate control input
            u = self.lqr_control(X)
            X = self.simulation(X, u)

            # time_list.append(time_elapsed)
            time_list.append(time_elapsed)
            # u_list.append(u[0, 0])
            x00_list.append(X[0, 0])
            x10_list.append(X[1, 0])
            x20_list.append(X[2, 0])
            x30_list.append(X[3, 0])
            u_list.append(u[0, 0])

        print("Finish")
        print(f"x={float(X[0, 0]):.2f} [m] , theta={math.degrees(X[2, 0]):.2f} [deg]")
        if self.show_animation:
            # Plotting
            fig, axs = plt.subplots(2, 1, figsize=(10, 8))

            # Plot x0
            axs[0].plot(x00_list, label='x')
            axs[0].plot(x10_list, label='x_dot')
            axs[0].plot(x20_list, label='theta')
            axs[0].plot(x30_list, label='theta_dot')
            axs[0].set_title('State over time')
            axs[0].set_xlabel('Iteration')
            axs[0].set_ylabel('State')
            axs[0].legend()
            axs[0].grid(True)


            # Plot ctrl
            axs[1].plot(u_list, label='ctrl', color='orange')
            axs[1].set_title('Control over time')
            axs[1].set_xlabel('Iteration')
            axs[1].set_ylabel('Control Input')
            axs[1].legend()
            axs[1].grid(True)


            # Save the plot as an image file
            plt.tight_layout()
            plt.show()
    



    def simulation(self, x, u):
        X_dot = self.A @ x + self.B @ u

        return X_dot

    def solve_DARE(self, A, B, Q, R, maxiter=150, eps=0.01):
        """
        Solve a discrete time Algebraic Riccati equation (DARE)
        """
        P = Q

        for i in range(maxiter):
            Pn = A.T @ P @ A - A.T @ P @ B @ \
                inv(R + B.T @ P @ B) @ B.T @ P @ A + Q
            if (abs(Pn - P)).max() < eps:
                break
            P = Pn

        return Pn

    def dlqr(self, A, B, Q, R):
        """
        Solve the discrete time lqr controller.
        x[k+1] = A x[k] + B u[k]
        cost = sum x[k].T*Q*x[k] + u[k].T*R*u[k]
        """

        # first, try to solve the ricatti equation
        P = self.solve_DARE(A, B, Q, R)

        # compute the LQR gain
        K = inv(B.T @ P @ B + R) @ (B.T @ P @ A)

        eigVals, eigVecs = eig(A - B @ K)
        return K, P, eigVals

    def lqr_control(self, x):
        start = time.time()
        u = -self.K @ x
        # elapsed_time = time.time() - start
        # print(f"calc time:{elapsed_time:.6f} [sec]")
        return u

    def get_model_matrix(self):
        # A = np.array([
        #     [0.0, 1.0, 0.0, 0.0],
        #     [0.0, 0.0, self.m * self.g / self.M, 0.0],
        #     [0.0, 0.0, 0.0, 1.0],
        #     [0.0, 0.0, self.g * (self.M + self.m) / (self.l_bar * self.M), 0.0]
        # ])
        # print('A=',A)
        Jz = (1/3) * self.m * self.l_bar**2
        I = (1/2) * self.M * self.wheel_r**2
        Q_eq = Jz * self.m + (Jz + self.m * self.l_bar * self.l_bar) * \
            (2 * self.M + (2 * I) / (self.wheel_r**2))
        A_23 = -(self.m**2)*(self.l_bar**2)*self.g / Q_eq
        A_43 = self.m*self.l_bar*self.g * \
            (self.m+2*self.M+(2*I/(self.wheel_r**2)))/Q_eq
        A = np.array([
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, A_23, 0.0],
            [0.0, 0.0, 0.0, 1.0],
            [0.0, 0.0, A_43, 0.0]
        ])
        A = np.eye(self.nx) + self.delta_t * A

        # B = np.array([
        #     [0.0],
        #     [1.0 / self.M],
        #     [0.0],
        #     [1.0 / (self.l_bar * self.M)]
        # ])
        # print('B=',B)
        B_21 = (Jz+self.m*self.l_bar**2+self.m *
                self.l_bar*self.wheel_r)/Q_eq/self.wheel_r
        B_41 = -((self.m*self.l_bar/self.wheel_r)+self.m +
                 2*self.M+(2*I/(self.wheel_r**2)))/Q_eq
        B = np.array([
            [0.0],
            [2*B_21],
            [0.0],
            [2*B_41]
        ])
        B = self.delta_t * B

        return A, B

if __name__ == '__main__':
    
    # for i in range(1, 11):
    Q = np.diag([100., 10., 100.0, 0.1])
    R = np.diag([25.]) 
    lqr = InvertedPendulumLQR(Q = Q, R=R)
    lqr.main()

    
