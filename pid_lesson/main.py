from robot import Robot
import matplotlib.pyplot as plt
import numpy as np
from twiddle import twiddle, make_robot, run



def run_pid(robot, tau_p, tau_d, tau_i, n=200, speed=1.0):
    x_trajectory = []
    y_trajectory = []
    cte_prev = robot.y
    cte_sum = 0
    for _ in range(n):
        cte = robot.y
        cte_sum += cte
        delta_cte = cte - cte_prev
        cte_prev = cte
        # print('current cte ', cte, '\nprev cte ', cte_prev, '\ndelta cte ', delta_cte)
        # print('cte sum ', cte_sum)
        steer = - tau_p * cte - tau_d * delta_cte - tau_i * cte_sum
        robot.move(steer, speed)
        # print('current x ', robot.x, '\ncurrent y ', robot.y)
        x_trajectory.append(robot.x)
        y_trajectory.append(robot.y)          
    
    return x_trajectory, y_trajectory
 


if __name__ == "__main__":
    is_p_controller_enabled = False
    if is_p_controller_enabled:
        ### P Controller
        # Basic Idea：Decrease Error
        # Problem：P Control Never Really Converge
        # Larger tau p, oscillate faster
        robot = Robot()
        robot.set(0, 1, 0)
        x_trajectory, y_trajectory = run_pid(robot, 0.1, 0.0, 0.0)

        robot_2 = Robot()
        robot_2.set(0, 1, 0)
        x_trajectory_2, y_trajectory_2 = run_pid(robot_2, 0.5, 0.0, 0.0)

        robot_3 = Robot()
        robot_3.set(0, 1, 0)
        x_trajectory_3, y_trajectory_3 = run_pid(robot_3, 0.05, 0.0, 0.0)


        n = len(x_trajectory)
        _, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 15))
        
        ax1.plot(x_trajectory, y_trajectory, 'g', label='P controller')
        ax1.plot(x_trajectory, np.zeros(n), 'r', label='reference')
        ax1.legend()
        ax1.set_title('P Control with tau 0.1',fontsize=12,color='k')

        ax2.plot(x_trajectory_2, y_trajectory_2, 'g', label='P controller')
        ax2.plot(x_trajectory_2, np.zeros(n), 'r', label='reference')
        ax2.legend()
        ax2.set_title('P Control with tau 0.5',fontsize=12,color='k')

        ax3.plot(x_trajectory_3, y_trajectory_3, 'g', label='P controller')
        ax3.plot(x_trajectory_3, np.zeros(n), 'r', label='reference')
        ax3.legend()
        ax3.set_title('P Control with tau 0.05',fontsize=12,color='k')


    is_pd_controller_enabled = False
    if is_pd_controller_enabled:
        ### PD Controller
        # Basic Idea：Avoid Overshoot
        robot_4 = Robot()
        robot_4.set(0, 1, 0)
        x_trajectory_4, y_trajectory_4 = run_pid(robot_4, 0.2, 1.0, 0.0)

        robot_5 = Robot()
        robot_5.set(0, 1, 0)
        x_trajectory_5, y_trajectory_5 = run_pid(robot_5, 0.2, 3.0, 0.0)

        robot_6 = Robot()
        robot_6.set(0, 1, 0)
        x_trajectory_6, y_trajectory_6 = run_pid(robot_6, 0.2, 5.0, 0.0)


        n2 = len(x_trajectory_4)
        _, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 15))
        
        ax1.plot(x_trajectory_4, y_trajectory_4, 'g', label='PD controller')
        ax1.plot(x_trajectory_4, np.zeros(n2), 'r', label='reference')
        ax1.legend()
        ax1.set_title('PD Control with tau_p 0.2, tau_d 1.0',fontsize=12,color='k')

        ax2.plot(x_trajectory_5, y_trajectory_5, 'g', label='PD controller')
        ax2.plot(x_trajectory_5, np.zeros(n2), 'r', label='reference')
        ax2.legend()
        ax2.set_title('PD Control with tau_p 0.2, tau_d 3.0',fontsize=12,color='k')

        ax3.plot(x_trajectory_6, y_trajectory_6, 'g', label='PD controller')
        ax3.plot(x_trajectory_6, np.zeros(n2), 'r', label='reference')
        ax3.legend()
        ax3.set_title('PD Control with tau_p 0.2, tau_d 5.0',fontsize=12,color='k')
    

    is_system_bias_enabled = False
    if is_system_bias_enabled:
        ### P and PD Controller Can't solve Systematic Bias
        robot_7 = Robot()
        robot_7.set(0, 1, 0)
        robot_7.set_steering_drift(10.0 / 180.0 * np.pi)
        x_trajectory_7, y_trajectory_7 = run_pid(robot_7, 0.2, 0.0, 0.0)

        robot_8 = Robot()
        robot_8.set(0, 1, 0)
        robot_8.set_steering_drift(10.0 / 180.0 * np.pi)    
        x_trajectory_8, y_trajectory_8 = run_pid(robot_8, 0.2, 3.0, 0.0)

        n3 = len(x_trajectory_7)
        _, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 10))
        ax1.plot(x_trajectory_7, y_trajectory_7, 'b', label='P controller')
        ax1.plot(x_trajectory_7, np.zeros(n3), 'r', label='reference')
        ax1.legend()
        ax1.set_title('P Control with steering drift 10 degree and tau_p 0.2',fontsize=12,color='k')

        ax2.plot(x_trajectory_7, y_trajectory_7, 'b', label='P controller')
        ax2.plot(x_trajectory_8, y_trajectory_8, 'g', label='PD controller')
        ax2.plot(x_trajectory_7, np.zeros(n3), 'r', label='reference')
        ax2.legend()
        ax2.set_title('PD Control with steering drift 10 degree and tau_p 0.2 tau_d 3.0',fontsize=12,color='k')


    is_pid_controller_enabled = False
    if is_pid_controller_enabled:
        ### PID Controller
        # Basic Idea：Compensate System Bias
        # Instianate robots without system bias
        robot_p = Robot()
        robot_p.set(0, 1, 0)
        x_trajectory_p, y_trajectory_p = run_pid(robot_p, 0.2, 0.0, 0.0)

        robot_pi = Robot()
        robot_pi.set(0, 1, 0)
        x_trajectory_pi, y_trajectory_pi = run_pid(robot_pi, 0.2, 0.0, 0.004)

        robot_pid = Robot()
        robot_pid.set(0, 1, 0)    
        x_trajectory_pid, y_trajectory_pid = run_pid(robot_pid, 0.2, 3.0, 0.004)


        # Instianate robots with system bias
        robot_2_pi = Robot()
        robot_2_pi.set(0, 1, 0)
        robot_2_pi.set_steering_drift(10.0 / 180.0 * np.pi)
        x_trajectory_2_pi, y_trajectory_2_pi = run_pid(robot_2_pi, 0.2, 0.0, 0.004)

        robot_2_pid = Robot()
        robot_2_pid.set(0, 1, 0)
        robot_2_pid.set_steering_drift(10.0 / 180.0 * np.pi)    
        x_trajectory_2_pid, y_trajectory_2_pid = run_pid(robot_2_pid, 0.2, 3.0, 0.004)


        n4 = len(x_trajectory_p)
        _, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 10))
        ax1.plot(x_trajectory_p, y_trajectory_p, 'b', label='P controller')
        ax1.plot(x_trajectory_5, y_trajectory_5, 'g', label='PD controller')
        ax1.plot(x_trajectory_pi, y_trajectory_pi, 'm', label='PI controller')
        ax1.plot(x_trajectory_pid, y_trajectory_pid, 'k', label='PID controller')
        ax1.plot(x_trajectory_p, np.zeros(n4), 'r', label='reference')
        ax1.legend()
        ax1.set_title('PID without steering drift, tau_p 0.2, tau_d 3, tau_i 0.004',fontsize=12,color='k')

        ax2.plot(x_trajectory_7, y_trajectory_7, 'b', label='P controller')
        ax2.plot(x_trajectory_8, y_trajectory_8, 'g', label='PD controller')
        ax2.plot(x_trajectory_2_pi, y_trajectory_2_pi, 'm', label='PI controller')
        ax2.plot(x_trajectory_2_pid, y_trajectory_2_pid, 'k', label='PID controller')
        ax2.plot(x_trajectory_7, np.zeros(n4), 'r', label='reference')
        ax2.legend()
        ax2.set_title('PID with steering drift 10 degree, tau_p 0.2, tau_d 3, tau_i 0.004',fontsize=12,color='k')


    is_twiddlw_enabled = True
    if is_twiddlw_enabled:
        # Start Twiddle without drift
        params_, err_ = twiddle(drift=False)
        print('Start Twiddle without drift')
        print('Final parameters: tau_p = {}, tau_i = {}, tau_d = {}'.format(params_[0], params_[2], params_[1]))
        print("Final twiddle error = {}".format(err_))
        print('\n')
        robot_1 = make_robot(drift=False)
        x_trajectory_1, y_trajectory_1, _ = run(robot_1, params_)
        n1 = len(x_trajectory_1)
        robot_pid_1 = make_robot(drift=False)
        x_trajectory_pid_1, y_trajectory_pid_1, _ = run(robot_pid_1, [0.2, 3.0, 0.004])


        # Start Twiddle with drift
        params, err = twiddle(drift=True)
        print('Start Twiddle with drift')
        print('Final parameters: tau_p = {}, tau_i = {}, tau_d = {}'.format(params[0], params[2], params[1]))
        print("Final twiddle error = {}".format(err))
        robot_2 = make_robot(drift=True)
        x_trajectory_2, y_trajectory_2, _ = run(robot_2, params)
        n2 = len(x_trajectory_2)
        robot_pid_2 = make_robot(drift=True)
        x_trajectory_pid_2, y_trajectory_pid_2, _ = run(robot_pid_2, [0.2, 3.0, 0.004])


        ### Plot
        _, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 8))

        ax1.plot(x_trajectory_1, y_trajectory_1, 'g', label='Twiddled PID controller')
        ax1.plot(x_trajectory_pid_1, y_trajectory_pid_1, 'b', label='UnTwiddled PID controller')
        ax1.plot(x_trajectory_1, np.zeros(n1), 'r', label='reference')
        ax1.legend()
        ax1.set_title('PID contrast without steering drift',fontsize=12,color='k')

        ax2.plot(x_trajectory_2, y_trajectory_2, 'g', label='Twiddled PID controller')
        ax2.plot(x_trajectory_pid_2, y_trajectory_pid_2, 'b', label='UnTwiddled PID controller')
        ax2.plot(x_trajectory_2, np.zeros(n2), 'r', label='reference')
        ax2.legend()
        ax2.set_title('PID contrast with steering drift',fontsize=12,color='k')


    plt.show()