from robot import Robot
import numpy as np



def make_robot(drift):
    """
    Resets the robot back to the initial position and drift.
    You'll want to call this after you call `run`.
    """
    robot = Robot()
    robot.set(0, 2, 5 / 180 * np.pi)
    if drift:
        robot.set_steering_drift(10 / 180 * np.pi)

    return robot



# NOTE: We use params instead of tau_p, tau_d, tau_i
def run(robot, params, n=200, speed=1.0):
    x_trajectory = []
    y_trajectory = []
    err = 0
    prev_cte = robot.y
    int_cte = 0
    # for i in range(2*n):
    #     cte = robot.y
    #     diff_cte = cte - prev_cte
    #     int_cte += cte
    #     prev_cte = cte
    #     steer = -params[0] * cte - params[1] * diff_cte - params[2] * int_cte
    #     robot.move(steer, speed)
    #     x_trajectory.append(robot.x)
    #     y_trajectory.append(robot.y)
    #     if i >= n:
    #         err += cte ** 2
        # err += cte
    for _ in range(n):
        cte = robot.y
        diff_cte = cte - prev_cte
        int_cte += cte
        prev_cte = cte
        steer = -params[0] * cte - params[1] * diff_cte - params[2] * int_cte
        robot.move(steer, speed)
        x_trajectory.append(robot.x)
        y_trajectory.append(robot.y)
        err += cte ** 2

    return x_trajectory, y_trajectory, err / n



# Make this tolerance bigger if you are timing out!
def twiddle(drift, tol=0.2): 
    # Don't forget to call `make_robot` before every call of `run`!
    p = [0, 0, 0]
    dp = [1, 1, 1]    # [1, 1, 1]
    robot = make_robot(drift)
    _, _, best_err = run(robot, p)
    
    it = 0
    while sum(dp) > tol:
        # time.sleep(2)
        print('Iteration {}'.format(it))
        print('best error = {}'.format(best_err))
        print('Parameters: tau_p = {}, tau_i = {}, tau_d = {}'.format(p[0], p[2], p[1]))
        print('dp: [{}, {}, {}]'.format(dp[0], dp[1], dp[2]))
        print()
        for i in range(len(p)):
            p[i] += dp[i]
            robot = make_robot(drift)
            _, _, err = run(robot, p)

            if err < best_err:
                best_err = err
                dp[i] *= 1.1
            else:
                p[i] -= 2 * dp[i]
                robot = make_robot(drift)
                _, _, err = run(robot, p)

                if err < best_err:
                    best_err = err
                    dp[i] *= 1.1
                else:
                    p[i] += dp[i]
                    dp[i] *= 0.9        
        it += 1

    return p, best_err
