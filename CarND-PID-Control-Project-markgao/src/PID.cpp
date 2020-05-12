/**************************************************************************************************
 * @ File: serialThread.cpp
 * @ Author: MarkGao
 * @ Date: 2020-04-26
 * @ Email: 819699632@qq.com
 * @ Version: 1.0
 * @ Description: System MAIN function
**************************************************************************************************/
#include "PID.h"

// TODO: Complete the PID class. You may add any additional desired functions.

PID::PID()
{
    p_error = 0.0;
    i_error = 0.0;
    d_error = 0.0;
}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_)
{
   // TODO: Initialize PID coefficients (and errors, if needed)
    Kp = Kp_;
    Ki = Ki_;
    Kd = Kd_;
}



/**************************************************************************************************
 * @ Function: UpdateError()
 * @ Description: Update PID errors based on cte.
 * @ Calls:
 * @ Input: void
 * @ Output: void
 * @ Return: void
 * @ Others:
**************************************************************************************************/
void PID::UpdateError(double cte)
{
    static double cte_prev = cte;
    p_error = cte;
    i_error += cte;
    d_error = cte - cte_prev;

    cte_prev = cte;
}



/**************************************************************************************************
 * @ Function: TotalError()
 * @ Description: Calculate and return the total error
 * @ Calls:
 * @ Input: void
 * @ Output: void
 * @ Return: void
 * @ Others:
**************************************************************************************************/
double PID::TotalError() {

    double P_term = - Kp * p_error;
    double I_term = - Ki * i_error;
    double D_term = - Kd * d_error;

    double steer_value = P_term + I_term + D_term;

    return steer_value;  // TODO: Add your total error calc here!
}
