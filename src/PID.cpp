#include "PID.h"
#include <cmath>
#include <iostream>

using namespace std;

/*
 * TODO: Complete the PID class.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd, bool doTwiddle) {
    p[0]=Kp;
    p[1]=Kd;
    p[2]=Ki;
    // Initialize dp values to a tenth of the initial value of their corresponding coefficient
    dp[0] = 0.1*Kp;
    dp[1] =0.1*Kd;
    dp[2]= 0.1*Ki;
    // Set the tolerance equal to one tenth of the sum of the dps
    tol=0.1*GetSumDP();
    
    p_error = d_error = i_error = 0.0;
    max_p_error = max_d_error = max_i_error = 0.0;
    
    run_twiddle=doTwiddle;
    step=1;
    p_index=0;
    
    step = 1;
    p_index = 0;
    n_steps_transition = 300;
    n_steps_eval = 1600;
// We don't want twiddle to run forever
    max_steps=1000000;
    total_error = 0;
    best_error = std::numeric_limits<double>::max();
// Set the first state for our state machine
    state=FIRSTTIMETHRU;
    std::cout << "Initialized PID Controller" << std::endl;
    std::cout << "Initial parameters" << std::endl;
    std::cout << "P: " << p[0] << ", D: " << p[1] << ", I: " << p[2] << std::endl;
    if (run_twiddle) {
        std::cout << "Evaluation steps: " << n_steps_eval << ", Transition steps: " << n_steps_transition << std::endl;
        std::cout << "dp[0]: " << dp[0] << ", dp[1]: " << dp[1] << ", dp[2]: " << dp[2] << std::endl;
        std::cout << "tolerance: " << tol << std::endl;
    }
}

void PID::UpdateError(double cte) {
    if (step == 1)
        d_error = 0;
    d_error = cte - p_error;
    p_error = cte;
    i_error += cte;
    
    if (fabs(p_error) > max_p_error)
        max_p_error=fabs(p_error);
    if (fabs(d_error) > max_d_error)
        max_d_error=fabs(d_error);
    if (fabs(i_error) > max_i_error)
        max_i_error=fabs(i_error);
    
    // Let error settle with new weights before we start the evaluation
    if (step % (n_steps_eval + n_steps_transition) > n_steps_transition)
        total_error += cte*cte;
    
    // print out the number of steps each 100 in order to count how many steps are needed to
    // once around the track
    if ((step % 100)==0)
        std::cout << "Steps=: " << step << std::endl;
    
    // Run twiddle algorithm
    if (run_twiddle && (step % (n_steps_transition + n_steps_eval) == 0)) {
        if (step <= max_steps && GetSumDP() > tol) {
            std::cout << "step: " << step << std::endl;
            std::cout << "total error: " << total_error << std::endl;
            std::cout << "best error: " << best_error << std::endl;
            switch (state) {
                case FIRSTTIMETHRU:
                    state=FIRSTADD;
                    break;
                case FIRSTADD:
                    if (best_error==std::numeric_limits<double>::max()) {
                        std::cout << "Setting best error=" << TotalError() << std::endl;
                        best_error=TotalError();
                    }
                    std::cout << "FIRST ADD, p_index=" << p_index << std::endl;
                    p[p_index]+=dp[p_index];
                    state=EVALAFTERADD;
                    break;
                case EVALAFTERADD:
                    std::cout << "EVALUATE AFTER ADD, p_index=" << p_index << std::endl;
                    if (TotalError() < best_error) {
                        std::cout << "Error decreased: total_error=" << TotalError() << " best_error=" << best_error << std::endl;
                        best_error=total_error;
                        dp[p_index] *= 1.1;
                        // move on to next index
                        p_index = (p_index + 1) % 3;
                        p[p_index]+=dp[p_index];
                        state=EVALAFTERADD;
                    }
                    else {
                        std::cout << "Error increased: total_error=" << TotalError() << " best_error=" << best_error << std::endl;
                        p[p_index] -= 2 * dp[p_index];
                        state=EVALAFTERSUBTRACT;
                        std::cout << "New p[" << p_index << "]=" << p[p_index] << std::endl;
                    }
                    break;
                case EVALAFTERSUBTRACT:
                    std::cout << "EVALUATE AFTER SUBTRACTION, p_index=" << p_index << endl;
                    if (TotalError() < best_error) {
                        std::cout << "Error decreased: total_error=" << TotalError() << " best_error=" << best_error << std::endl;
                        best_error = total_error;
                        dp[p_index] *= 1.1;
                    }
                    else {
                        std::cout << "Error increased: total_error=" << TotalError() << " best_error=" << best_error << std::endl;
                        p[p_index] += dp[p_index];
                        dp[p_index] *= 0.9;
                        std::cout << "New p[" << p_index << "]=" << p[p_index] << std::endl;
                    }
                    // move on to next index
                    p_index = (p_index + 1) % 3;
                    p[p_index]+=dp[p_index];
                    state=EVALAFTERADD;
                    break;
                default:
                    std::cout << "Should not get here" << std::endl;
                    break;
            }
            ResetTotalError();
            std::cout << "updated parameters" << std::endl;
            std::cout << "P: " << p[0] << ", D: " << p[1] << ", I: " << p[2] << std::endl;
            std::cout << "dp[0]: " << dp[0] << ", dp[1]: " << dp[1] << ", dp[2]: " << dp[2] << std::endl;
            std::cout << "ErrorP: " << max_p_error << ", ErrorI: " << max_i_error << ", ErrorD: " << max_d_error << std::endl;
        }
        else {
            std::cout << "Twiddle has completed. Steps= " << step << ", Sum of DPs=" << GetSumDP() << std::endl;
            std::cout << "P: " << p[0] << ", D: " << p[1] << ", I: " << p[2] << std::endl;
        }
    }
    step++;
}

double PID::TotalError() {
    return total_error; 
}

void PID::ResetTotalError() {
    total_error=0.0;
}

double PID::GetControlValue() {
    return -p[0] * p_error - p[1] * d_error - p[2] * i_error;
}

double PID::GetSumDP() {
    double sum=0.0;
    int i;
    for (i=0;i<3;i++)
        sum+=dp[i];
    return sum;
}
