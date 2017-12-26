#ifndef PID_H
#define PID_H
#include <vector>

enum TwiddleState {
    FIRSTTIMETHRU, FIRSTADD, EVALAFTERADD, EVALAFTERSUBTRACT
};

class PID {
public:
    /*
     * Errors
     */
    double p_error, max_p_error;
    double i_error, max_i_error;
    double d_error, max_d_error;
    /*
     * Coefficients
     */
    double p[3], dp[3];
    /*
     * Twiddle variables
     */
    int step, param_index, p_index;
    // number of steps to allow changes to coefficients to transition, then to evaluate error
    int n_steps_transition, n_steps_eval, max_steps;
    /// The total error for n_steps_eval, the best error, and the tolerance
    double total_error, best_error, tol;
    bool run_twiddle;
    
    TwiddleState state;
    /*
     * Constructor
     */
    PID();
    
    /*
     * Destructor.
     */
    virtual ~PID();
    
    /*
     * Initialize PID.
     */
    void Init(double Kp, double Ki, double Kd,bool doTwiddle);
    
    /*
     * Update the PID error variables given cross track error.
     */
    void UpdateError(double cte);
    void ResetTotalError();
    /*
     * Calculate the total PID error.
     */
    double TotalError();
    double GetControlValue();
    double GetSumDP();
};

#endif /* PID_H */

