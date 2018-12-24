#include "PID.h"
#include <iostream>
#include <limits>
#include <math.h>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    this->Kp = Kp;  // proportional term: how hard you want to steer back to the center of the road
    this->Ki = Ki;  // integral term: accumulated error due to wheel misalignment or others
    this->Kd = Kd;  // derivative term: smooth the turn back to the center of the road


    p_error = 0;
    i_error = 0;
    d_error = 0;
    cte_prev = 0.0;

    b_twiddle_init = false;
    b_twiddle_reset = false;
    twiddle_tolerance = 0.01;

    dp = {0.1 * Kp, 0.1 * Kd, 0.1 * Ki};
    pid_idx = 2;
    i_eval_steps = 1800;
    total_error = 0;
    best_error = std::numeric_limits<double>::max();


}

void PID::UpdateError(double cte) {

    p_error = cte;
    i_error += cte;
    d_error = cte - cte_prev;
    cte_prev = cte;
   
}

double PID::TotalError() {
    
    double output = -1 * (Kp * p_error + Kd * d_error + Ki * i_error); 
    double max_output = 1.0;
    double min_output = -1.0;
    if (output > max_output){
        output = max_output;
    }
    else if (output < min_output){
        output = min_output;
    }
    return output;
    
}

void PID::Twiddle(double total_error){
    if(!b_twiddle_init){
        cout<<"twiddle initialization";
        best_error = total_error;
        b_twiddle_init = true;
        return;
    }

    double sum_dp = dp[0] + dp[1] + dp[2];
    if (sum_dp > twiddle_tolerance){

        if (b_twiddle_reset){
            cout<<"twiddle reset!..."<<"\n";
            PID_hyper_prev = return_Hyperparameter(pid_idx);
            PID_Hyperparameter(pid_idx, dp[pid_idx]);
            cout<<"hyperparamter magnitude is increased!"<<"\n";
            b_twiddle_reset = false;
        }
        else{
            if (total_error < best_error){
                    cout<<"PID twiddle improvement!"<<"\n";
                    best_error =  total_error;
                    dp[pid_idx] *= 1.1;
                    b_twiddle_reset = true;
                    pid_idx = (pid_idx + 1) % 3;
            }
            else{
                if (fabs(PID_hyper_prev) < fabs(return_Hyperparameter(pid_idx))){
                    PID_hyper_prev = return_Hyperparameter(pid_idx);
                    PID_Hyperparameter(pid_idx, -2 * dp[pid_idx]); 
                    cout<<"hyperparamter magnitude is decreased!"<<"\n";
                }
                else{
                    PID_hyper_prev = return_Hyperparameter(pid_idx);
                    PID_Hyperparameter(pid_idx, dp[pid_idx]);
                    dp[pid_idx] *= 0.9;
                    cout<<"hyperparamter magnitude remains same!"<<"\n";
                    b_twiddle_reset = true;
                    pid_idx = (pid_idx + 1) % 3;
                }
                
            }
        }

        
        cout<<"new PID hyperparameters "<<"\n";
        cout<<"P: "<<Kp<<" , I: "<<Ki<<" , D: "<<Kd<<"\n";
    }
}

void PID::Twiddle_Restart(uWS::WebSocket<uWS::SERVER> ws) {

  std::string reset_msg = "42[\"reset\",{}]";
  ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);

}

void PID::PID_Hyperparameter(int index, double increment){
    switch (index)
    {
        case 0: Kp += increment; break;
        case 1: Kd += increment; break;
        case 2: Ki += increment; break;
        default: cout<<"PID Hyperparameter index is out of bounds"; break;
    }

}

double PID::return_Hyperparameter(int index){
    double val;
    switch (index)
    {
        case 0: val = Kp; break;
        case 1: val = Kd; break;
        case 2: val = Ki; break;
    }    
    return val;
}

