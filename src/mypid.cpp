#include "mypid.h"

MyPIDController::MyPIDController(float P, float I, float D, float ramp, float integral_limit, float limit)
    : P(P)
    , I(I)
    , D(D)
    , output_ramp(ramp)              // output derivative limit [volts/second]
    , integral_limit(integral_limit) // integral limit
    , limit(limit)                   // output supply limit     [volts]
    , error_prev(0.0f)
    , output_prev(0.0f)
    , integral_prev(0.0f)
{
    timestamp_prev = _micros();
}

// PID controller function
float MyPIDController::operator() (float error){
    // calculate the time from the last call
    unsigned long timestamp_now = _micros();
    float Ts = (timestamp_now - timestamp_prev) * 1e-6f;
    // quick fix for strange cases (micros overflow)
    if(Ts <= 0.001f || Ts > 0.1f) {
        Ts = 0.1f;
        error_prev = error;
        integral_prev = 0.0f;
        output_prev = 0.0f;
    }

    // u(s) = (P + I/s + Ds)e(s)
    // Discrete implementations
    // proportional part
    // u_p  = P *e(k)
    float proportional = P * error;
    // Tustin transform of the integral part
    // u_ik = u_ik_1  + I*Ts/2*(ek + ek_1)
    float integral = integral_prev + I*Ts*0.5f*(error + error_prev);
    // antiwindup - limit the output
    integral = _constrain(integral, -integral_limit, integral_limit);
    // Discrete derivation
    // u_dk = D(ek - ek_1)/Ts
    float derivative = D*(error - error_prev)/Ts;

    // sum all the components
    float output = proportional + integral + derivative;

    // if output ramp defined
    if(output_ramp > 0){
        // limit the acceleration by ramping the output
        float output_rate = (output - output_prev)/Ts;
        if (output_rate > output_ramp)
            output = output_prev + output_ramp*Ts;
        else if (output_rate < -output_ramp)
            output = output_prev - output_ramp*Ts;
    }

    // antiwindup - limit the output variable
    output = _constrain(output, -limit, limit);

    // saving for the next pass
    integral_prev = integral;
    output_prev = output;
    error_prev = error;
    timestamp_prev = timestamp_now;
    return output;
}

void MyPIDController::reset(){
    integral_prev = 0.0f;
    output_prev = 0.0f;
    error_prev = 0.0f;
}
