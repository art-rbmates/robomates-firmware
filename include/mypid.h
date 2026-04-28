#ifndef MY_PID_H
#define MY_PID_H


#include <SimpleFOC.h>

/**
 *  PID controller class
 */
class MyPIDController
{
public:
    /**
     *  
     * @param P - Proportional gain 
     * @param I - Integral gain
     * @param D - Derivative gain 
     * @param ramp - Maximum speed of change of the output value
     * @param integral_limit - Maximum output value
     * @param limit - Maximum output value
     */
    MyPIDController(float P, float I, float D, float ramp, float integral_limit, float limit);
    ~MyPIDController() = default;

    float operator() (float error);
    void reset();

    float P; //!< Proportional gain 
    float I; //!< Integral gain 
    float D; //!< Derivative gain 
    float output_ramp; //!< Maximum speed of change of the output value
    float integral_limit;
    float limit; //!< Maximum output value

protected:
    float error_prev; //!< last tracking error value
    float output_prev;  //!< last pid output value
    float integral_prev; //!< last integral component value
    unsigned long timestamp_prev; //!< Last execution timestamp
};

#endif // MY_PID_H