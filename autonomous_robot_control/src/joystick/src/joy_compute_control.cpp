#include "joystick/joy_compute_control.hpp"

// Class only - Take commands from the joystick and convert them to PWM signals
SerialCom PrototypeControl::compute_control(double throttle, double steering, bool is_low_gear) 
{
    SerialCom ser_com;  // Serial communication structure for sending PWM and direction

    int pwm_L;              // PWM output for the left wheel
    bool is_reverse_dir_L;  // Left wheel direction: false (forward), true (reverse)
    int pwm_R;              // PWM output for the right wheel
    bool is_reverse_dir_R;  // Right wheel direction: false (forward), true (reverse)
    double omega_L;         // Angular velocity for the left wheel
    double omega_R;         // Angular velocity for the right wheel
        
    // Input (ratio) ranges from -1 to 1
    if (abs(throttle) < thres){  // Ignore small throttle values (dead zone)
        throttle = 0;
    }

    if (abs(steering) < thres){  // Ignore small steering values (dead zone)
        steering = 0;
    }

    // Apply sensitivity correction for reverse movement
    if(throttle < 0){     
        throttle = throttle * (1 / thr_sens);
    }

    // Case 1: No throttle (rotation in place)
    if (throttle == 0){     
        steering = steering * 1.6;  // Boost steering when stationary

        if(steering < 0){  // Turn right
            omega_L = 6.16 * steering;
            omega_R = -6.16 * steering;
        }
        else{              // Turn left
            omega_L = -6.16 * steering;
            omega_R = 6.16 * steering;
        }
    }

    // Case 2: Moving forward
    else if (throttle > 0){    
        if (is_low_gear == true){   // Low gear

            if(steering == 0){      // Move straight
                omega_L = 15.56 * throttle;
                omega_R = 15.56 * throttle;
            }
            else if (steering < 0){  // Steer right
                omega_L = 15.56 * throttle * (1 + steering);
                omega_R = 15.56 * throttle;
            }
            else if (steering > 0) { // Steer left
                omega_L = 15.56 * throttle;
                omega_R = 15.56 * throttle * (1 - steering);
            }

        } else {    // High gear

            if(steering == 0){  // Move straight
                omega_L = 31.35 * throttle;
                omega_R = 31.35 * throttle;
            }
            if (steering < 0){   // Steer right
                omega_L = 31.35 * throttle * (1 + steering);
                omega_R = 31.35 * throttle;
            }
            else if (steering > 0) {  // Steer left
                omega_L = 31.35 * throttle;
                omega_R = 31.35 * throttle * (1 - steering);
            }

        }
    }

    // Case 3: Moving backward
    else if (throttle < 0){    

        if(steering == 0){  // Move straight backward
            omega_L = 6.16 * throttle;
            omega_R = 6.16 * throttle;
        }
        else if(steering < 0){  // Turn left while moving backward
            omega_L = 6.16 * throttle * (1 + steering);
            omega_R = 6.16 * throttle;
        }
        else {  // Turn right while moving backward
            omega_L = 6.16 * throttle;
            omega_R = 6.16 * throttle * (1 - steering);
        }  

    }

    // Convert angular velocities to PWM signals
    if (is_low_gear == true){  // Low gear

        if (omega_L == 0){  // No movement
            pwm_L = 0;
        }
        else if (omega_L < 0){  // Reverse
            pwm_L = SpeedCtrl(abs(omega_L), 2);  // Backward low gear
        }
        else if (omega_L > 0){  // Forward
            pwm_L = SpeedCtrl(omega_L, 0);  // Forward low gear
        }

        if (omega_R == 0){  // No movement
            pwm_R = 0;
        }
        else if (omega_R < 0){  // Reverse
            pwm_R = SpeedCtrl(abs(omega_R), 2);  // Backward low gear
        }
        else if (omega_R > 0){  // Forward
            pwm_R = SpeedCtrl(omega_R, 0);  // Forward low gear
        }

    }
    else if (is_low_gear == false){  // High gear

        if(omega_L == 0){  // No movement
            pwm_L = 0;
        }
        else if (omega_L < 0){  // Reverse
            pwm_L = SpeedCtrl(abs(omega_L), 3);  // Backward high gear
        }
        else if (omega_L > 0){  // Forward
            pwm_L = SpeedCtrl(omega_L, 1);  // Forward high gear
        }

        if(omega_R == 0){  // No movement
            pwm_R = 0;
        }
        else if (omega_R < 0){  // Reverse
            pwm_R = SpeedCtrl(abs(omega_R), 3);  // Backward high gear
        }
        else if (omega_R > 0){  // Forward
            pwm_R = SpeedCtrl(omega_R, 1);  // Forward high gear
        }

    }

    // Determine wheel directions
    is_reverse_dir_L = (omega_L < 0);  // True if left wheel is moving backward
    is_reverse_dir_R = (omega_R < 0);  // True if right wheel is moving backward

    // Set PWM and direction values in serial communication structure
    ser_com.pwm_L = pwm_L * 100 / 255;  // Normalize PWM value for left wheel (0-100%)
    ser_com.pwm_R = pwm_R * 100 / 255;  // Normalize PWM value for right wheel (0-100%)
    ser_com.is_reverse_dir_L = is_reverse_dir_L;  // Left wheel direction
    ser_com.is_reverse_dir_R = is_reverse_dir_R;  // Right wheel direction
    ser_com.gear = is_low_gear;  // Current gear state

    return ser_com;
}

// Function to calculate PWM based on angular velocity (omega) and configuration (gear and direction)
int PrototypeControl::SpeedCtrl(double omega, int config) {
    int pwm_outp;  // PWM output
    if (omega == 0)  // No movement, return 0
        pwm_outp = 0;
    else {
        switch (config) {
        case 0:  // Forward low gear
            if (omega <= 15.56){
                pwm_outp = int(7.2374 * omega + 64.5066);  // Linear formula for low-speed PWM
                break;
            }
            else{
                pwm_outp = 178;  // Cap PWM to max value
                break;
            }
        case 1:  // Forward high gear
            if (omega <= 31.35){
                pwm_outp = int(3.7325 * omega + 61.2301);  // Linear formula for high-speed PWM
                break;
            }
            else{
                pwm_outp = 178;  // Cap PWM to max value
                break;
            }
        case 2:  // Backward low gear
            if (omega <= 6.4){
                pwm_outp = int((6.921 * omega + 65.9686) * 1.4);  // Adjusted formula for reverse
                break;
            }
            else{
                pwm_outp = int((6.921 * 6.4 + 65.9686) * 1.4);  // Cap PWM for reverse low
                break;
            }
        case 3:  // Backward high gear
            if (omega <= 6.4){
                pwm_outp = int((3.6837 * omega + 61.8913) * 1.4);  // Adjusted formula for reverse
                break;
            }
            else{
                pwm_outp = int((3.6837 * 6.4 + 61.8913) * 1.4);  // Cap PWM for reverse high
                break;
            }
        }
    }
    return pwm_outp;
}
