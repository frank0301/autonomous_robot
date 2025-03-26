#ifndef PROTOTYPE_DRIVER_HPP
#define PROTOTYPE_DRIVER_HPP

#include <cmath>

class SerialCom {
public:
    int pwm_L;
    bool dir_L; // if true - going reverse
    int pwm_R;
    bool dir_R;
    bool gear; // if true - high gear
};

class PrototypeControl {
public:
    SerialCom compute_control(double throttle, double steering, bool is_low_gear);

    int SpeedCtrl(double omega, int config);

private:
    // const double wheel_r = 0.175; // Wheel's radius [m]
    // const double d = 0.485;       // Wheels' distance [m]
    const double pi = 3.14159265;
    const int thres = 0.05;          // Threshold for mode switch (%)
    const double thr_sens = 0.9;      // 
    const double tur_sens = 0.9;
};


#endif // PROTOTYPE_DRIVER_HPP
