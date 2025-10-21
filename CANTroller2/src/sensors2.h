#pragma once
// below are beginnings of making hotrc and motors also conform to this class structure.
// it was removed from the bottom of the other existing device classes in sensors.h
// 
// this is all unfinished and unused
//
// RCChannel is a class for the channels of the hotrc
// it contains the association with the RMT channels and probably filtering
// I imagine it might make sense to have two child classes, one for the two toggle channels, and one for the two analog channels
// Anyway eventually we'd like to move channel input related functionality from hotrc class into these classes, 
// then hotrc contains four instances, one per channel, which it can manage
class RCChannel : public Sensor {  // class for each channel of the hotrc
  protected:
  public:
    RCChannel(int arg_pin) : Sensor(arg_pin) {
        _long_name = "RC channel";
        _short_name = "rcchan";
        _native_units = "us";
        _si_units = "%";
    }
    RCChannel() = delete;
    virtual void setup() {
        Sensor::setup();
        set_pin(_pin, INPUT);
        set_can_source(src::Pin, true);
        set_source(src::Pin);
    }
};
class RCToggle : public RCChannel {};
class RCAnalog : public RCChannel {};

// note: I implemented the gas servo, but it looks like it's all in native units. should it still be a transducer?
// ServoMotor is a base class for our type of actuators, where by varying a pulse width (in us), motors move.
//   The gas motor is an actual rc servo, the brake and steering are 12v dc motors with voltage controlled based on servo-like pwm signals (feature of the jaguars)
class ServoMotor2 : public Transducer {
  protected:
    Servo motor;
    Timer updatetimer{85000}, outchangetimer;
    float lastoutput, max_out_change_rate_pcps = 800.0;
    int _pin, _freq;
    virtual float write_sensor() {  // note: should be marked 'override' but compiler says it doesn't override anything...?
        ezread.squintf(ezread.madcolor, "Err: %s does not have an overridden write_sensor() function\n", _short_name.c_str());
        return NAN;
    }
    void changerate_limiter() {
        float max_out_change_pc = max_out_change_rate_pcps * outchangetimer.elapsed() / 1000000.0;
        outchangetimer.reset();
        set_si(constrain(val(), lastoutput - max_out_change_pc, lastoutput + max_out_change_pc));
        // lastoutput = pc[Out];  // note: you must set lastoutput = pc[Out]
    }
  public:
    // ServoMotor(int pin, int freq) : Transducer<float, float>(pin) {
    ServoMotor2(int pin, int freq) : Transducer(pin), _freq(freq) {
        set_pin(_pin, OUTPUT);   // needed?
        _transtype = ActuatorType;
        _long_name = "Unknown PWM motor";
        _short_name = "pwmout";
        _native_units = "us";
    }
    ServoMotor2() = delete;
    void setup() {  // from child classes first set up limits, then run this
        motor.setPeriodHertz(_freq);
        motor.attach(_pin, _native.min(), _native.max());  // servo goes from 500us (+90deg CW) to 2500us (-90deg CCW)
    }
    void write() {
        if (!std::isnan(_native.val())) motor.writeMicroseconds((int)(_native.val()));
        lastoutput = _native.val();    
        // _val_raw = _native.val();
        // _servo.writeMicroseconds((int)_val_raw);  // Write result to servo interface
    }
    float max_changerate() { return max_out_change_rate_pcps; };
    void set_max_changerate(float a_newrate) { max_out_change_rate_pcps = a_newrate; };
};
class Jaguar : public ServoMotor2 {
  public:
    Jaguar(int pin, int freq) : ServoMotor2(pin, freq) {
        _convmethod = OpLimMap;
        _long_name = "Unknown Jaguar controller";
        _short_name = "unkjag";
        _si_units = "V";
    }
    Jaguar() = delete;
    void setup() {
        set_abslim(45.0, 168.2);  //this should also set oplim to the same
        float m = (absmax() - absmin()) / (absmax_native() / absmin_native());  // (180 - 0) / (2500 - 500) = 0.09 deg/us
        set_abslim(0.0, 180.0, false);
    }
};
class ThrottleServo2 : public ServoMotor2 {
  protected:
    Param governor_pc, idle_si, idletemp_f;
    float max_throttle_angular_velocity_pcps;  // Software governor will only allow this percent of full-open throttle (percent 0-100)
    
    // float idle_si[NumMotorVals] = { 45.0, NAN, 60.0, 58.0, NAN, 43.0, 75.0, 1.0 };          // in angular degrees [OpMin(hot)/-/OpMax(cold)/Out/-/AbsMin/AbsMax/Margin]
    // float idletemp_f[NumMotorVals] = { 60.0, NAN, 205.0, 75.0, NAN, 40.0, 225.0, 1.5};      // in degrees F [OpMin/-/OpMax/Out/-/AbsMin/AbsMax/Margin]
    float idle_pc = 11.3;                              // idle percent is derived from the si (degrees) value
    float starting_pc = 25.0;                          // percent throttle to open to while starting the car
  public:
    ThrottleServo2(int pin, int freq) : ServoMotor2(pin, freq) {
        _dir = TransDir::Fwd;  // if your servo goes CCW with increasing pulsewidths, change to Rev
        _long_name = "Throttle servo";
        _short_name = "throtl";
        _si_units = "deg";
    }
    ThrottleServo2() = delete;
    void setup() {
        _convmethod = AbsLimMap;
        set_abslim(0.0, 180.0, false);
        set_abslim_native(500.0, 2500.0, false);
        float m = (absmax() - absmin()) / (absmax_native() / absmin_native());  // (180 - 0) / (2500 - 500) = 0.09 deg/us
        set_conversions(m, 0.0);
        governor_pc.set(95);
        idle_si.set_limits(43.0, 75.0);
        idle_si.set(58.0);
        ServoMotor2::setup();

    }
    // set_oplim_native(1000.0, 2000.0, false);       
    // jaguar range in degrees: (45.0, 168.2);
};
class BrakeMotor2 : public Jaguar {
  public:
    BrakeMotor2(int pin, int freq) : Jaguar(pin, freq) {
        _long_name = "Brake motor";
        _short_name = "brake";
    }
    BrakeMotor2() = delete;
};
class SteerMotor2 : public Jaguar {
  public:
    SteerMotor2(int pin, int freq) : Jaguar(pin, freq) {
        _long_name = "Steering motor";
        _short_name = "steer";
    }
    SteerMotor2() = delete;
};
trying to begin to make temp sensors conform to this class structure so i can simulate them. may be throwaway?
class TempSensor : public Sensor {
  protected:
    TemperatureSensorManager* _tempsens;
  public:
    void setup() {
        set_can_source(src::Pot, true);
    }
};
