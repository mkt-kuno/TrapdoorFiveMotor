#ifndef AsyncStepper_h
#define AsyncStepper_h

#include <Arduino.h>
#include <math.h>

// Memo 
// Linear Actuator: 2400 steps / mm
//                    12   rot / mm
// Motor:            200 steps / rot
// MotorDriver TB6600

class AsyncStepper {
  enum STEPPER_STATE {
    STEPPER_STOP = 0,
    STEPPER_READY,
    STEPPER_DRIVE_CHECK_SWITCH,
    STEPPER_DRIVE_HIGH,
    STEPPER_DRIVE_KEEP_HIGH,
    STEPPER_DRIVE_LOW,
    STEPPER_DRIVE_KEEP_LOW,
  };
  private:
    const unsigned int step_per_mm = 2400;
    long position_step = 0;
    long target_step = 0;
    unsigned int pulse_high_usec = 0;
    unsigned int pulse_low_usec = 0;
    STEPPER_STATE state = STEPPER_STOP;
    unsigned long prev_micros = 0;
    int ena_pin, dir_pin, pul_pin, max_pin, min_pin;

    // About duty cycle
    // 20kHz(50us) when duty cyle is 25 high / 75 low
    // 13kHz(77us) when duty cycle is 50 high / 50 high
    void set_pulse_width(double mm_per_min) {
        if (mm_per_min < 0.0) mm_per_min = mm_per_min * (-1.0);
        if (mm_per_min > 30.0) mm_per_min = 30.0;

        double step_microsec = floor((double)60.0E6 / ((double)mm_per_min * step_per_mm));
        pulse_high_usec = pulse_low_usec = floor(step_microsec / 2);
        if (step_microsec < 50.0) {
            pulse_high_usec = floor(step_microsec / 4);
            pulse_low_usec = floor(step_microsec * 3 / 4);
        }
    }

  public:
    AsyncStepper(const int pulse, const int dir, const int ena, const int max, const int min) {
      pul_pin = pulse;
      dir_pin = dir;
      ena_pin = ena;
      max_pin = max;
      min_pin = min;
      pinMode(pul_pin, OUTPUT);
      pinMode(dir_pin, OUTPUT);
      pinMode(ena_pin, OUTPUT);
      pinMode(max_pin, INPUT_PULLUP);
      pinMode(min_pin, INPUT_PULLUP);
    }

    void start(double move_mm, double mm_per_min, bool incremental = false) {
        if (state != STEPPER_STOP) return;
        if (move_mm == 0 || mm_per_min == 0) return;
        long steps = floor(step_per_mm * move_mm);
        if (incremental) target_step = position_step + steps;
        else target_step = steps;
        set_pulse_width(mm_per_min);
        state = STEPPER_READY;
        return;
    }

    float get_mm(void) { return (float)((double)position_step / (double)step_per_mm);}
    void set_mm(double current_mm) {position_step = (long)((double)step_per_mm * current_mm);}
    bool get_max_swich(void) { return digitalRead(max_pin); }
    bool get_min_swich(void) { return digitalRead(min_pin); }

    bool is_stop() { return (state==STEPPER_STOP)?true:false; }

    void loop() {
      switch (state) {
        case STEPPER_STOP:
          break;
        case STEPPER_READY:
          digitalWrite(dir_pin, (target_step < position_step)?HIGH:LOW);
          digitalWrite(ena_pin, HIGH);
          //即ステート推移可能にする
          state = STEPPER_DRIVE_CHECK_SWITCH;
          break;
        case STEPPER_DRIVE_CHECK_SWITCH:
          state = STEPPER_DRIVE_HIGH;
          if (((target_step > position_step) && !get_max_swich())
           || ((target_step < position_step) && !get_min_swich())) {
             state = STEPPER_STOP;
           }
          break;
        case STEPPER_DRIVE_HIGH:
          digitalWrite(pul_pin, LOW);
          //直前時間の更新
          prev_micros = micros();
          state = STEPPER_DRIVE_KEEP_HIGH;
          break;
        case STEPPER_DRIVE_KEEP_HIGH:
          if ((micros() - prev_micros) > pulse_high_usec) state = STEPPER_DRIVE_LOW;
          break;
        case STEPPER_DRIVE_LOW:
          digitalWrite(pul_pin, HIGH);
          if (target_step > position_step) position_step++;
          if (target_step < position_step) position_step--;
          prev_micros = micros();
          state = STEPPER_DRIVE_KEEP_LOW;
          if (position_step == target_step) state = STEPPER_STOP;
          break;
        case STEPPER_DRIVE_KEEP_LOW:
          if ((micros() - prev_micros) > pulse_low_usec) state = STEPPER_DRIVE_CHECK_SWITCH;
          break;
      }
    }
};

#endif /*AsyncStepper_h*/