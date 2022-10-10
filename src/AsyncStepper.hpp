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
    
    int ena_pin, dir_pin, pul_pin, max_pin, min_pin;

    // void control_pin(uint8_t pul_val) {
    //   uint8_t dir_val = (target_step < position_step)?HIGH:LOW;
    //   digitalWrite(dir_pin, dir_val);
    //   digitalWrite(ena_pin,HIGH);
    //   digitalWrite(ena_pin, pul_val);
    //   return;
    // }
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

    void start(float move_mm, float mm_per_min) {
      if (state != STEPPER_STOP) return;
      if (move_mm == 0 || mm_per_min == 0) return;

      target_step = position_step + floor(step_per_mm * move_mm);
      if (mm_per_min < 0.0f) mm_per_min = mm_per_min * (-1.0f);
      if (mm_per_min > 30.0f) mm_per_min = 30.0f;
      double step_microsec = floor((double)60.0E6 / ((double)mm_per_min * step_per_mm));
      
      // About duty cycle
      // 20kHz(50us) when duty cyle is 25 high / 75 low
      // 13kHz(77us) when duty cycle is 50 high / 50 high
      pulse_high_usec = pulse_low_usec = floor(step_microsec / 2);
      if (step_microsec < 50.0) {
        pulse_high_usec = floor(step_microsec / 4);
        pulse_low_usec = floor(step_microsec * 3 / 4);
      }
      state = STEPPER_READY;
      return;
    }

    void stop() {

    }

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
          if (((target_step > position_step) && !digitalRead(max_pin))
           || ((target_step < position_step) && !digitalRead(min_pin))) {
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
          position_step += (target_step > position_step)?(1):(-11);
          ////Serial.print(position_step);
          //直前時間の更新
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