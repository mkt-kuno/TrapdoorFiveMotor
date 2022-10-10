#ifndef AsyncTimer_h
#define AsyncTimer_h

#include <Arduino.h>

class AsyncTimer {
    private:
        unsigned long prev_micros = 0;
        unsigned long next_usec = 0;
    public:
        AsyncTimer() {};
        void init(void) {
            prev_micros = micros();
        }
        void next_micorsec(unsigned long usec) {
            next_usec = usec;
        }
        void next_millisec(unsigned long msec) {
            next_usec = msec * 1000;
        }
        void next_sec(unsigned long sec) {
            next_usec = sec * 1000 * 1000;
        }
        bool trigger(void) {
            bool ret = ((micros() - prev_micros) > next_usec)? true: false;
            return ret;
        };
        bool trigger_and_next(void) {
            bool ret = ((micros() - prev_micros) > next_usec)? true: false;
            if (ret) prev_micros = micros();
            return ret;
        };
};

#endif /*AsyncTimer_h*/