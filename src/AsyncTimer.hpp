#ifndef AsyncTimer_h
#define AsyncTimer_h

#include <Arduino.h>

class AsyncTimer {
    private:
        unsigned long prev_micros = 0;
        unsigned int next_usec = 0;
    public:
        void init() {
            prev_micros = micros();
        }
        void next_usec(unsigned long usec) {
            next_usec = usec;
        }
        void next_msec(unsigned long msec) {
            next_usec = msec * 1000;
        }
        void next_sec(unsigned long sec) {
            next_usec = sec * 1000 * 1000;
        }
        bool trigger() {
            return ((micros() - prev_micros) > next_usec)? true: false;
        }
}

#endif /*AsyncTimer_h*/