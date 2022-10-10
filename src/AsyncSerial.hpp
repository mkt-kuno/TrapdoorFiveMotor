#ifndef AsyncSerial_h
#define AsyncSerial_h

#include <Arduino.h>
#include <Print.h>
#include <RingBuf.h>

#define TX_BUFFER_SIZE 1024
#define RX_BUFFER_SIZE 256

class AsyncSerial : public Print {
    private:
        RingBuf<unsigned char, TX_BUFFER_SIZE> tx_buf;
        HardwareSerial *serial = NULL;

    public:
        AsyncSerial(int num) {
            switch(num) {
                case 0:serial = &Serial;break;
                case 1:serial = &Serial1;break;
                case 2:serial = &Serial2;break;
                case 3:serial = &Serial3;break;
                default: serial = &Serial;break;
            }
        }

        // override function
        size_t write(uint8_t data) {
            if (tx_buf.isFull()) return 0;
            tx_buf.push(data);
            return 1;
        }

        void begin(long baud) {
            serial->begin(baud);
        }
        bool available() {
            return serial->available();
        }
        int read() {
            return serial->read();
        }

        void loop(void) {
            unsigned char c;
            if (tx_buf.pop(c)) {
                serial->print((char)c);
            }
        }
        // String readline(void) {
            
        // }
    
};
#endif /*AsyncSerial_h*/