#ifndef AsyncSerial_h
#define AsyncSerial_h

#include <Arduino.h>
#include <Print.h>
#include <RingBuf.h>

#define TX_BUFFER_SIZE 512
#define RX_BUFFER_SIZE 128

class AsyncSerial : public Print {
    private:
        RingBuf<unsigned char, TX_BUFFER_SIZE> tx_buf;
        unsigned char rx_buf[RX_BUFFER_SIZE+1];
        int rx_ptr = 0;
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
        }

        void begin(long baud) {
            serial->begin(baud);
        }

        void loop(void) {
            unsigned char c;
            if (serial->available()) {
                rx_buf[rx_ptr] = serial->read();
                if (rx_ptr < (RX_BUFFER_SIZE-1)) rx_ptr++;
            }
            if (tx_buf.pop(c)) {
                serial->print(c);
            }
        }
        String readline(void) {

        }
    
};
#endif /*AsyncSerial_h*/