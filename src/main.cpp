#include <Arduino.h>
#include <MsTimer2.h>
#include <GCodeParser.h>
#include "AsyncStepper.hpp"
#include "AsyncSerial.hpp"
#include "AsyncTimer.hpp"

#define AUTO_REPORT_SEC (1)

// (pulse, dir, ena, max, min) 
AsyncStepper MotorI(29,30,31,32,33);
AsyncStepper MotorJ(34,35,36,37,38);
AsyncStepper MotorK(39,40,41,42,43);
AsyncStepper MotorL(44,45,46,47,48);
AsyncStepper MotorM(49,50,51,52,53);
AsyncSerial ASerial(0);//0(RX), 1(TX)
AsyncSerial ASerial1(1);//19(RX), 18(TX)	
AsyncTimer ATimer;
GCodeParser GCode;

#define MSTIMER_MILLIS (10)
static uint64_t t_millis = 0;
float secs(void) {
    volatile uint64_t temp;
    noInterrupts();
    temp = t_millis;
    interrupts();
    return (float)((double)temp * MSTIMER_MILLIS / 1000.0);
}
void tick() {
  t_millis++;
}

void setup() {
    MsTimer2::set(MSTIMER_MILLIS, tick);
    MsTimer2::start();
    ASerial.begin(921600);
    //serial1.begin(921600);
    // MotorI.start(1, 0.5);
    // MotorJ.start(1, 1);
    // MotorK.start(1, 5);
    // MotorL.start(1, 10);
    // MotorM.start(1, 30);
    ATimer.init();
    ATimer.next_sec(AUTO_REPORT_SEC);
}

void gcode_loop() {
    if (!Serial.available()) return;
    if (!GCode.AddCharToLine(ASerial.read())) return;
    GCode.ParseLine();
    ASerial.print("Command Line: ");
    ASerial.println(GCode.line);
    if (GCode.HasWord('G')) {
        int k_val = (int)GCode.GetWordValue('G');
        switch(k_val) {
            case 90:
                // ABCDEモータ駆動(アブソリュート指令)
                // K91 I_ J_ K_ L_ M_ V_ W_ X_ Y_ Z_
                // I-M:モータ位置[mm]
                // V:モータI速度[mm/min]
                // W:モータJ速度[mm/min]
                // X:モータK速度[mm/min]
                // Y:モータL速度[mm/min]
                // Z:モータM速度[mm/min]
                if (GCode.HasWord('I') && GCode.HasWord('V')) MotorI.start(GCode.GetWordValue('I'), GCode.GetWordValue('V'));
                if (GCode.HasWord('J') && GCode.HasWord('W')) MotorJ.start(GCode.GetWordValue('J'), GCode.GetWordValue('W'));
                if (GCode.HasWord('K') && GCode.HasWord('X')) MotorK.start(GCode.GetWordValue('K'), GCode.GetWordValue('X'));
                if (GCode.HasWord('L') && GCode.HasWord('Y')) MotorL.start(GCode.GetWordValue('L'), GCode.GetWordValue('Y'));
                if (GCode.HasWord('M') && GCode.HasWord('Z')) MotorM.start(GCode.GetWordValue('M'), GCode.GetWordValue('Z'));
                break;
            case 91:
                //ABCDEモータ駆動(インクリメンタル指令)
                if (GCode.HasWord('I') && GCode.HasWord('V')) MotorI.start(GCode.GetWordValue('I'), GCode.GetWordValue('V'), true);
                if (GCode.HasWord('J') && GCode.HasWord('W')) MotorJ.start(GCode.GetWordValue('J'), GCode.GetWordValue('W'), true);
                if (GCode.HasWord('K') && GCode.HasWord('X')) MotorK.start(GCode.GetWordValue('K'), GCode.GetWordValue('X'), true);
                if (GCode.HasWord('L') && GCode.HasWord('Y')) MotorL.start(GCode.GetWordValue('L'), GCode.GetWordValue('Y'), true);
                if (GCode.HasWord('M') && GCode.HasWord('Z')) MotorM.start(GCode.GetWordValue('M'), GCode.GetWordValue('Z'), true);
                break;
            case 52:
                //ローカル座標系設定
                if (GCode.HasWord('I')) MotorI.set_mm(GCode.GetWordValue('I'));
                if (GCode.HasWord('J')) MotorJ.set_mm(GCode.GetWordValue('J'));
                if (GCode.HasWord('K')) MotorK.set_mm(GCode.GetWordValue('K'));
                if (GCode.HasWord('L')) MotorL.set_mm(GCode.GetWordValue('L'));
                if (GCode.HasWord('M')) MotorM.set_mm(GCode.GetWordValue('M'));
                break;
            case 100:
                // ヘルプ
                break;
        }
    }
}

bool all_motor_stop(){
    if (!MotorI.is_stop()) return false;
    if (!MotorJ.is_stop()) return false;
    if (!MotorK.is_stop()) return false;
    if (!MotorL.is_stop()) return false;
    if (!MotorM.is_stop()) return false;
    return true;
}

void motor_loop(){
    MotorI.loop();
    MotorJ.loop();
    MotorK.loop();
    MotorL.loop();
    MotorM.loop();
}

void report() {
    if (ATimer.trigger_and_next()) {
        ASerial.print("REPORT: {");
        ASerial.print(String("\"Time\":") + String(secs(), 3) + ",");
        if (all_motor_stop()) ASerial.print("\"Status\":\"IDLE\",");
        else ASerial.print("\"Status\":\"BUSY\",");
        ASerial.print(String("\"I\":") + String(MotorI.get_mm(), 4) + ",");
        ASerial.print(String("\"J\":") + String(MotorJ.get_mm(), 4) + ",");
        ASerial.print(String("\"K\":") + String(MotorK.get_mm(), 4) + ",");
        ASerial.print(String("\"L\":") + String(MotorL.get_mm(), 4) + ",");
        ASerial.print(String("\"M\":") + String(MotorM.get_mm(), 4));
        ASerial.println("}");
    }
}

void loop() {
    if(all_motor_stop()) gcode_loop();
    motor_loop();
    report();
    ASerial.loop();
}