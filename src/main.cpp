#include <Arduino.h>
#include <MsTimer2.h>
#include <GCodeParser.h>
#include <EEPROMWearLevel.h>

#include "AsyncStepper.hpp"
#include "AsyncSerial.hpp"
#include "AsyncTimer.hpp"

#define AUTO_REPORT_MSEC (200)
#define MSTIMER_MILLIS (10)

#define DUMMY_FIRMWARE (0)

/**
 * @brief 非同期ステッピングモータドライバ
 * pulse, dir, ena, max, min
 * モーターI pulse:29, dir:30, ena:31, max:32, min33
 * モーターJ pulse:34, dir:35, ena:36, max:37, min38
 * モーターK pulse:39, dir:40, ena:41, max:42, min43
 * モーターL pulse:44, dir:45, ena:46, max:47, min48
 * モーターM pulse:49, dir:50, ena:51, max:52, min53
 */
AsyncStepper MotorI(29,30,31,32,33);
AsyncStepper MotorJ(34,35,36,37,38);
AsyncStepper MotorK(39,40,41,42,43);
AsyncStepper MotorL(44,45,46,47,48);
AsyncStepper MotorM(49,50,51,52,53);

AsyncSerial ASerial(0);//0(RX), 1(TX)
//AsyncSerial Serial(1);//19(RX), 18(TX)	
GCodeParser GCode;

//非同期タイマー
AsyncTimer ATimer;

static uint64_t t_millis = 0;
void tick() { t_millis++; }
float secs(void) {
    volatile uint64_t temp;
    noInterrupts();
    temp = t_millis;
    interrupts();
    return (float)((float)temp * MSTIMER_MILLIS / 1000.0);
}

typedef enum GCODE_STATE {
    GCODE_IDLE = 0,
    GCODE_RUNNING
} GCODE_STATE_T;
static GCODE_STATE_T state = GCODE_IDLE;

#define EEPROM_LAYOUT_VERSION 0
#define EEPROM_INDEX_NUM 20
enum EEPROM_INDEX {
    EEPROM_INDEX_MOTOR_I = 0,
    EEPROM_INDEX_MOTOR_J,
    EEPROM_INDEX_MOTOR_K,
    EEPROM_INDEX_MOTOR_L,
    EEPROM_INDEX_MOTOR_M
};

void eeprom_init(void){
    long val = 0;
    EEPROMwl.put(EEPROM_INDEX_MOTOR_I, val);
    EEPROMwl.put(EEPROM_INDEX_MOTOR_J, val);
    EEPROMwl.put(EEPROM_INDEX_MOTOR_K, val);
    EEPROMwl.put(EEPROM_INDEX_MOTOR_L, val);
    EEPROMwl.put(EEPROM_INDEX_MOTOR_M, val);
}
void report(bool forced=false);
void motor_loop(void);

void setup() {
    MsTimer2::set(MSTIMER_MILLIS, tick);
    MsTimer2::start();
    ASerial.begin(115200);
    ATimer.init();
    ATimer.next_millisec(AUTO_REPORT_MSEC);
    EEPROMwl.begin(EEPROM_LAYOUT_VERSION, EEPROM_INDEX_NUM);
    
    long steps = -1;
    EEPROMwl.get(EEPROM_INDEX_MOTOR_I, steps);
    MotorI.set_current_steps(steps);
    EEPROMwl.get(EEPROM_INDEX_MOTOR_J, steps);
    MotorJ.set_current_steps(steps);
    EEPROMwl.get(EEPROM_INDEX_MOTOR_K, steps);
    MotorK.set_current_steps(steps);
    EEPROMwl.get(EEPROM_INDEX_MOTOR_L, steps);
    MotorL.set_current_steps(steps);
    EEPROMwl.get(EEPROM_INDEX_MOTOR_M, steps);
    MotorM.set_current_steps(steps);
}

// ABCDEモータ駆動(アブソリュート指令/インクリメンタル指令)
// G90 I5 I_ J_ K_ L_ M_ V_ W_ X_ Y_ Z_
// G91 I5 I_ J_ K_ L_ M_ V_ W_ X_ Y_ Z_
// I-M:モータ位置[mm]
// V:モータI速度[mm/min]
// W:モータJ速度[mm/min]
// X:モータK速度[mm/min]
// Y:モータL速度[mm/min]
// Z:モータM速度[mm/min]
void G90_91(GCodeParser *gcode, bool incremental = false) {
    if (gcode->HasWord('I') && gcode->HasWord('V')) MotorI.start(gcode->GetWordValue('I'), gcode->GetWordValue('V'), incremental);
    if (gcode->HasWord('J') && gcode->HasWord('W')) MotorJ.start(gcode->GetWordValue('J'), gcode->GetWordValue('W'), incremental);
    if (gcode->HasWord('K') && gcode->HasWord('X')) MotorK.start(gcode->GetWordValue('K'), gcode->GetWordValue('X'), incremental);
    if (gcode->HasWord('L') && gcode->HasWord('Y')) MotorL.start(gcode->GetWordValue('L'), gcode->GetWordValue('Y'), incremental);
    if (gcode->HasWord('M') && gcode->HasWord('Z')) MotorM.start(gcode->GetWordValue('M'), gcode->GetWordValue('Z'), incremental);
}

//ローカル座標系設定
// I-M:現在のモータ位置[mm]
void G52(GCodeParser *gcode) {
    if (gcode->HasWord('I')) {
        MotorI.set_current_mm(gcode->GetWordValue('I'));
        EEPROMwl.put(EEPROM_INDEX_MOTOR_I, MotorI.get_current_steps());
    }
    if (gcode->HasWord('J')) {
        MotorJ.set_current_mm(gcode->GetWordValue('J'));
        EEPROMwl.put(EEPROM_INDEX_MOTOR_J, MotorJ.get_current_steps());
    }
    if (gcode->HasWord('K'))  {
        MotorK.set_current_mm(gcode->GetWordValue('K'));
        EEPROMwl.put(EEPROM_INDEX_MOTOR_K, MotorK.get_current_steps());
    }
    if (gcode->HasWord('L')) {
        MotorL.set_current_mm(gcode->GetWordValue('L'));
        EEPROMwl.put(EEPROM_INDEX_MOTOR_L, MotorL.get_current_steps());
    }
    if (gcode->HasWord('M')) {
        MotorM.set_current_mm(gcode->GetWordValue('M'));
        EEPROMwl.put(EEPROM_INDEX_MOTOR_M, MotorM.get_current_steps());
    }
}

void gcode_loop() {
    if (state != GCODE_IDLE) return;
    if (!ASerial.available()) return;
    if (!GCode.AddCharToLine(ASerial.read())) return;
    GCode.ParseLine();
    if (GCode.HasWord('G')) {
        int k_val = (int)GCode.GetWordValue('G');
        switch(k_val) {
            case 90:
                //Absolute
                G90_91(&GCode, false);
                state = GCODE_RUNNING;
                report(true);
                break;
            case 91:
                //Incremental
                G90_91(&GCode, true);
                state = GCODE_RUNNING;
                report(true);
                break;
            case 52:
                //Set Local Position
                G52(&GCode);
                report(true);
                break;
            case 28:
                // @todo
                // Auto Homing
                break;
            case 17:
                // Enable Stepper(Lock)
                MotorI.stepper_power(true);
                MotorJ.stepper_power(true);
                MotorK.stepper_power(true);
                MotorL.stepper_power(true);
                MotorM.stepper_power(true);
                report(true);
                break;
            case 18:
                // Disable Stepper(Unlock)
                MotorI.stepper_power(false);
                MotorJ.stepper_power(false);
                MotorK.stepper_power(false);
                MotorL.stepper_power(false);
                MotorM.stepper_power(false);
                report(true);
                break;
            default:
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

void report_json(bool forced) {
    static char dbuf[8];
    if (forced || (ATimer.trigger_and_next())) {
        ASerial.print("REPORT: {\"Time\":");
        ASerial.print(dtostrf(secs(),0,3,dbuf));
        if (state==GCODE_IDLE) ASerial.print(",\"Status\":\"IDLE\",");
        else ASerial.print(",\"Status\":\"BUSY\",");
        ASerial.print("\"I\":");
        ASerial.print(dtostrf(MotorI.get_current_mm(),0,3,dbuf));
        ASerial.print(",\"J\":");
        ASerial.print(dtostrf(MotorJ.get_current_mm(),0,3,dbuf));
        ASerial.print(",\"K\":");
        ASerial.print(dtostrf(MotorK.get_current_mm(),0,3,dbuf));
        ASerial.print(",\"L\":");
        ASerial.print(dtostrf(MotorL.get_current_mm(),0,3,dbuf));
        ASerial.print(",\"M\":");
        ASerial.print(dtostrf(MotorM.get_current_mm(),0,3,dbuf));
        ASerial.println("}");
    }
}

void report_short(bool forced) {
    static char buf[128];
    static char dbuf[8];
    pinMode(7, OUTPUT);
    if (forced || (ATimer.trigger_and_next())) {
        int bid = 0;
        memset(buf, 0x00, sizeof(buf));
        bid += sprintf(&buf[bid], "T:%s", dtostrf(secs(),0,3,dbuf));
        if (state==GCODE_IDLE) bid += sprintf(&buf[bid], ",S:I");
        else bid += sprintf(&buf[bid], ",S:B");
        bid += sprintf(&buf[bid], ",I:%s", dtostrf(MotorI.get_current_mm(),0,3,dbuf));
        bid += sprintf(&buf[bid], ",J:%s", dtostrf(MotorJ.get_current_mm(),0,3,dbuf));
        bid += sprintf(&buf[bid], ",K:%s", dtostrf(MotorK.get_current_mm(),0,3,dbuf));
        bid += sprintf(&buf[bid], ",L:%s", dtostrf(MotorL.get_current_mm(),0,3,dbuf));
        bid += sprintf(&buf[bid], ",M:%s", dtostrf(MotorM.get_current_mm(),0,3,dbuf));
        ASerial.println(buf);
    }
}

void report(bool forced){report_json(forced);};

void motor_loop(){
    MotorI.loop();
    MotorJ.loop();
    MotorK.loop();
    MotorL.loop();
    MotorM.loop();
    if (all_motor_stop() && state != GCODE_IDLE) {
        state = GCODE_IDLE;
        report(true);
        EEPROMwl.put(EEPROM_INDEX_MOTOR_I, MotorI.get_current_steps());
        EEPROMwl.put(EEPROM_INDEX_MOTOR_J, MotorJ.get_current_steps());
        EEPROMwl.put(EEPROM_INDEX_MOTOR_K, MotorK.get_current_steps());
        EEPROMwl.put(EEPROM_INDEX_MOTOR_L, MotorL.get_current_steps());
        EEPROMwl.put(EEPROM_INDEX_MOTOR_M, MotorM.get_current_steps());
    }
}

void loop() {
    gcode_loop();
    motor_loop();
    report(false);
    ASerial.loop();
}