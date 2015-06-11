#ifndef RoboClaw_h
#define RoboClaw_h

#include <pthread.h>
#include <iostream>
#include <stdarg.h>
#include "TimeoutSerial.h"

using namespace std;
using namespace boost;

class Roboclaw{
	enum {M1FORWARD = 0,
			M1BACKWARD = 1,
			SETMINMB = 2,
			SETMAXMB = 3,
			M2FORWARD = 4,
			M2BACKWARD = 5,
			M17BIT = 6,
			M27BIT = 7,
			MIXEDFORWARD = 8,
			MIXEDBACKWARD = 9,
			MIXEDRIGHT = 10,
			MIXEDLEFT = 11,
			MIXEDFB = 12,
			MIXEDLR = 13,
			GETM1ENC = 16,
			GETM2ENC = 17,
			GETM1SPEED = 18,
			GETM2SPEED = 19,
			RESETENC = 20,
			GETVERSION = 21,
			GETMBATT = 24,
			GETLBATT = 25,
			SETMINLB = 26,
			SETMAXLB = 27,
			SETM1PID = 28,
			SETM2PID = 29,
			GETM1ISPEED = 30,
			GETM2ISPEED = 31,
			M1DUTY = 32,
			M2DUTY = 33,
			MIXEDDUTY = 34,
			M1SPEED = 35,
			M2SPEED = 36,
			MIXEDSPEED = 37,
			M1SPEEDACCEL = 38,
			M2SPEEDACCEL = 39,
			MIXEDSPEEDACCEL = 40,
			M1SPEEDDIST = 41,
			M2SPEEDDIST = 42,
			MIXEDSPEEDDIST = 43,
			M1SPEEDACCELDIST = 44,
			M2SPEEDACCELDIST = 45,
			MIXEDSPEEDACCELDIST = 46,
			GETBUFFERS = 47,
			GETCURRENTS = 49,
			MIXEDSPEED2ACCEL = 50,
			MIXEDSPEED2ACCELDIST = 51,
			M1DUTYACCEL = 52,
			M2DUTYACCEL = 53,
			MIXEDDUTYACCEL = 54,
			READM1PID = 55,
			READM2PID = 56,
			SETMAINVOLTAGES = 57,
			SETLOGICVOLTAGES = 58,
			GETMINMAXMAINVOLTAGES = 59,
			GETMINMAXLOGICVOLTAGES = 60,
			SETM1POSPID = 61,
			SETM2POSPID = 62,
			READM1POSPID = 63,
			READM2POSPID = 64,
			M1SPEEDACCELDECCELPOS = 65,
			M2SPEEDACCELDECCELPOS = 66,
			MIXEDSPEEDACCELDECCELPOS = 67,
			GETTEMP = 82,
			GETERROR = 90,
			GETENCODERMODE = 91,
			SETM1ENCODERMODE = 92,
			SETM2ENCODERMODE = 93,
			WRITENVM = 94};	




public:
    Roboclaw(const std::string port, int baud_rate, uint8_t address, int timeout=1);
    ~Roboclaw();
    std::string ReadVersion();
    int32_t ReadEncoderM1(uint8_t *status=NULL, bool *valid=NULL);
    int32_t ReadEncoderM2(uint8_t *status=NULL, bool *valid=NULL);
    int32_t ReadSpeedM1(uint8_t *status=NULL, bool *valid=NULL);
    int32_t ReadSpeedM2(uint8_t *status=NULL, bool *valid=NULL);
    int32_t ReadMainBatteryVoltage(bool *valid=NULL);
    uint16_t ReadTemperature(bool *valid=NULL);
    int8_t ReadErrorState(bool *valid=NULL);
    bool ReadCurrents(int16_t &current1, int16_t &current2);
    void SetM1VelocityPID(float kd_fp, float kp_fp, float ki_fp, uint32_t qpps);
    void SetM2VelocityPID(float kd_fp, float kp_fp, float ki_fp, uint32_t qpps);
    void SetMixedSpeed(uint32_t m1_speed, uint32_t m2_speed);
    void ResetEncoders();
    bool ReadEncoderModes(uint8_t &M1mode, uint8_t &M2mode);

    enum ErrorCodes {ERR_M1_CURRENT = 1,
                     ERR_M2_CURRENT = 2,
                     ERR_E_STOP = 4,
                     ERR_TEMP = 8,
                     ERR_MAIN_BATT_HIGH = 10,
                     ERR_MAIN_BATT_LOW = 32,
                     ERR_LOGIC_BATT_HIGH = 64,
                     ERR_LOGIC_BATT_LOW = 128};

private:
    TimeoutSerial* _t_serial;
    std::string _port;
    int _timeout;
    uint8_t _address;

    void SetM1Constants(uint32_t kd, uint32_t kp, uint32_t ki, uint32_t qpps);
    void SetM2Constants(uint32_t kd, uint32_t kp, uint32_t ki, uint32_t qpps);
    uint16_t Read2(uint8_t cmd, bool *valid);
    uint32_t Read4(uint8_t cmd, bool *valid);
    uint32_t Read4_1(uint8_t cmd, uint8_t *status, bool *valid);
    void write(char c);
    void write_n(uint8_t byte, ...);    
    uint8_t read();
};


#endif
