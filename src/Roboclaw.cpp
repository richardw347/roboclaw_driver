#include "Roboclaw.h"

Roboclaw::Roboclaw(const std::string port, int baud_rate, uint8_t address, int timeout):
    _io(),
    _port(port),
    _serial(_io){

    _address = address;
    _timeout = timeout;
    _serial.open(_port);
    _serial.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));

}
Roboclaw::~Roboclaw(){
    _serial.close();
}

std::string Roboclaw::ReadVersion(){
	uint8_t crc;
    write(_address);
    crc=_address;
    uint8_t c = GETVERSION;
    write(c);
	crc+=GETVERSION;
    char version[32];

    for(uint8_t i=0;i<32;i++){
        version[i]=this->read();
		crc+=version[i];
		if(version[i]==0){
            if((crc&0x7F)==this->read())
                return std::string(version).substr(0,25);
			else
                return std::string("NULL");
		}
	}
	return false;
}

uint16_t Roboclaw::Read2(uint8_t cmd,bool *valid){
    uint8_t crc;
    write(_address);
    crc=_address;
    write(cmd);
    crc+=cmd;

    uint16_t value;
    uint8_t data = read();
    crc+=data;
    value=(uint16_t)data<<8;

    data = read();
    crc+=data;
    value|=(uint16_t)data;

    data = read();
    if(valid)
        *valid = ((crc&0x7F)==data);

    return value;
}

uint32_t Roboclaw::Read4_1(uint8_t cmd, uint8_t *status, bool *valid){
    uint8_t crc;
    write(_address);
    crc=_address;
    write(cmd);
    crc+=cmd;

    long value;
    uint8_t data = read();
    crc+=data;
    value=(uint32_t)data<<24;

    data = read();
    crc+=data;
    value|=(uint32_t)data<<16;

    data = read();
    crc+=data;
    value|=(uint32_t)data<<8;

    data = read();
    crc+=data;
    value|=(uint32_t)data;

    data = read();
    crc+=data;
    if(status)
        *status = data;

    data = read();
    if(valid)
        *valid = ((crc&0x7F)==data);

    return value;
}

int32_t Roboclaw::ReadEncoderM1(uint8_t &status, bool &valid){
    return (int32_t) Read4_1(uint8_t(GETM1ENC), &status, &valid);
}

int32_t Roboclaw::ReadEncoderM2(uint8_t &status, bool &valid){
    return (int32_t) Read4_1(uint8_t(GETM2ENC), &status, &valid);
}

int32_t Roboclaw::ReadSpeedM1(uint8_t &status, bool &valid){
    return (int32_t) Read4_1(uint8_t(GETM1SPEED), &status, &valid);
}
int32_t Roboclaw::ReadSpeedM2(uint8_t &status, bool &valid){
    return (int32_t) Read4_1(uint8_t(GETM2SPEED), &status, &valid);
}

uint16_t Roboclaw::ReadTemperature(bool &valid){
    uint16_t temp = Read2(uint8_t(GETTEMP),&valid);
    return temp;
}

int8_t Roboclaw::ReadErrorState(bool &valid){
    uint8_t crc;
    write(_address);
    crc=_address;
    write(uint8_t(GETERROR));
    crc+=uint8_t(GETERROR);

    uint8_t value = read();
    crc+=value;

    if(valid)
        valid = ((crc&0x7F)==read());
    else
        read();

    return value;
}

void Roboclaw::ResetEncoders(){
    uint8_t crc=0;
    write(_address);
    crc+=_address;
    write(uint8_t(RESETENC));
    crc+=RESETENC;
    write(crc&0x7F);
}

int32_t Roboclaw::ReadMainBatteryVoltage(bool &valid){
    uint8_t crc=0;
    write(_address);
    crc+=_address;
    write(uint8_t(GETMBATT));
    crc+=uint8_t(GETMBATT);
    uint16_t value;
    uint8_t data = read();
    crc+=data;
    value=(uint16_t)data<<8;
    data = read();
    crc+=data;
    value|=(uint16_t)data;
    data = read();
    if(valid)
        valid = ((crc&0x7F)==data);
    return value;
}

bool Roboclaw::ReadCurrents(uint16_t &current1, uint16_t &current2){

    uint8_t crc=0;
    write(_address);
    crc+=_address;
    write(uint8_t(GETCURRENTS));
    crc+=uint8_t(GETCURRENTS);

    uint8_t data = read();
    crc+=data;
    current1=(uint16_t)data<<8;
    data = read();
    crc+=data;
    current1|=(uint16_t)data;

    data = read();
    crc+=data;
    current2=(uint16_t)data<<8;
    data = read();
    crc+=data;
    current2|=(uint16_t)data;

    data = read();
    if ((crc&0x7F)==data){
        return true;
    }
    return false;
}


void Roboclaw::SetM1VelocityPID(float kd_fp, float kp_fp, float ki_fp, uint32_t qpps){

    uint32_t kd = kd_fp*65536;
    uint32_t kp = kp_fp*65536;
    uint32_t ki = ki_fp*65536;

    uint8_t crc=0;

    write(_address);
    crc+=_address;

    write(uint8_t(SETM1PID));
    crc+=uint8_t(SETM1PID);

    write_32(crc, kd);
    write_32(crc, kp);
    write_32(crc,ki);
    write_32(crc,qpps);

    write(crc&0x7F);
}

void Roboclaw::SetM2VelocityPID(float kd_fp, float kp_fp, float ki_fp, uint32_t qpps){

    uint32_t kd = kd_fp*65536;
    uint32_t kp = kp_fp*65536;
    uint32_t ki = ki_fp*65536;

    uint8_t crc=0;

    write(_address);
    crc+=_address;

    write(uint8_t(SETM2PID));
    crc+=uint8_t(SETM2PID);

    write_32(crc, kd);
    write_32(crc, kp);
    write_32(crc,ki);
    write_32(crc,qpps);
    write(crc&0x7F);
}

void Roboclaw::SetMixedSpeed(uint32_t m1_speed, uint32_t m2_speed){

    uint8_t crc=0;

    write(_address);
    crc+=_address;

    write(uint8_t(MIXEDSPEED));
    crc+=uint8_t(MIXEDSPEED);

    write_32(crc, m1_speed);
    write_32(crc, m2_speed);

    write(crc&0x7F);
}

void Roboclaw::write_32( uint8_t& crc, uint32_t val){

    crc+=(uint8_t)(val>>24);
    crc+=(uint8_t)(val>>16);
    crc+=(uint8_t)(val>>8);
    crc+=(uint8_t)(val);
    uint8_t commands[4] = { (uint8_t)(val>>24), (uint8_t)(val>>16), (uint8_t)(val>>8), (uint8_t)val };
    boost::asio::write(_serial, boost::asio::buffer(&commands,4));

}

void Roboclaw::write(uint8_t c){
    boost::asio::write(_serial, boost::asio::buffer(&c,1));
}

uint8_t Roboclaw::read(){
    uint8_t c;
    boost::asio::read(_serial, boost::asio::buffer(&c,1));
    return c;
}
