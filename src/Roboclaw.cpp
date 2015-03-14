#include "Roboclaw.h"

Roboclaw::Roboclaw(const std::string port, int baud_rate, char address, int timeout)
    //:
    //_io(),
    //_port(port),
    //_serial(_io)
{


    _address = address;
    _timeout = timeout;

    _t_serial = new TimeoutSerial(port, baud_rate);
    _t_serial->setTimeout(posix_time::seconds(_timeout));


    //_serial.open(_port);
    //_serial.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));

}
Roboclaw::~Roboclaw(){
    _t_serial->close();
}

std::string Roboclaw::ReadVersion(){
    char crc;
    write(_address);
    crc=_address;
    char c = GETVERSION;
    write(c);
	crc+=GETVERSION;
    char version[32];

    for(char i=0;i<32;i++){
        version[i]=this->read();
		crc+=version[i];
		if(version[i]==0){
            if((crc&0x7F)==this->read())
                return std::string(version).substr(0,25);
			else
                return std::string("NULL");
		}
	}
	return std::string("NULL");
}

uint16_t Roboclaw::Read2(char cmd,bool *valid){
    char crc;
    write(_address);
    crc=_address;
    write(cmd);
    crc+=cmd;

    uint16_t value;
    uint8_t data = (uint8_t)read();
    crc+=data;
    value=(uint16_t)data<<8;

    data = (uint8_t)read();
    crc+=data;
    value|=(uint16_t)data;

    data = read();
    if(valid)
        *valid = ((crc&0x7F)==data);

    return value;
}

uint32_t Roboclaw::Read4_1(char cmd, char *status, bool *valid){
    char crc;
    write(_address);
    crc=_address;
    write(cmd);
    crc+=cmd;

    uint32_t value;
    uint8_t data = (uint8_t)read();
    crc+=data;
    value=(uint32_t)data<<24;

    data = (uint8_t)read();
    crc+=data;
    value|=(uint32_t)data<<16;

    data = (uint8_t)read();
    crc+=data;
    value|=(uint32_t)data<<8;

    data = (uint8_t)read();
    crc+=data;
    value|=(uint32_t)data;

    data = (uint8_t)read();
    crc+=data;
    if(status)
        *status = data;

    data = read();
    if(valid)
        *valid = ((crc&0x7F)==data);

    return value;
}

int32_t Roboclaw::ReadEncoderM1(char &status, bool &valid){
    return (int32_t) Read4_1(char(GETM1ENC), &status, &valid);
}

int32_t Roboclaw::ReadEncoderM2(char &status, bool &valid){
    return (int32_t) Read4_1(char(GETM2ENC), &status, &valid);
}

int32_t Roboclaw::ReadSpeedM1(char &status, bool &valid){
    return (int32_t) Read4_1(char(GETM1SPEED), &status, &valid);
}
int32_t Roboclaw::ReadSpeedM2(char &status, bool &valid){
    return (int32_t) Read4_1(char(GETM2SPEED), &status, &valid);
}

uint16_t Roboclaw::ReadTemperature(bool &valid){
    uint16_t temp = Read2(char(GETTEMP),&valid);
    return temp;
}

int8_t Roboclaw::ReadErrorState(bool &valid){
    char crc;
    write(_address);
    crc=_address;
    write(char(GETERROR));
    crc+=char(GETERROR);

    char value = read();
    crc+=value;

    if(valid)
        valid = ((crc&0x7F)==read());
    else
        read();

    return value;
}

void Roboclaw::ResetEncoders(){
    char crc=0;
    write(_address);
    crc+=_address;
    write(char(RESETENC));
    crc+=RESETENC;
    write(crc&0x7F);
}

int32_t Roboclaw::ReadMainBatteryVoltage(bool &valid){
    char crc=0;
    write(_address);
    crc+=_address;
    write(char(GETMBATT));
    crc+=char(GETMBATT);
    uint16_t value;
    char data = read();
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

    char crc=0;
    write(_address);
    crc+=_address;
    write(char(GETCURRENTS));
    crc+=char(GETCURRENTS);

    char data = read();
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

    char crc=0;

    write(_address);
    crc+=_address;

    write(char(SETM1PID));
    crc+=char(SETM1PID);

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

    char crc=0;

    write(_address);
    crc+=_address;

    write(char(SETM2PID));
    crc+=char(SETM2PID);

    write_32(crc, kd);
    write_32(crc, kp);
    write_32(crc,ki);
    write_32(crc,qpps);
    write(crc&0x7F);
}

void Roboclaw::SetMixedSpeed(int32_t m1_speed, int32_t m2_speed){

    char crc=0;

    write(_address);
    crc+=_address;

    write(char(MIXEDSPEED));
    crc+=char(MIXEDSPEED);

    write_32(crc, m1_speed);
    write_32(crc, m2_speed);

    write(crc&0x7F);
}

void Roboclaw::write_32( char& crc, uint32_t val){

    crc+=(char)(val>>24);
    crc+=(char)(val>>16);
    crc+=(char)(val>>8);
    crc+=(char)(val);
    char commands[4] = { (char)(val>>24), (char)(val>>16), (char)(val>>8), (char)val };
    //boost::asio::write(_serial, boost::asio::buffer(&commands,4));
    _t_serial->write((char*)&commands, 4);
}

void Roboclaw::write(char c){
    _t_serial->write(&c, 1);
}

char Roboclaw::read(){
    char c;
    _t_serial->read(&c, 1);
    return c;
}
