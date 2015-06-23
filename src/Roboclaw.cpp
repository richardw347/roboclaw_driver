#include "Roboclaw.h"

Roboclaw::Roboclaw(const std::string port, int baud_rate, uint8_t address, float timeout)
{
    _address = address;
    _t_serial = new TimeoutSerial(port, baud_rate);
    _t_serial->setTimeout(posix_time::seconds(timeout));

}
Roboclaw::~Roboclaw(){
    _t_serial->close();
}

std::string Roboclaw::ReadVersion(){
    std::string version;
    version.resize(32);

    uint8_t crc;
    write(_address);
    crc=_address;
    write(GETVERSION);
    crc+=GETVERSION;

    for(uint8_t i=0;i<32;i++) {
      version[i] = read();
      crc += version[i];
      if (version[i] == 0) {
          if((crc&0x7F)==this->read()){
	      return version;
          } else {
              return std::string("NULL");
	  }
      }
    }
    return std::string("NULL");
}

uint16_t Roboclaw::Read2(uint8_t cmd, bool *valid){
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

uint32_t Roboclaw::Read4(uint8_t cmd, bool *valid){

  uint8_t crc;
  write(_address);
  crc=_address;
  write(cmd);
  crc+=cmd;

  uint32_t value;
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

    uint32_t value;
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

bool Roboclaw::ReadEncoderModes(uint8_t &M1mode, uint8_t &M2mode){

	bool valid;
	uint16_t value = Read2(GETENCODERMODE,&valid);
	if(valid){
		M1mode = value>>8;
		M2mode = value;
	}
	return valid;
}

int32_t Roboclaw::ReadEncoderM1(uint8_t *status, bool *valid){
    return (int32_t) Read4_1(GETM1ENC, status, valid);
}

int32_t Roboclaw::ReadEncoderM2(uint8_t *status, bool *valid){
    return (int32_t) Read4_1(GETM2ENC, status, valid);
}

int32_t Roboclaw::ReadSpeedM1(uint8_t *status, bool *valid){
    return (int32_t) Read4_1(GETM1SPEED, status, valid);
}
int32_t Roboclaw::ReadSpeedM2(uint8_t *status, bool *valid){
    return (int32_t) Read4_1(GETM2SPEED, status, valid);
}

uint16_t Roboclaw::ReadTemperature(bool *valid){
    uint16_t temp = Read2(GETTEMP, valid);
    return temp;
}

uint16_t Roboclaw::ReadErrorState(bool *valid){

    return Read2(GETERROR, valid);
}

void Roboclaw::ResetEncoders(){
    write_n(2, _address, RESETENC);
}

int32_t Roboclaw::ReadMainBatteryVoltage(bool *valid){
    return Read2(GETMBATT, valid);
}

bool Roboclaw::ReadCurrents(int16_t &current1, int16_t &current2){
  bool valid;
  uint32_t value = Read4(GETCURRENTS, &valid);
  if(valid) {
    current1 = value>>16;
    current2 = value;
  }
  return valid;
}

bool Roboclaw::SetM1VelocityPID(float kd_fp, float kp_fp, float ki_fp, uint32_t qpps){

    uint32_t kd = kd_fp*65536;
    uint32_t kp = kp_fp*65536;
    uint32_t ki = ki_fp*65536;
    return write_n(18,_address,SETM1PID,SetDWORDval(kd),SetDWORDval(kp),SetDWORDval(ki),SetDWORDval(qpps));

}

bool Roboclaw::SetM2VelocityPID(float kd_fp, float kp_fp, float ki_fp, uint32_t qpps){

    uint32_t kd = kd_fp*65536;
    uint32_t kp = kp_fp*65536;
    uint32_t ki = ki_fp*65536;
    return write_n(18,_address,SETM2PID,SetDWORDval(kd),SetDWORDval(kp),SetDWORDval(ki),SetDWORDval(qpps));


}

void Roboclaw::SetMixedSpeed(int32_t m1_speed, int32_t m2_speed){
  write_n(10, _address, MIXEDSPEED, SetDWORDval(m1_speed), SetDWORDval(m2_speed));
}

void Roboclaw::SetSpeedM1(uint32_t speed) {
  write_n(6,_address,M1SPEED,SetDWORDval(speed));
}

void Roboclaw::SetSpeedM2(uint32_t speed) {
  write_n(6,_address,M2SPEED,SetDWORDval(speed));
}

bool Roboclaw::write_n(uint8_t cnt, ... ) {
  static char buff[256];
  int ind = 0;
  uint8_t crc=0;

  va_list marker;
  va_start(marker, cnt);

  for(uint8_t index=0; index < cnt; index++) {
    uint8_t data = va_arg(marker, int);
    crc += data;
    buff[ind++] = static_cast<char>(data);
  }
  va_end(marker);
  buff[ind++] = static_cast<char>(crc & 0x7F | 0x80);
  _t_serial->write(buff, cnt+1);
  if(read()==0xFF)
      return true;
  return false;
}

void Roboclaw::write(uint8_t c){
    _t_serial->write(reinterpret_cast<const char*>(&c), 1);
}

uint8_t Roboclaw::read(){
   char c;
   _t_serial->read(&c, 1);
   return static_cast<uint8_t>(c);
}
