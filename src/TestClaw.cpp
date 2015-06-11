#include <string>
#include "Roboclaw.h"
#define address 0x80

int main(){
    Roboclaw cl(std::string("/dev/ttyS2"), 38400, address);
    std::cout << "Connected to " << cl.ReadVersion() << std::endl;
    bool valid;
    std::cout << "Voltage is " << (cl.ReadMainBatteryVoltage(&valid)/10.0) << std::endl;
    return 1; 
}
