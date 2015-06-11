#include <string>
#include "Roboclaw.h"
#define address 0x80

int main(){
    
    // connect to device
    Roboclaw cl(std::string("/dev/ttyS2"), 38400, address);
    
    // get firmware version
    std::cout << "Connected to " << cl.ReadVersion() << std::endl;
    
    // read voltage
    bool valid;
    std::cout << "Voltage is " << (cl.ReadMainBatteryVoltage(&valid)/10.0) << std::endl;
    
    // set pid constants
    std::cout << "Set PID & QPPS constants" << std::endl;
    cl.SetM1VelocityPID(0.5,0.1,0.25,11600);

    // read temperature
    std::cout << "Temperature is " <<  (cl.ReadTemperature(&valid)/10.0) << std::endl;
    
    return 1; 
}
