#ifndef CAR_HPP
#define CAR_HPP

#include <cstdint>
#include <string>

// Function declarations only
int openI2C(int addr);
void writeRegister(int file, uint8_t reg, uint8_t value);
void setPWM(int file, int channel, int on, int off);
void initPCA9685(int file);
void setMotorPWM(int file, int chA, int chB, int duty);
void setMotorModel(int file, int d1, int d2, int d3, int d4);
void moveForDuration(int file, int frontLeft, int backLeft, int frontRight, int backRight, double seconds, const std::string& action);
bool initUltrasonic();
double getDistance();

#endif // CAR_HPP