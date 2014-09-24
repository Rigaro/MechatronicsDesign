#ifndef SENDSERIAL_H
#define SENDSERIAL_H

#define PORT_0 "/dev/ttyACM0"
#define PORT_1 "/dev/ttyACM1"
#define PORT_2 "/dev/ttyACM2"
#define PORT_3 "/dev/ttyACM3"

#include <fstream>

using namespace std;

void SendSerial(int, char, const char*);

#endif // SENDSERIAL_H
