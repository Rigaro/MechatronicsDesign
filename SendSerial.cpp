#include "SendSerial.h"

//void sendSerial(int angle, char axis)
//Receives the angle value and a char for the axis and sends
//the angle data through the selected port (see definitions).
//@param angle integer value.
//@param axis letter.
//@param port string, use definitions (PORT_0, PORT_1).
void SendSerial(int angle, char axis, const char* port)
{
    //Opens the port
    fstream serialData(port);
    //Sets the axis as header, angle as message and '/' as footer.
    serialData << axis << angle << "/";
    //Sends data
    serialData.flush();
}
