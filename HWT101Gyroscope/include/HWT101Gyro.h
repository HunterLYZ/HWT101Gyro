#include <Arduino.h>

#ifndef USE_SERIAL
#define USE_SERIAL Serial
#endif

#ifndef HWT101Gyro_H
#define HWT101Gyro_H

/**
 * @brief
 * 2023.1.28: unsolved problem
 * Uart check bit feature failure
 * -HunterL
 * 
 * 2023.2.3:
 * add angle and rad unit process
 */
class HWT101Gyro
{
public:
    int yaw;
    float yaw_angle;
    float yaw_rad;
    int heading = 0;
    float heading_angle = 0;
    float heading_rad = 0;
    uint8_t qual;

    HWT101Gyro(HardwareSerial *mySerial, int8_t rxPin = -1, int8_t txPin = -1)
    {
        gyroSerial = mySerial;
        gyroSerial->begin(115200, SERIAL_8N1, rxPin, txPin);
        USE_SERIAL.print("HWT101 begin");
    }

    bool available()
    {
        if (readUart())
            if (decode())
                return true;
        return false;
    }
    void printValue()
    {
        USE_SERIAL.printf("gyro_yaw=%4d, ", this->yaw);
        USE_SERIAL.printf("gyro_heading_raw=%4d, ", this->heading);
        USE_SERIAL.printf("gyro_heading_angle=%6f, ", this->heading_angle);
        USE_SERIAL.printf("gyro_qual=%4d, ", this->qual);
        USE_SERIAL.println("");
    }

private:
    HardwareSerial *gyroSerial;
    byte UartRx[11];
    bool decode()
    {
        // uint8_t Check_sum = 0;
        static int16_t yaw;
        if (UartRx[0] == 0x55) // 校验头
        {
            // Check_sum = (uint8_t)(UartRx[0] + UartRx[1] + UartRx[6] + UartRx[7] + UartRx[8] + UartRx[9]);
            //  if (UartRx[10] == Check_sum) // 校验和正确
            {
                yaw = UartRx[6] + (UartRx[7] << 8);
                qual = 255;
                this->yaw = yaw;
                yaw_angle = (float)yaw / 32768 * 18;
                yaw_rad = (float)yaw / 32768 / 10 * 3.14159;
                heading += yaw;
                heading_angle = (float)heading / 32768 * 18;
                heading_rad = (float)heading / 32768 / 10 * 3.14159;
                return true;
            }
        }
        qual = 0;
        Serial.println("decode failure");
        return false;
    }
    bool readUart()
    {
        int j = gyroSerial->available();
        if (j != 0)
        {
            for (int n = 0; n < j; n++)
            {
                UartRx[n] = gyroSerial->read();
            }
            return true;
        }
        return false;
    }
};
#endif