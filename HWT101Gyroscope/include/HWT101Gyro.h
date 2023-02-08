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
 * -HunterL
 *
 * 2023.2.6
 * updated: more robust on Serial reading process
 * updated: check bit fully funtional
 * -HunterL
 * 
 * 2023.2.7
 * updated: Add heading
 * -HunterL
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
        if (gyroSerial->available() >= messageLength)
            if (0x55 == gyroSerial->read()) // 校验头
                if (readUart())
                    if (decode())
                        return true;
        return false;
    }
    void printValue()
    {
        USE_SERIAL.printf("gyro_yaw_angle=%4f, ", this->yaw_angle);
        // USE_SERIAL.println("gyro_heading_raw=%4d, ", this->heading);
        USE_SERIAL.printf("gyro_heading_angle=%6f, ", this->heading_angle);
        // USE_SERIAL.println("gyro_qual=%4d, ", this->qual);
        USE_SERIAL.println("");
    }

private:
    HardwareSerial *gyroSerial;
    const static uint8_t messageLength = 10;
    byte UartRx[messageLength];
    bool decode()
    {
        uint8_t Check_sum = 0;
        static int16_t yaw;
        if (UartRx[0] == 0x53) // 角度输出
        {
            Check_sum = (uint8_t)(0x55 + UartRx[0] + UartRx[5] + UartRx[6] + UartRx[7] + UartRx[8]);
            if (UartRx[9] == Check_sum) // 校验和正确
            {
                qual = 255;
                yaw = (UartRx[6] << 8) | UartRx[5];
                heading = yaw;
                heading_angle = (float)heading / 32768 * 180;
                heading_rad = (float)heading / 32768 * 3.14159;
                return true;
            }
        }
        else if (UartRx[0] == 0x52) // 角速度输出
        {
            Check_sum = (uint8_t)(0x55 + UartRx[0] + UartRx[3] + UartRx[4] + UartRx[5] + UartRx[6]);
            if (UartRx[9] == Check_sum) // 校验和正确
            {
                qual = 255;
                yaw = (UartRx[6] << 8) | UartRx[5];
                this->yaw = yaw;
                yaw_angle = (float)yaw / 32768 * 2000;
                yaw_rad = yaw_angle / 180 * 3.14159;
                return true;
            }
        }
        qual = 0;
        USE_SERIAL.println("Gyro decode failure");
        return false;
    }
    bool readUart()
    {
        uint8_t i = 0;
        while (gyroSerial->available() < messageLength)
        {
            if (i++ > 250)
            {
                USE_SERIAL.println("Gyro read failure");
                return false;
            }
        }
        for (uint8_t n = 0; n < messageLength; n++)
        {
            UartRx[n] = gyroSerial->read();
        }
        return true;
    }
};
#endif
