#ifndef VESCPROPELLER_H
#define VESCPROPELLER_H

// Includes for VescUart library (main thruster)
// 
// https://github.com/SolidGeek/VescUart
#include <VescUart.h>
#include <buffer.h>
#include <crc.h>
#include <datatypes.h>
#include <iostream> // Include this for std::cout and std::hex

class VescPropeller
{
  public:

    typedef struct
    {
      float avgMotorCurrent;
      float avgInputCurrent;
      float dutyCycleNow;
      float rpm;
      float inpVoltage;
      float ampHours;
      float ampHoursCharged;
      float wattHours;
      float wattHoursCharged;
      long tachometer;
      long tachometerAbs;
      float tempMosfet;
      float tempMotor;
      float pidPos;
      uint8_t id;
      mc_fault_code error; 
	  } dataPackage;

    typedef struct
    {
      float roll;
      float pitch;
      float yaw;
      float accel_x;
      float accel_y;
      float accel_z;
      float gyro_x;
      float gyro_y;
      float gyro_z;
      float mag_x;
      float mag_y;
      float mag_z;
      float quat_w;
      float quat_x;
      float quat_y;
      float quat_z;
      uint8_t id;
    } imuPackage;

    typedef struct
    {
      uint8_t major;
      uint8_t minor;
    } FWversionPackage;

    VescPropeller();

    void setSerialPort(Stream* port);

    void setRPM(float rpm, uint8_t canId = 0);

    void setDuty(float rpm, uint8_t canId = 0);

    void setCurrent(float rpm, uint8_t canId = 0);

    // This just sends a binary packet to the Vesc to request the values.
    // Rx is done elsewhere to avoid blocking for a response.
    bool requestVescValues(uint8_t canId = 0);

    // Request the IMU data included in the VESC
    bool requestImuData(uint8_t canId = 0);

    // Request FW version from VESC
    bool requestFWVersion(uint8_t canId = 0);

    // This just sends this payload to the serial port
    int packSendPayload(uint8_t * payload, int lenPay);

    // (1) Feeds this class with one byte read from a serial port. Return true if the message buffer is full
    bool feed(uint8_t databyte);

    // (2) if the buffer is full, call this to unpack the payload. Returns true if the payload is successfully extracted (good CRC)
    bool unpackPayload();

    // (3) Processes a good payload and assigns to structure depending on the packet type
    bool processReadPacket(uint8_t * message);

    // Returns a pointer to the first byte in the message buffer
    uint8_t* msg();

    // Returns a pointer to the first byte in the payload included in the message
    uint8_t* getPayload();

    // Returns a pointer to the data from Vesc
    dataPackage getTheData(void);

    // Returns a pointer to the FW version from Vesc
    FWversionPackage getTheFW(void);

    // Returns a pointer to the IMU data from VESC
    imuPackage VescPropeller::getTheIMU(void);

    // Resets the buffering internals
    bool bufferReset();
    
  private:
    Stream* serialPort = NULL;

    // A buffer for incoming messages
    uint8_t message[256];
    // Counting index within the rx'd packet
    uint16_t counter;
    // Buffering utility
    uint16_t endMessage;
    uint16_t lenPayload;
    uint8_t payload[256]; // Unpacked payload rxd

    // Responses from Vesc
    FWversionPackage fw_version;
    dataPackage data;
    imuPackage imu_data;
};

#endif // VESCPROPELLER_H