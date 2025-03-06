#include "vesc_propeller.h"

VescPropeller::VescPropeller()
{
  bufferReset();
}

void VescPropeller::setSerialPort(Stream* port)
{
	serialPort = port;
}

void VescPropeller::setRPM(float rpm, uint8_t canId)
{
	int32_t index = 0;
	int payloadSize = (canId == 0 ? 5 : 7);
	uint8_t payload[payloadSize];
	if (canId != 0) {
		payload[index++] = { COMM_FORWARD_CAN };
		payload[index++] = canId;
	}
	payload[index++] = { COMM_SET_RPM };
	buffer_append_int32(payload, (int32_t)(rpm), &index);
	packSendPayload(payload, payloadSize);
}

void VescPropeller::setDuty(float duty, uint8_t canId)
{
	int32_t index = 0;
	int payloadSize = (canId == 0 ? 5 : 7);
	uint8_t payload[payloadSize];
	if (canId != 0) {
		payload[index++] = { COMM_FORWARD_CAN };
		payload[index++] = canId;
	}
	payload[index++] = { COMM_SET_DUTY };
	buffer_append_int32(payload, (int32_t)(duty * 100000), &index);

	packSendPayload(payload, payloadSize);
}

void VescPropeller::setCurrent(float current, uint8_t canId)
{
	int32_t index = 0;
	int payloadSize = (canId == 0 ? 5 : 7);
	uint8_t payload[payloadSize];
	if (canId != 0) {
		payload[index++] = { COMM_FORWARD_CAN };
		payload[index++] = canId;
	}
	payload[index++] = { COMM_SET_CURRENT };
	buffer_append_int32(payload, (int32_t)(current * 1000), &index);
	packSendPayload(payload, payloadSize);
}

bool VescPropeller::requestVescValues(uint8_t canId)
{

	int32_t index = 0;
	int payloadSize = (canId == 0 ? 1 : 3);
	uint8_t payload[payloadSize];
	if (canId != 0) {
		payload[index++] = { COMM_FORWARD_CAN };
		payload[index++] = canId;
	}
	payload[index++] = { COMM_GET_VALUES };

	packSendPayload(payload, payloadSize);

	return false;
}

bool VescPropeller::requestImuData(uint8_t canId)
{

	int32_t index = 0;
	int payloadSize = (canId == 0 ? 3 : 5);
	uint8_t payload[payloadSize];
	if (canId != 0) {
		payload[index++] = { COMM_FORWARD_CAN };
		payload[index++] = canId;
	}
	payload[index++] = { COMM_GET_IMU_DATA };
  payload[index++] = 0xFF; // Add Mask, request all fields
	payload[index++] = 0xFF; // Add Mask, request all fields

	packSendPayload(payload, payloadSize);

	return false;
}

bool VescPropeller::requestFWVersion(uint8_t canId)
{

	int32_t index = 0;
	int payloadSize = (canId == 0 ? 1 : 3);
	uint8_t payload[payloadSize];
	if (canId != 0) {
		payload[index++] = { COMM_FORWARD_CAN };
		payload[index++] = canId;
	}
	payload[index++] = { COMM_FW_VERSION };

	packSendPayload(payload, payloadSize);

	return false;
}

int VescPropeller::packSendPayload(uint8_t * payload, int lenPay)
{

	uint16_t crcPayload = crc16(payload, lenPay);
	int count = 0;
	uint8_t messageSend[256];
	
	if (lenPay <= 256)
	{
		messageSend[count++] = 2;
		messageSend[count++] = lenPay;
	}
	else
	{
		messageSend[count++] = 3;
		messageSend[count++] = (uint8_t)(lenPay >> 8);
		messageSend[count++] = (uint8_t)(lenPay & 0xFF);
	}

	memcpy(messageSend + count, payload, lenPay);
	count += lenPay;

	messageSend[count++] = (uint8_t)(crcPayload >> 8);
	messageSend[count++] = (uint8_t)(crcPayload & 0xFF);
	messageSend[count++] = 3;
	
	// Sending package
	if( serialPort != NULL )
		serialPort->write(messageSend, count);

	// Returns number of send bytes
	return count;
}

bool VescPropeller::feed(uint8_t databyte)
{
  message[counter++] = databyte;

	if (counter == 2)
  {
		switch (message[0])
		{
			case 2:
				endMessage = message[1] + 5; //Payload size + 2 for sice + 3 for SRC and End.
				lenPayload = message[1];
			break;

			case 3:
				// ToDo: Add Message Handling > 255 (starting with 3)
			break;

			default: // Invalid start bit
        counter = 0; // Restart from the beginning
			break;
		}
	}

	if (counter >= sizeof(message))
  {
    counter = 0; // Restart from the beginning
		return false;
	}

	if (counter == endMessage && message[endMessage - 1] == 3)
  {
		message[endMessage] = 0;
		// End of message reached
    counter = 0; // Prepare for next buffer to restart
		return true;
	}
  
  return false;
}

bool VescPropeller::unpackPayload()
{
	uint16_t crcMessage = 0;
	uint16_t crcPayload = 0;

	// Rebuild crc:
	crcMessage = message[endMessage - 3] << 8;
	crcMessage &= 0xFF00;
	crcMessage += message[endMessage - 2];

	// Extract payload:
	memcpy(payload, &message[2], message[1]);

	crcPayload = crc16(payload, message[1]);

	if (crcPayload == crcMessage)
  {
		// Just print OK
    bufferReset();
    return true;
	}	
	else
  {
    bufferReset();
		return false;
	}
}

bool VescPropeller::processReadPacket(uint8_t * message)
{
	COMM_PACKET_ID packetId;
	int32_t index = 0;

	packetId = (COMM_PACKET_ID)message[0];
	message++; // Removes the packetId from the actual message (payload)

	switch (packetId){
		case COMM_FW_VERSION: { // Structure defined here: https://github.com/vedderb/bldc/blob/43c3bbaf91f5052a35b75c2ff17b5fe99fad94d1/commands.c#L164

			fw_version.major = message[index++];
			fw_version.minor = message[index++];
			
      return true;
    }

		case COMM_GET_VALUES: { // Structure defined here: https://github.com/vedderb/bldc/blob/43c3bbaf91f5052a35b75c2ff17b5fe99fad94d1/commands.c#L164

			data.tempMosfet 		= buffer_get_float16(message, 10.0, &index); 	// 2 bytes - mc_interface_temp_fet_filtered()
			data.tempMotor 			= buffer_get_float16(message, 10.0, &index); 	// 2 bytes - mc_interface_temp_motor_filtered()
			data.avgMotorCurrent 	= buffer_get_float32(message, 100.0, &index); // 4 bytes - mc_interface_read_reset_avg_motor_current()
			data.avgInputCurrent 	= buffer_get_float32(message, 100.0, &index); // 4 bytes - mc_interface_read_reset_avg_input_current()
			index += 4; // Skip 4 bytes - mc_interface_read_reset_avg_id()
			index += 4; // Skip 4 bytes - mc_interface_read_reset_avg_iq()
			data.dutyCycleNow 		= buffer_get_float16(message, 1000.0, &index); 	// 2 bytes - mc_interface_get_duty_cycle_now()
			data.rpm 				= buffer_get_float32(message, 1.0, &index);		// 4 bytes - mc_interface_get_rpm()
			data.inpVoltage 		= buffer_get_float16(message, 10.0, &index);		// 2 bytes - GET_INPUT_VOLTAGE()
			data.ampHours 			= buffer_get_float32(message, 10000.0, &index);	// 4 bytes - mc_interface_get_amp_hours(false)
			data.ampHoursCharged 	= buffer_get_float32(message, 10000.0, &index);	// 4 bytes - mc_interface_get_amp_hours_charged(false)
			data.wattHours			= buffer_get_float32(message, 10000.0, &index);	// 4 bytes - mc_interface_get_watt_hours(false)
			data.wattHoursCharged	= buffer_get_float32(message, 10000.0, &index);	// 4 bytes - mc_interface_get_watt_hours_charged(false)
			data.tachometer 		= buffer_get_int32(message, &index);				// 4 bytes - mc_interface_get_tachometer_value(false)
			data.tachometerAbs 		= buffer_get_int32(message, &index);				// 4 bytes - mc_interface_get_tachometer_abs_value(false)
			data.error 				= (mc_fault_code)message[index++];								// 1 byte  - mc_interface_get_fault()
			data.pidPos				= buffer_get_float32(message, 1000000.0, &index);	// 4 bytes - mc_interface_get_pid_pos_now()
			data.id					= message[index++];								// 1 byte  - app_get_configuration()->controller_id	

			return true;
    }

    case COMM_GET_IMU_DATA: { // Structure defined here: https://github.com/vedderb/bldc/blob/5dde8f3dfe6ba1f1e5a1ad6d62633e4c479ec531/comm/commands.c#L1060

      // Use mask to check which data is available and then conditionally unpack data
      uint32_t mask = buffer_get_uint16(message, &index);  // Read the 16-bit mask (2 bytes)

      // Print the mask for debugging
      //std::cout << "IMU Data Mask: 0x" << std::hex << mask << std::dec << std::endl;

      // Roll, Pitch, Yaw  
      if (mask & ((uint32_t)1 << 0)) {
          imu_data.roll   = buffer_get_float32_auto(message, &index); // Roll
      }
      if (mask & ((uint32_t)1 << 1)) {
          imu_data.pitch  = buffer_get_float32_auto(message, &index); // Pitch
      }
      if (mask & ((uint32_t)1 << 2)) {
          imu_data.yaw    = buffer_get_float32_auto(message, &index); // Yaw
      }

      // Acceleration
      if (mask & ((uint32_t)1 << 3)) {
          imu_data.accel_x = buffer_get_float32_auto(message, &index); // Acceleration X
      }
      if (mask & ((uint32_t)1 << 4)) {
          imu_data.accel_y = buffer_get_float32_auto(message, &index); // Acceleration Y
      }
      if (mask & ((uint32_t)1 << 5)) {
          imu_data.accel_z = buffer_get_float32_auto(message, &index); // Acceleration Z
      }

      // Angular velocity
      if (mask & ((uint32_t)1 << 6)) {
          imu_data.gyro_x = buffer_get_float32_auto(message, &index); // Gyroscope X
      }
      if (mask & ((uint32_t)1 << 7)) {
          imu_data.gyro_y = buffer_get_float32_auto(message, &index); // Gyroscope Y
      }
      if (mask & ((uint32_t)1 << 8)) {
          imu_data.gyro_z = buffer_get_float32_auto(message, &index); // Gyroscope Z
      }

      // Magnetic field
      if (mask & ((uint32_t)1 << 9)) {
          imu_data.mag_x = buffer_get_float32_auto(message, &index); // Magnetometer X
      }
      if (mask & ((uint32_t)1 << 10)) {
          imu_data.mag_y = buffer_get_float32_auto(message, &index); // Magnetometer Y
      }
      if (mask & ((uint32_t)1 << 11)) {
          imu_data.mag_z = buffer_get_float32_auto(message, &index); // Magnetometer Z
      }

      // Quaternion components, Structure defined here: https://github.com/vedderb/bldc/blob/5dde8f3dfe6ba1f1e5a1ad6d62633e4c479ec531/imu/imu.c#L621 and https://github.com/vedderb/bldc/blob/5dde8f3dfe6ba1f1e5a1ad6d62633e4c479ec531/imu/imu.c#L351
      if (mask & ((uint32_t)1 << 12)) {
          imu_data.quat_w = buffer_get_float32_auto(message, &index); // Quaternion W
      }
      if (mask & ((uint32_t)1 << 13)) {
          imu_data.quat_x = buffer_get_float32_auto(message, &index); // Quaternion X
      }
      if (mask & ((uint32_t)1 << 14)) {
          imu_data.quat_y = buffer_get_float32_auto(message, &index); // Quaternion Y
      }
      if (mask & ((uint32_t)1 << 15)) {
          imu_data.quat_z = buffer_get_float32_auto(message, &index); // Quaternion Z
      }

      imu_data.id	= message[index++];	// 1 byte  - app_get_configuration()->controller_id	
      
      return true;
    }

		default:

			return false;
	}
}

uint8_t* VescPropeller::msg()
{
  return &message[0];
}

uint8_t* VescPropeller::getPayload()
{
  return &payload[0];
}

VescPropeller::dataPackage VescPropeller::getTheData()
{
  return data;
}

VescPropeller::FWversionPackage VescPropeller::getTheFW()
{
  return fw_version;
}

VescPropeller::imuPackage VescPropeller::getTheIMU()
{
  return imu_data;
}

bool VescPropeller::bufferReset()
{
  counter = 0;
  endMessage = 256;
  lenPayload = 0;

  return true;
}