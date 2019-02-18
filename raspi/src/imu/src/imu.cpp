
#include <ros/ros.h>
#include "imu/imu.h"

#include <iostream>
#include <cstdio>
#include <thread>
#ifdef _WIN32
#include "LpmsSensorI.h"
#include "LpmsSensorManagerI.h"
#endif
#ifdef __GNUC__
#include "lpsensor/LpmsSensorI.h"
#include "lpsensor/LpmsSensorManagerI.h"
#endif

ImuData d;
// Gets a LpmsSensorManager instance
LpmsSensorManagerI* manager = LpmsSensorManagerFactory();
// 
// DEVICE_LPMS_B        LPMS-B (Bluetooth)
// DEVICE_LPMS_U        LPMS-CU / LPMS-USBAL (USB)
// DEVICE_LPMS_C        LPMS-CU / LPMS-CANAL(CAN bus)
// DEVICE_LPMS_BLE      LPMS-BLE (Bluetooth low energy)
// DEVICE_LPMS_RS232    LPMS-UARTAL (RS-232)
// DEVICE_LPMS_B2       LPMS-B2
// DEVICE_LPMS_U2       LPMS-CU2/URS2/UTTL2/USBAL2 (USB)
// DEVICE_LPMS_C2       LPMS-CU2/CANAL2 (CAN)

// Connects to LPMS-B2 sensor with address 00:11:22:33:44:55 
//LpmsSensorI* lpms = manager->addSensor(DEVICE_LPMS_B2, "00:11:22:33:44:55");
// Connects to LPMS-CURS2 sensor try virtual com port 
LpmsSensorI* lpms = manager->addSensor(DEVICE_LPMS_RS232, "/dev/ttyUSB0");

uint32_t count = 0;

//define message
sensor_msgs::Imu imu_msg;
// 定义请求处理函数
bool handle_function(imu::imu::Request &req,
					imu::imu::Response &res)
{
	// read imu data
      d = lpms->getCurrentData();
      // write the imu data into imu_message
      imu_msg.header.seq = count;
      count++;
      imu_msg.header.stamp = ros::Time::now();

      imu_msg.orientation.w = d.q[0];
      imu_msg.orientation.x = d.q[1];
      imu_msg.orientation.y = d.q[2];
      imu_msg.orientation.z = d.q[3];

      imu_msg.angular_velocity.x = d.g[0];
      imu_msg.angular_velocity.y = d.g[1];
      imu_msg.angular_velocity.z = d.g[2];

      imu_msg.linear_acceleration.x = d.linAcc[0];
      imu_msg.linear_acceleration.y = d.linAcc[1];
      imu_msg.linear_acceleration.z = d.linAcc[2];

      res.imu_msg = imu_msg;

	return true;
}

int main(int argc, char **argv)
{
    // Gets a LpmsSensorManager instance
    //manager = LpmsSensorManagerFactory();
    // 
    // DEVICE_LPMS_B        LPMS-B (Bluetooth)
    // DEVICE_LPMS_U        LPMS-CU / LPMS-USBAL (USB)
    // DEVICE_LPMS_C        LPMS-CU / LPMS-CANAL(CAN bus)
    // DEVICE_LPMS_BLE      LPMS-BLE (Bluetooth low energy)
    // DEVICE_LPMS_RS232    LPMS-UARTAL (RS-232)
    // DEVICE_LPMS_B2       LPMS-B2
    // DEVICE_LPMS_U2       LPMS-CU2/URS2/UTTL2/USBAL2 (USB)
    // DEVICE_LPMS_C2       LPMS-CU2/CANAL2 (CAN)

    // Connects to LPMS-B2 sensor with address 00:11:22:33:44:55 
    //LpmsSensorI* lpms = manager->addSensor(DEVICE_LPMS_B2, "00:11:22:33:44:55");
    // Connects to LPMS-CURS2 sensor try virtual com port 
    //lpms = manager->addSensor(DEVICE_LPMS_RS232, "/dev/ttyUSB1");
    imu_msg.header.frame_id = "imu";
    uint32_t count = 0;
    //define node and service
	ros::init(argc, argv, "imu");
	
	ros::NodeHandle nh;
	ros::ServiceServer service = nh.advertiseService("imu", handle_function);
	
	ros::spin();

	return 0;
}