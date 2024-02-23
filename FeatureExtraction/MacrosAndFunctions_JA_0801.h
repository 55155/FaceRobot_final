#pragma once

#include "LandmarkCoreIncludes.h"

#include <Face_utils.h>
#include <FaceAnalyser.h>
#include <GazeEstimation.h>
#include <RecorderOpenFace.h>
#include <RecorderOpenFaceParameters.h>
#include <SequenceCapture.h>
#include <Visualizer.h>
#include <VisualizationUtils.h>

#include "dynamixel_sdk.h"
#include <dense>
#include <string>
#include <fstream>
#include <iostream>
#include <thread>
#include <mutex>
#include <chrono>
#include <conio.h>
#include <windows.h>
#include<filesystem>

#include <mmsystem.h>
#pragma comment(lib,"winmm.lib")

using namespace Eigen;
using namespace std;
using std::cout;
using std::endl;
using std::chrono::duration_cast;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::chrono::system_clock;
using namespace std::this_thread;     // sleep_for, sleep_until
using namespace std::chrono_literals; // ns, us, ms, s, h, etc.

// ========================================== DXL SETTINGS

// Control table address
#define ADDR_PRO_TORQUE_ENABLE							64                 // Control table address is different in Dynamixel model
#define ADDR_PRO_GOAL_POSITION							116
#define ADDR_PRO_PRESENT_POSITION						132
#define ADDR_PRO_PROFILE_VELOCITY						112
#define ADDR_PRO_PRESENT_CURRENT						126

// Data Byte Length
#define LEN_PRO_GOAL_POSITION							4
#define LEN_PRO_PRESENT_POSITION						4
#define LEN_PRO_PROFILE_VELOCITY						4
#define LEN_PRO_PRESENT_CURRENT							2

// Protocol version
#define PROTOCOL_VERSION								2.0                 // See which protocol version is used in the Dynamixel

#define TORQUE_ENABLE									1					// Value for enabling the torque
#define TORQUE_DISABLE									0					// Value for disabling the torque

#define DXL_PROFILE_VELOCITY_HOMING						50
#define DXL_PROFILE_VELOCITY							0					// NO LIMIT
#define DXL_PROFILE_VELOCITY_CONFIGCHANGE				70

#define MAX_ROLL										0.7					// + left - right 
#define MAX_PITCH										0.6					// + down - up
#define MAX_YAW											1.0					// + right - left

// ========================================== ROBOT SETTINGS

// Default setting
#define DXL1_ID											1					// pitch
#define DXL2_ID											2					// right(from observer)
#define DXL3_ID											3					// left(from observer)
#define DXL4_ID											4					// yaw
#define DXL5_ID											5					// mouth

// starting positions of motors
#define default_PITCH									1200
#define default_ROLLR									6000
#define default_ROLLL									1000
#define default_YAW										500
#define default_MOUTH									1650

// ending positions of motors; position increase -> string tension down
#define END_PITCH										default_PITCH 
#define END_ROLLR										default_ROLLR 
#define END_ROLLL										default_ROLLL 
#define END_YAW											default_YAW
#define END_MOUTH										default_MOUTH

#define BAUDRATE										57600				// bps?
#define DEVICENAME										"COM5"				// Check which port is being used on your controller, ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

#define p_d												50					// pulley_diameter 40,50
#define ROBOT_HEIGHT									180					// small: 100, large: 180 (mm)
#define ROBOT_LAYERS									13					// number of layers
#define ROBOT_HOLE_RADIUS								50					// 머리 중간 빈 부분 반지름 / small: 25, large: 50 (mm) radius of string hole
#define ROBOT_YAW_GEAR_RATIO							1					// yaw gear ratio
#define ROBOT_MOUTH_TUNE								600					// mouth movement size
#define ROBOT_MOUTH_BACK_COMPENSATION					1.0				// 입에 대한 뒷쪽 보상(small: 1.2, large: 1.5)
#define ROBOT_MOUTH_PITCH_COMPENSATION					1.0					// 입에 대한 피치 보상

// ========================================== ETC SETTINGS
#define PI												3.141592
#define FLAG_SAVE_DXL_PRESENT_POSITION					false
#define FLAG_SAVE_DXL_PRESENT_CURRENT					false

#ifndef CONFIG_DIR
#define CONFIG_DIR "~"
#endif

#define INFO_STREAM( stream ) \
std::cout << stream << std::endl

#define WARN_STREAM( stream ) \
std::cout << "Warning: " << stream << std::endl

#define ERROR_STREAM( stream ) \
std::cout << "Error: " << stream << std::endl

static void printErrorAndAbort(const std::string& error) { std::cout << error << std::endl; }
#define FATAL_STREAM( stream ) \
printErrorAndAbort( std::string( "Fatal error: " ) + stream )


////////////////////////////////////////////////////////////////////////
// DXL FUNCTIONS
////////////////////////////////////////////////////////////////////////

void setDXLGoalPosition(int out[], int in1, int in2, int in3, int in4, int in5)
{
	out[0] = in1;
	out[1] = in2;
	out[2] = in3;
	out[3] = in4;
	out[4] = in5;
}

//void to_DXL_param(uint8_t output[], int input)
//{
//	// Allocate motor control value into byte array for GroupSyncWrite (ex. goal position)	
//	output[0] = DXL_LOBYTE(DXL_LOWORD(input));
//	output[1] = DXL_HIBYTE(DXL_LOWORD(input));
//	output[2] = DXL_LOBYTE(DXL_HIWORD(input));
//	output[3] = DXL_LOBYTE(DXL_HIWORD(input));
//}


bool to_SyncWrite_addParam(dynamixel::GroupSyncWrite& groupSyncWrite, int dxl_id, int goalpostion)
{
	// Allocate value into byte array
	uint8_t param[4];
	param[0] = DXL_LOBYTE(DXL_LOWORD(goalpostion));
	param[1] = DXL_HIBYTE(DXL_LOWORD(goalpostion));
	param[2] = DXL_LOBYTE(DXL_HIWORD(goalpostion));
	param[3] = DXL_HIBYTE(DXL_HIWORD(goalpostion));

	// Add Dynamixel value to the Syncwrite storage
	if (!groupSyncWrite.addParam(dxl_id, param)) { fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", dxl_id); return 0; }

	return true;

}

//bool moveDXLtoDesiredPosition(dynamixel::GroupSyncWrite& groupSyncWriteVelocity, dynamixel::GroupSyncWrite& groupSyncWritePosition, int velocity, int position[])
//{
//	uint8_t param_profile_velocity[4];
//	int DXL_ID[5] = { DXL1_ID, DXL2_ID, DXL3_ID, DXL4_ID, DXL5_ID };
//
//	to_DXL_param(param_profile_velocity, velocity);
//
//	// Add Dynamixel profile velocity value to the Syncwrite storage
//	for (int i = 0; i < 5; i++) {
//		if (!groupSyncWriteVelocity.addParam(DXL_ID[i], param_profile_velocity)) {
//			fprintf(stderr, "[ID:%03d] groupSyncWriteVelocity addparam failed", DXL_ID[i]);
//			return false;
//		}
//	}
//
//	// Syncwrite profile velocity
//	int dxl_comm_result = groupSyncWriteVelocity.txPacket();
//
//	//roll, pitch, yaw parameter goal position
//	uint8_t param_zero_position_pitch[4], param_zero_position_rollr[4], param_zero_position_rolll[4], param_zero_position_yaw[4], param_zero_position_mouth[4];
//
//	//Allocate goal position value into byte array for pitch, rollr, rolll, yaw, mouth
//	to_DXL_param(param_zero_position_pitch, position[0]);
//	to_DXL_param(param_zero_position_rollr, position[1]);
//	to_DXL_param(param_zero_position_rolll, position[2]);
//	to_DXL_param(param_zero_position_yaw, position[3]);
//	to_DXL_param(param_zero_position_mouth, position[4]);
//
//	// Add Dynamixel #1(pitch),#2(roll_right),#3(roll_left),#4(yaw),#5(mouth) goal position values to the Syncwrite parameter storage	
//	if (!groupSyncWritePosition.addParam(DXL_ID[0], param_zero_position_pitch)) { fprintf(stderr, "[ID:%03d] groupSyncWritePosition addparam failed", DXL_ID[0]); return 0; }
//	if (!groupSyncWritePosition.addParam(DXL_ID[1], param_zero_position_rollr)) { fprintf(stderr, "[ID:%03d] groupSyncWritePosition addparam failed", DXL_ID[1]); return 0; }
//	if (!groupSyncWritePosition.addParam(DXL_ID[2], param_zero_position_rolll)) { fprintf(stderr, "[ID:%03d] groupSyncWritePosition addparam failed", DXL_ID[2]); return 0; }
//	if (!groupSyncWritePosition.addParam(DXL_ID[3], param_zero_position_yaw)) { fprintf(stderr, "[ID:%03d] groupSyncWritePosition addparam failed", DXL_ID[3]); return 0; }
//	if (!groupSyncWritePosition.addParam(DXL_ID[4], param_zero_position_mouth)) { fprintf(stderr, "[ID:%03d] groupSyncWritePosition addparam failed", DXL_ID[4]); return 0; }
//
//	// Syncwrite goal position
//	dxl_comm_result = groupSyncWritePosition.txPacket();
//
//	// Clear syncwrite parameter storage
//	groupSyncWriteVelocity.clearParam();
//	groupSyncWritePosition.clearParam();
//
//	to_DXL_param(param_profile_velocity, DXL_PROFILE_VELOCITY); // unlimit motor velocity
//
//	// Add Dynamixel profile velocity value to the Syncwrite storage
//	for (int i = 0; i < 5; i++) {
//		if (!groupSyncWriteVelocity.addParam(DXL_ID[i], param_profile_velocity)) {
//			fprintf(stderr, "[ID:%03d] groupSyncWriteVelocity addparam failed", DXL_ID[i]);
//			return false;
//		}
//	}
//
//	// Syncwrite goal velocity
//	dxl_comm_result = groupSyncWriteVelocity.txPacket();
//
//	return true;
//}


//bool moveDXLtoDesiredPosition_NoVelLimit(dynamixel::PacketHandler* packetHandler, dynamixel::GroupSyncWrite& groupSyncWritePosition, int dxl_goal_position[])
//{
//	int DXL_ID[5] = { DXL1_ID, DXL2_ID, DXL3_ID, DXL4_ID, DXL5_ID };
//
//	//roll, pitch, yaw parameter goal position
//	uint8_t param_goal_position_pitch[4], param_goal_position_rollr[4], param_goal_position_rolll[4], param_goal_position_yaw[4], param_goal_position_mouth[4];
//
//	// Allocate goal position value into byte array for pitch, rollr, rolll, yaw, mouth
//	to_DXL_param(param_goal_position_pitch, dxl_goal_position[0]);
//	to_DXL_param(param_goal_position_rollr, dxl_goal_position[1]);
//	to_DXL_param(param_goal_position_rolll, dxl_goal_position[2]);
//	to_DXL_param(param_goal_position_yaw, dxl_goal_position[3]);
//	to_DXL_param(param_goal_position_mouth, dxl_goal_position[4]);
//
//	// Add Dynamixel #1~#5 goal position values to the Syncwrite parameter storage (pith,rollright,rollleft,yaw,mouth)
//	if (!groupSyncWritePosition.addParam(DXL_ID[0], param_goal_position_pitch)) { fprintf(stderr, "[ID:%03d] groupSyncWritePosition addparam failed", DXL_ID[0]); return 0; }
//	if (!groupSyncWritePosition.addParam(DXL_ID[1], param_goal_position_rollr)) { fprintf(stderr, "[ID:%03d] groupSyncWritePosition addparam failed", DXL_ID[1]); return 0; }
//	if (!groupSyncWritePosition.addParam(DXL_ID[2], param_goal_position_rolll)) { fprintf(stderr, "[ID:%03d] groupSyncWritePosition addparam failed", DXL_ID[2]); return 0; }
//	if (!groupSyncWritePosition.addParam(DXL_ID[3], param_goal_position_yaw)) { fprintf(stderr, "[ID:%03d] groupSyncWritePosition addparam failed", DXL_ID[3]); return 0; }
//	if (!groupSyncWritePosition.addParam(DXL_ID[4], param_goal_position_mouth)) { fprintf(stderr, "[ID:%03d] groupSyncWritePosition addparam failed", DXL_ID[4]); return 0; }
//
//	// Syncwrite goal position
//	int dxl_comm_result = groupSyncWritePosition.txPacket();
//	if (dxl_comm_result != COMM_SUCCESS) { printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result)); return false; }
//
//	return true;
//}


bool moveDXLtoDesiredPosition(dynamixel::PortHandler* portHandler, dynamixel::PacketHandler* packetHandler, int position[], int velocity)
{
	uint8_t param_profile_velocity[4];
	int DXL_ID[5] = { DXL1_ID, DXL2_ID, DXL3_ID, DXL4_ID, DXL5_ID };
	int dxl_comm_result;
		
	dynamixel::GroupSyncWrite groupSyncWritePosition(portHandler, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION);
	dynamixel::GroupSyncWrite groupSyncWriteVelocity(portHandler, packetHandler, ADDR_PRO_PROFILE_VELOCITY, LEN_PRO_PROFILE_VELOCITY);

	if (velocity != DXL_PROFILE_VELOCITY)
	{
		// Allocate profile velocity value into byte array and add Dynamixel profile velocity value to the Syncwrite storage
		for (int i = 0; i < 5; i++) { to_SyncWrite_addParam(groupSyncWriteVelocity, DXL_ID[i], velocity); }

		// Syncwrite profile velocity
		dxl_comm_result = groupSyncWriteVelocity.txPacket();
	}

	// Allocate goal position value into byte array and add Dynamixel goal position value to the Syncwrite storage
	for (int i = 0; i < 5; i++) { to_SyncWrite_addParam(groupSyncWritePosition, DXL_ID[i], position[i]); }

	// Syncwrite goal position
	dxl_comm_result = groupSyncWritePosition.txPacket();

	// Clear syncwrite parameter storage
	if (velocity != DXL_PROFILE_VELOCITY) groupSyncWriteVelocity.clearParam();
	groupSyncWritePosition.clearParam();

	if (velocity != DXL_PROFILE_VELOCITY)
	{
		// Allocate profile velocity value into byte array and add Dynamixel profile velocity value to the Syncwrite storage
		for (int i = 0; i < 5; i++) { to_SyncWrite_addParam(groupSyncWriteVelocity, DXL_ID[i], DXL_PROFILE_VELOCITY); }

		// Syncwrite goal velocity
		dxl_comm_result = groupSyncWriteVelocity.txPacket();
	}
		return true;
}


vector<int> RPY2DXL(double roll_f, double pitch_f, double yaw_f, double mouth_f, int mode)
{
	// CHANGE ROLL PITCH YAW MOUTH VALUES TO DXL POSITIONS

	//-------- change roll pitch yaw angle to string length L1 L2 L3
	//--------

	double yaw_degree = yaw_f * 180 / PI + 0.05;
	double roll = roll_f;
	double pitch = pitch_f;

	MatrixXd R_X_roll(3, 3), R_Y_pitch(3, 3);				// x축 rotation matrix, y축 rotation matrix
	R_X_roll << 1, 0, 0,
		0, cos(roll), -sin(roll),
		0, sin(roll), cos(roll);
	R_Y_pitch << cos(pitch), 0, sin(pitch),
		0, 1, 0,
		-sin(pitch), 0, cos(pitch);

	VectorXd zp(3), zn(3);									// 바닥 평면 수직벡터, 머리뚜껑 평면 수직벡터
	zp << 0, 0, 1;
	zn = R_Y_pitch * R_X_roll * zp;
	double n1 = zn(0), n2 = zn(1), n3 = zn(2);

	double theta = acos((zn.transpose() * zp).value());	// zp~zn 각도 (0 이상, 90 이하)
	double alpha = atan2(n2, n1);							// x축~zn바닥projection 각도

	VectorXd u_r(2);										// zn바닥projection 방향
	u_r << cos(alpha), sin(alpha);

	if (theta <= 0.00001)
		theta = 0.001;

	double r = ROBOT_HEIGHT / theta;						// 평면 중심 원격 회전 반경
	double r_x = r * cos(alpha);							// 원격 회전 중심점 x,y
	double r_y = r * sin(alpha);

	VectorXd R(2);
	R << r_x, r_y;

	VectorXd P1(2), P2(2), P3(2);
	P1 << ROBOT_HOLE_RADIUS * cos(0), ROBOT_HOLE_RADIUS* sin(0);			// 1번 구멍의 바닥위치
	P2 << ROBOT_HOLE_RADIUS * cos(2 * PI / 3), ROBOT_HOLE_RADIUS* sin(2 * PI / 3);	// 2번 구멍의 바닥위치
	P3 << ROBOT_HOLE_RADIUS * cos(4 * PI / 3), ROBOT_HOLE_RADIUS* sin(4 * PI / 3);	// 3번 구멍의 바닥위치

	VectorXd RP1(2), RP2(2), RP3(2);						// R -> Pi 벡터
	RP1 = P1 - R;
	RP2 = P2 - R;
	RP3 = P3 - R;

	double r1 = (-u_r.transpose() * RP1).value();			// Pi의 회전 반경
	double r2 = (-u_r.transpose() * RP2).value();
	double r3 = (-u_r.transpose() * RP3).value();

	double L1 = abs(r1) * theta; //앞쪽(DXL#1) // abs는 혹시 몰라서
	double L2 = abs(r2) * theta; //오른쪽(관찰자 시점//DXL#2)
	double L3 = abs(r3) * theta; //왼쪽(관찰자 시점//DXL#3)

	//--------- string length to DXL position
	//---------

	double dxl_goal_position_pitch_double, dxl_goal_position_rollr_double, dxl_goal_position_rolll_double, dxl_goal_position_yaw_double, dxl_goal_position_mouth_double;

	double pitch_diff = (ROBOT_HEIGHT - L1) * (4096 / (p_d * PI));
	if (pitch_diff < 0) // pitch down -> pitch tension too loose
		pitch_diff *= 0.5;

	dxl_goal_position_pitch_double = default_PITCH - pitch_diff;

	double delta_mouth = (float)mouth_f * ROBOT_MOUTH_TUNE; // 250
	dxl_goal_position_mouth_double = default_MOUTH - delta_mouth - pitch_diff / ROBOT_MOUTH_PITCH_COMPENSATION;

	dxl_goal_position_yaw_double = (-1) * static_cast<double> (yaw_degree) * ROBOT_YAW_GEAR_RATIO * 4096.0 / 360.0 + default_YAW; // 2

	if (mode == 0) // mirroring
	{
		dxl_goal_position_rollr_double = default_ROLLR - (ROBOT_HEIGHT - L2) * (4096 / (p_d * PI)) - (delta_mouth / ROBOT_MOUTH_BACK_COMPENSATION); // 1.5
		dxl_goal_position_rolll_double = default_ROLLL - (ROBOT_HEIGHT - L3) * (4096 / (p_d * PI)) - (delta_mouth / ROBOT_MOUTH_BACK_COMPENSATION);

		//// R, L이 너무 기울어졌을 때 입 보상 끄는거
		//if (dxl_goal_position_rollr_double < 200) dxl_goal_position_rollr_double += delta_mouth / ROBOT_MOUTH_BACK_COMPENSATION;
		//if (dxl_goal_position_rolll_double < 200) dxl_goal_position_rolll_double += delta_mouth / ROBOT_MOUTH_BACK_COMPENSATION;
	}
	else if (mode == 1) // cloning
	{
		// R L 변화량 change
		dxl_goal_position_rollr_double = default_ROLLR - (ROBOT_HEIGHT - L3) * (4096 / (p_d * PI)) - (delta_mouth / ROBOT_MOUTH_BACK_COMPENSATION);
		dxl_goal_position_rolll_double = default_ROLLL - (ROBOT_HEIGHT - L2) * (4096 / (p_d * PI)) - (delta_mouth / ROBOT_MOUTH_BACK_COMPENSATION);

		// yaw direction reverse
		dxl_goal_position_yaw_double *= -1;
	}
	else {
		cout << "RPY2DXL: CHECK MODE NUMBER" << endl;
		dxl_goal_position_rollr_double = default_ROLLR - (ROBOT_HEIGHT - L2) * (4096 / (p_d * PI)) - (delta_mouth / ROBOT_MOUTH_BACK_COMPENSATION); // 1.5
		dxl_goal_position_rolll_double = default_ROLLL - (ROBOT_HEIGHT - L3) * (4096 / (p_d * PI)) - (delta_mouth / ROBOT_MOUTH_BACK_COMPENSATION);
	}

	vector<int> DXL(5);
	DXL[0] = (int)dxl_goal_position_pitch_double;
	DXL[1] = (int)dxl_goal_position_rollr_double;
	DXL[2] = (int)dxl_goal_position_rolll_double;
	DXL[3] = (int)dxl_goal_position_yaw_double;
	DXL[4] = (int)dxl_goal_position_mouth_double;

	return DXL;
}

vector<int> RPY2DXL4singing(double roll_f, double pitch_f, double yaw_f, double delta_mouth)
{
	// CHANGE ROLL PITCH YAW MOUTH VALUES TO DXL POSITIONS

	//-------- change roll pitch yaw angle to string length L1 L2 L3
	//--------

	double yaw_degree = yaw_f * 180 / PI + 0.05;
	double roll = roll_f;
	double pitch = pitch_f;

	MatrixXd R_X_roll(3, 3), R_Y_pitch(3, 3);				// x축 rotation matrix, y축 rotation matrix
	R_X_roll << 1, 0, 0,
		0, cos(roll), -sin(roll),
		0, sin(roll), cos(roll);
	R_Y_pitch << cos(pitch), 0, sin(pitch),
		0, 1, 0,
		-sin(pitch), 0, cos(pitch);

	VectorXd zp(3), zn(3);									// 바닥 평면 수직벡터, 머리뚜껑 평면 수직벡터
	zp << 0, 0, 1;
	zn = R_Y_pitch * R_X_roll * zp;
	double n1 = zn(0), n2 = zn(1), n3 = zn(2);

	double theta = acos((zn.transpose() * zp).value());	// zp~zn 각도 (0 이상, 90 이하)
	double alpha = atan2(n2, n1);							// x축~zn바닥projection 각도

	VectorXd u_r(2);										// zn바닥projection 방향
	u_r << cos(alpha), sin(alpha);

	if (theta <= 0.00001)
		theta = 0.001;

	double r = ROBOT_HEIGHT / theta;						// 평면 중심 원격 회전 반경
	double r_x = r * cos(alpha);							// 원격 회전 중심점 x,y
	double r_y = r * sin(alpha);

	VectorXd R(2);
	R << r_x, r_y;

	VectorXd P1(2), P2(2), P3(2);
	P1 << ROBOT_HOLE_RADIUS * cos(0), ROBOT_HOLE_RADIUS* sin(0);			// 1번 구멍의 바닥위치
	P2 << ROBOT_HOLE_RADIUS * cos(2 * PI / 3), ROBOT_HOLE_RADIUS* sin(2 * PI / 3);	// 2번 구멍의 바닥위치
	P3 << ROBOT_HOLE_RADIUS * cos(4 * PI / 3), ROBOT_HOLE_RADIUS* sin(4 * PI / 3);	// 3번 구멍의 바닥위치

	VectorXd RP1(2), RP2(2), RP3(2);						// R -> Pi 벡터
	RP1 = P1 - R;
	RP2 = P2 - R;
	RP3 = P3 - R;

	double r1 = (-u_r.transpose() * RP1).value();			// Pi의 회전 반경
	double r2 = (-u_r.transpose() * RP2).value();
	double r3 = (-u_r.transpose() * RP3).value();

	double L1 = abs(r1) * theta; //앞쪽(DXL#1) // abs는 혹시 몰라서
	double L2 = abs(r2) * theta; //오른쪽(관찰자 시점//DXL#2)
	double L3 = abs(r3) * theta; //왼쪽(관찰자 시점//DXL#3)

	//--------- string length to DXL position
	//---------

	double dxl_goal_position_pitch_double, dxl_goal_position_rollr_double, dxl_goal_position_rolll_double, dxl_goal_position_yaw_double, dxl_goal_position_mouth_double;

	double pitch_diff = (ROBOT_HEIGHT - L1) * (4096 / (p_d * PI));
	if (pitch_diff < 0) // pitch down -> pitch tension too loose
		pitch_diff *= 0.5;

	//delta_mouth = (delta_mouth / 500) * 600;

	/*dxl_goal_position_pitch_double = default_PITCH - pitch_diff;*/
	dxl_goal_position_pitch_double = (default_PITCH - pitch_diff) / 1.25;
	dxl_goal_position_mouth_double = default_MOUTH - delta_mouth - pitch_diff / ROBOT_MOUTH_PITCH_COMPENSATION;
	dxl_goal_position_yaw_double = (-1) * static_cast<double> (yaw_degree) * ROBOT_YAW_GEAR_RATIO * 4096.0 / 360.0 + default_YAW; // 2
	dxl_goal_position_rollr_double = default_ROLLR - (ROBOT_HEIGHT - L2) * (4096 / (p_d * PI)) - (delta_mouth / ROBOT_MOUTH_BACK_COMPENSATION); // 1.5
	dxl_goal_position_rolll_double = default_ROLLL - (ROBOT_HEIGHT - L3) * (4096 / (p_d * PI)) - (delta_mouth / ROBOT_MOUTH_BACK_COMPENSATION);

	//// R, L이 너무 기울어졌을 때 입 보상 끄는거
	//if (dxl_goal_position_rollr_double < 200) dxl_goal_position_rollr_double += delta_mouth / ROBOT_MOUTH_BACK_COMPENSATION;
	//if (dxl_goal_position_rolll_double < 200) dxl_goal_position_rolll_double += delta_mouth / ROBOT_MOUTH_BACK_COMPENSATION;


	vector<int> DXL(5);
	DXL[0] = (int)dxl_goal_position_pitch_double;
	DXL[1] = (int)dxl_goal_position_rollr_double;
	DXL[2] = (int)dxl_goal_position_rolll_double;
	DXL[3] = (int)dxl_goal_position_yaw_double;
	DXL[4] = (int)dxl_goal_position_mouth_double;

	return DXL;
}

//vector<int> RPY2DXL(double roll_f, double pitch_f, double yaw_f, double mouth_f, int mode)
//{
//	// CHANGE ROLL PITCH YAW MOUTH VALUES TO DXL POSITIONS
//
//	//-------- change roll pitch yaw angle to string length L1 L2 L3
//	//--------
//
//	double yaw_degree = yaw_f * 180 / PI + 0.05;	// yaw
//	float gamma = (float)(roll_f);					// roll
//	float beta = (float)(pitch_f);					// pitch 
//
//	MatrixXd T_X_gamma(4, 4), T_Y_beta(4, 4), T(4, 4);
//	T_X_gamma << 1, 0, 0, 0, 0, cos(gamma), -sin(gamma), 0, 0, sin(gamma), cos(gamma), 0, 0, 0, 0, 1;	// x축을 기준으로 gamma 회전, roll
//	T_Y_beta << cos(beta), 0, sin(beta), 0, 0, 1, 0, 0, -sin(beta), 0, cos(beta), 0, 0, 0, 0, 1;		// y축을 기준으로 beta 회전, pitch
//	T = T_Y_beta * T_X_gamma;																			// 평행이동, roll, pitch 변환하는 행렬 
//	//T = T_X_gamma * T_Y_beta;
//
//	MatrixXd Z_p(3, 1), Z_p_ext(4, 1), Z_n(4, 1), vec(4, 1);
//	Z_p << 0, 0, 1;											//바닥면의 normal vector, z방향 유닛백터 
//	Z_p_ext << 0, 0, 1, 1;
//	vec << 0, 0, 0, 1;
//	Z_n = T.inverse() * (Z_p_ext - vec);					//변환면 roll-pitch 후에 normal vector 
//	double n1 = Z_n(0, 0), n2 = Z_n(1, 0), n3 = Z_n(2, 0);	//n1x + n2y + n3z + d = 0; 의 평면방정식을 가정 
//
//	//cout << "n1 = " << n1 << endl;
//	//cout << "n2 = " << n2 << endl;
//	//cout << "n3 = " << n3 << endl;
//
//	MatrixXd normalvec(1, 3);
//	normalvec << n1, n2, n3;
//	double theta = acos((normalvec * Z_p).value() / (normalvec.norm() * Z_p.norm()));
//	double alpha = atan2(n2, n1);
//
//	if (theta <= 0.00001) // theta 가 0이 되는 상황을 방지, 계산상으로 아주 작은 값을 넣어줌 
//		theta = 0.001;
//
//	double r = ROBOT_HEIGHT / theta;		// 문제, theta 가 0에 가까울수록, r이 커짐....
//	double r_xy = r * cos(theta);
//
//	double rcm_x = r * cos(alpha);			//  원격회전을 위한 중심
//	double rcm_y = r * sin(alpha);
//
//	//double rx = rcm_x - r_xy * cos(alpha);	// 이동한 윗면의 중심 
//	//double ry = rcm_y - r_xy * sin(alpha);
//	//double rz = r * sin(theta);
//
//	//// (2) roll-pitch 가 될 때, 3개 실구멍의 위치 구하기
//	//double a = rx;	// roll-pitch에 의해 이동한 위치 x
//	//double b = ry;	// roll-pitch에 의해 이동한 위치 y
//	//double c = rz;	// roll-pitch에 의해 이동한 위치 z
//
//	// 바닥면의 구멍과 방향백터 정의
//	MatrixXd P1(3, 1), P2(3, 1), P3(3, 1);
//	//, Q1(4, 1), Q2(4, 1), Q3(4, 1), E1(3, 1), E2(3, 1), E3(3, 1), abc(3, 1), P11(4, 1), P21(4, 1), P31(4, 1);
//	P1 << ROBOT_HOLE_RADIUS * cos(0), ROBOT_HOLE_RADIUS* sin(0), 0; // 1번 구멍의 바닥위치
//	P2 << ROBOT_HOLE_RADIUS * cos(2 * PI / 3), ROBOT_HOLE_RADIUS* sin(2 * PI / 3), 0; // 2번 구멍의 바닥위치
//	P3 << ROBOT_HOLE_RADIUS * cos(4 * PI / 3), ROBOT_HOLE_RADIUS* sin(4 * PI / 3), 0; // 3번 구멍의 바닥위치
//	//P11 << P1, 1; //padding
//	//P21 << P2, 1;
//	//P31 << P3, 1;
//	//abc << a, b, c;
//	//Q1 << (T.inverse() * P11);
//	//E1 << (Q1.block(0, 0, 3, 1) + abc);
//	//Q2 << (T.inverse() * P21);
//	//E2 << (Q2.block(0, 0, 3, 1) + abc);
//	//Q3 << (T.inverse() * P31);
//	//E3 << (Q3.block(0, 0, 3, 1) + abc);
//
//	//실의 길이
//	MatrixXd R(1, 2), u_rcm(1, 2), P1R(1, 2), P2R(1, 2), P3R(1, 2), P1R_1(1, 2), P2R_1(1, 2), P3R_1(1, 2);
//	R << rcm_x, rcm_y;
//
//	u_rcm << cos(alpha), sin(alpha);
//	P1R_1 << P1(0), P1(1);
//	P2R_1 << P2(0), P2(1);
//	P3R_1 << P3(0), P3(1);
//
//	P1R << (P1R_1 - R);
//	P2R << (P2R_1 - R);
//	P3R << (P3R_1 - R);
//
//	double r1 = (P1R * u_rcm.transpose()).value();
//	double r2 = (P2R * u_rcm.transpose()).value();
//	double r3 = (P3R * u_rcm.transpose()).value();
//
//	double L1 = (ROBOT_LAYERS * abs(r1) * (theta / ROBOT_LAYERS)); //앞쪽(DXL#1)
//	double L2 = (ROBOT_LAYERS * abs(r2) * (theta / ROBOT_LAYERS)); //오른쪽(관찰자 시점//DXL#2)
//	double L3 = (ROBOT_LAYERS * abs(r3) * (theta / ROBOT_LAYERS)); //왼쪽(관찰자 시점//DXL#3)
//
//	//cout << "L1 = " << L1 << endl;
//	//cout << "L2 = " << L2 << endl;
//	//cout << "L3 = " << L3 << endl;
//
//	//--------- string length to DXL position
//	//---------
//
//	double dxl_goal_position_pitch_double, dxl_goal_position_rollr_double, dxl_goal_position_rolll_double, dxl_goal_position_yaw_double, dxl_goal_position_mouth_double;
//
//	if (mode == 0) // mirroring
//	{
//		double pitch_diff = (L1 - ROBOT_HEIGHT) * (4096 / (p_d * PI));
//		//cout << "pitch_diff old = " << pitch_diff << endl;
//		// pitch up -> pitch tension too loose
//		if (pitch_diff < 0)
//			pitch_diff *= 0.5;
//
//		dxl_goal_position_pitch_double = default_PITCH - pitch_diff;
//
//		//dxl_goal_position_pitch_double = default_PITCH - (L1 - ROBOT_HEIGHT) * (4096 / (p_d * PI));
//
//		double delta_mouth = (float)mouth_f * ROBOT_MOUTH_TUNE; // 입 크기180
//
//		dxl_goal_position_rollr_double = default_ROLLR - (L2 - ROBOT_HEIGHT) * (4096 / (p_d * PI)) - (delta_mouth / ROBOT_MOUTH_BACK_COMPENSATION);
//		dxl_goal_position_rolll_double = default_ROLLL - (L3 - ROBOT_HEIGHT) * (4096 / (p_d * PI)) - (delta_mouth / ROBOT_MOUTH_BACK_COMPENSATION);
//
//		//// R, L이 너무 기울어졌을 때 입 보상 끄는거
//		//if (dxl_goal_position_rollr_double < 200)
//		//	dxl_goal_position_rollr_double += (delta_mouth / 1.5);
//		//if (dxl_goal_position_rolll_double < 200)
//		//	dxl_goal_position_rolll_double += (delta_mouth / 1.5);
//
//		dxl_goal_position_mouth_double = default_MOUTH - delta_mouth - (default_PITCH - dxl_goal_position_pitch_double) / ROBOT_MOUTH_PITCH_COMPENSATION;
//		dxl_goal_position_yaw_double = (-1) * static_cast<double> (yaw_degree) * ROBOT_YAW_GEAR_RATIO * 4096.0 / 360.0 + default_YAW;
//	}
//	else if (mode == 1) // cloning
//	{
//		double pitch_diff = (L1 - ROBOT_HEIGHT) * (4096 / (p_d * PI));
//
//		// pitch up -> pitch tension too loose
//		if (pitch_diff < 0)
//			pitch_diff *= 0.5;
//
//		dxl_goal_position_pitch_double = default_PITCH - pitch_diff;
//
//		//dxl_goal_position_pitch_double = default_PITCH - (L1 - ROBOT_HEIGHT) * (4096 / (p_d * PI));
//		double delta_mouth = (float)mouth_f * ROBOT_MOUTH_TUNE;
//
//		// R L change
//		dxl_goal_position_rolll_double = default_ROLLL - (L2 - ROBOT_HEIGHT) * (4096 / (p_d * PI)) - (delta_mouth / ROBOT_MOUTH_BACK_COMPENSATION);
//		dxl_goal_position_rollr_double = default_ROLLR- (L3 - ROBOT_HEIGHT) * (4096 / (p_d * PI)) - (delta_mouth / ROBOT_MOUTH_BACK_COMPENSATION);
//
//		dxl_goal_position_mouth_double = default_MOUTH - delta_mouth - (default_PITCH - dxl_goal_position_pitch_double) / ROBOT_MOUTH_PITCH_COMPENSATION;
//
//		// yaw_degree reverse
//		dxl_goal_position_yaw_double = static_cast<double> (yaw_degree) * ROBOT_YAW_GEAR_RATIO * 4096.0 / 360.0 + default_YAW;
//	}
//
//	vector<int> DXL(5);
//	DXL[0] = (int)dxl_goal_position_pitch_double;
//	DXL[1] = (int)dxl_goal_position_rollr_double;
//	DXL[2] = (int)dxl_goal_position_rolll_double;
//	DXL[3] = (int)dxl_goal_position_yaw_double;
//	DXL[4] = (int)dxl_goal_position_mouth_double;
//
//	return DXL;
//}

////////////////////////////////////////////////////////////////////////
// OPENFACE FUNCTIONS
////////////////////////////////////////////////////////////////////////

cv::Mat getInitialFaceLandmarks() {
	float data[136] = { 271.69623, 271.82077, 273.89084, 277.28607, 281.68842, 288.05957, 295.75983, 305.14264, 316.90778, 329.39532,
	340.55182, 350.41235, 357.4267, 361.77673, 364.37564, 365.84381, 366.27979, 277.07129, 282.77597, 290.06531, 297.46561, 304.18289,
	325.82047, 333.80997, 341.86813, 349.25903, 354.44379, 315.28378, 314.66376, 314.09268, 313.52332, 305.48163, 309.8074, 314.3866,
	319.35226, 323.89697, 285.38028, 290.07968, 296.92252, 302.23816, 296.5549, 289.72995, 328.73456, 334.20651, 341.23352, 346.50391,
	341.97198, 335.08917, 298.36707, 303.95691, 309.7356, 314.16751, 318.98099, 325.9827, 332.49347, 326.05673, 319.34296, 314.05801,
	309.16208, 303.41394, 301.01834, 309.86325, 314.4035, 319.32504, 329.43152, 319.12863, 314.1265, 309.52582, 208.08917, 221.4265,
	234.70872, 247.66771, 259.24414, 269.22748, 276.93927, 282.35025, 283.43497, 281.82846, 275.47977, 267.10376, 256.85941, 245.26056,
	232.8992, 219.93524, 207.02707, 187.50021, 181.7841, 179.66052, 180.42511, 183.31708, 182.63174, 179.20944, 178.41438, 180.49646,
	186.0107, 197.87358, 208.03188, 218.02512, 228.08081, 234.92499, 236.61705, 238.02171, 236.70427, 235.03503, 201.43588, 196.92465,
	197.25784, 202.27583, 204.07004, 204.09869, 201.90338, 196.7272, 196.52003, 200.67773, 203.37827, 203.59331, 253.88109, 249.6516,
	247.41498, 248.73662, 247.49908, 249.85741, 253.50937, 258.86298, 260.81927, 261.22537, 260.68729, 258.59988, 253.9399, 252.87222,
	253.2009, 252.78207, 253.90419, 253.84804, 254.30095, 253.81267 };
	return cv::Mat(1, 136, CV_32F, data);
}

vector<double> MeanFiltering_RPYM(double roll, double pitch, double yaw, double mouth)
{
	//cout << "roll = " << roll << ", pitch = " << pitch << ", yaw = " << yaw << ", mouth = " << mouth << endl;

	static int iter = 0;

	int fsize_r = 6;
	int fsize_p = 6;
	int fsize_y = 6;
	int fsize_m = 8;
	static vector<double> grid_r(fsize_r);
	static vector<double> grid_p(fsize_p);
	static vector<double> grid_y(fsize_y);
	static vector<double> grid_m(fsize_m);

	double roll_f = 0, pitch_f = 0, yaw_f = 0, mouth_f = 0;
	double ep = 0.000001;

	grid_r[iter % fsize_r] = roll;
	grid_p[iter % fsize_p] = pitch;
	grid_y[iter % fsize_y] = yaw;
	grid_m[iter % fsize_m] = mouth;

	// roll
	if (iter < fsize_r - 1)
		roll_f = roll;	
	else {
		for (int i = 0; i < grid_r.size(); i++)
			roll_f += grid_r[i] / fsize_r;
	}
	if (abs(roll_f) < ep)
		roll_f = 0;

	// pitch
	if (iter < fsize_p - 1)
		pitch_f = pitch;
	else {
		for (int i = 0; i < grid_p.size(); i++)
			pitch_f += grid_p[i] / fsize_p;
	}
	if (abs(pitch_f) < ep)
		pitch_f = 0;

	// yaw
	if (iter < fsize_y - 1)
		yaw_f = yaw;
	else {
		for (int i = 0; i < grid_y.size(); i++)
			yaw_f += grid_y[i] / fsize_y;
	}
	if (abs(yaw_f) < ep)
		yaw_f = 0;

	// mouth
	if (iter < fsize_m - 1)
		mouth_f = mouth;
	else {
		for (int i = 0; i < grid_m.size(); i++)
			mouth_f += grid_m[i] / fsize_m;
	}	
	if (abs(mouth_f) < ep)
		mouth_f = 0;

	iter++;
	if (iter > 10000)
		iter -= fsize_r * fsize_p * fsize_y * fsize_m;

	vector<double> filtered_output;
	filtered_output.push_back(roll_f);
	filtered_output.push_back(pitch_f);
	filtered_output.push_back(yaw_f);
	filtered_output.push_back(mouth_f);

	//cout << "roll_f = " << roll_f << ", pitch_f = " << pitch_f << ", yaw_f = " << yaw_f << ", mouth_f = " << mouth_f << endl;

	return filtered_output;
}

void MeanFiltering_Current(int16_t current_in[], int16_t current_out[], bool MovedToNextConfig)
{
	static int iter = 0;

	int motorNumbers = 5;
	int fsize = 5;

	static MatrixXd grid(motorNumbers, fsize);

	if (MovedToNextConfig)
		iter = 0;

	if (iter == 0)
		grid.setZero();

	for (int i = 0; i < motorNumbers; i++)
		grid(i, iter % fsize) = current_in[i];

	if (iter < fsize - 1) {
		for (int i = 0; i < motorNumbers; i++) {
			for (int t = 0; t <= iter; t++)
				current_out[i] += grid(i, t);			
			current_out[i] = (int16_t)round(current_out[i] / (iter + 1));
			//current_out[i] = current_in[i];
		}			
	}
	else {
		for (int i = 0; i < motorNumbers; i++) {
			current_out[i] = 0;
			for (int j = 0; j < fsize; j++)
				current_out[i] += grid(i, j); 
			current_out[i] = (int16_t)round(current_out[i] / fsize);
		}
	}

	iter++;
	if (iter > 2000000000)
		iter -= fsize * 10000000;
}

vector<double> KalmanFiltering(double roll, double pitch, double yaw, double mouth)
{
	static bool first = true;
	static MatrixXd A(6, 6), H(3, 6), Q(6, 6), R(3, 3), x(6, 1), P(6, 6), xp(6, 1), Pp(6, 6), K(6, 3), z(3, 1);				// roll pitch yaw
	static MatrixXd Am(2, 2), Hm(1, 2), Qm(2, 2), Rm(1, 1), xm(2, 1), Pm(2, 2), xpm(2, 1), Ppm(2, 2), Km(2, 1), zm(1, 1);	// mouth
	double dt = 0.032;

	if (first) {
		// A,H,Q,R: constant
		A << 1, dt, 0, 0, 0, 0,
			0, 1, 0, 0, 0, 0,
			0, 0, 1, dt, 0, 0,
			0, 0, 0, 1, 0, 0,
			0, 0, 0, 0, 1, dt,
			0, 0, 0, 0, 0, 1;
		H << 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0;
		Q = 5 * MatrixXd::Identity(6, 6);
		R = 100 * MatrixXd::Identity(3, 3);  //50

		Am << 1, dt, 0, 1;
		Hm << 1, 0;
		Qm = 5 * MatrixXd::Identity(2, 2);
		Rm = 300 * MatrixXd::Identity(1, 1);

		// x,P,xp,Pp,K,z: change
		x.setZero();
		P = 200 * MatrixXd::Identity(6, 6);

		xm.setZero();
		Pm = 200 * MatrixXd::Identity(2, 2);

		first = false;
	}

	//filtering pitch,yaw,roll values
	xp = A * x;
	Pp = A * P * A.transpose() + Q;
	K = Pp * H.transpose() * (H * Pp * H.transpose() + R).inverse();
	z << roll, pitch, yaw;
	x = xp + K * (z - H * xp);
	P = Pp - K * H * Pp;

	double roll_f = x(0, 0);	// -0.8 ~ +0.8
	double pitch_f = x(2, 0);	// -0.7 ~ +0.7
	double yaw_f = x(4, 0);	// -1.0 ~ +1.0

	// filtering mouth values
	xpm = Am * xm;
	Ppm = Am * Pm * Am.transpose() + Qm;
	Km = Ppm * Hm.transpose() * (Hm * Ppm * Hm.transpose() + Rm).inverse();
	zm << mouth;
	xm = xpm + Km * (zm - Hm * xpm);
	Pm = Ppm - Km * Hm * Ppm;

	double mouth_f = xm(0, 0);	// 0 ~ 2.5

	double ep = 0.000001;
	if (abs(roll_f) < ep)
		roll_f = 0;
	if (abs(pitch_f) < ep)
		pitch_f = 0;
	if (abs(yaw_f) < ep)
		yaw_f = 0;
	if (abs(mouth_f) < ep)
		mouth_f = 0;

	vector<double> filtered_rpym(4);
	filtered_rpym[0] = roll_f;
	filtered_rpym[1] = pitch_f;
	filtered_rpym[2] = yaw_f;
	filtered_rpym[3] = mouth_f;

	return filtered_rpym;
}

////////////////////////////////////////////////////////////////////////
// CSV FUNCTIONS
////////////////////////////////////////////////////////////////////////

std::vector<std::string> csv_read_row(std::istream& in, char delimiter)
{
	std::stringstream ss;
	bool inquotes = false;
	std::vector<std::string> row;//relying on RVO
	while (in.good())
	{
		char c = in.get();
		if (!inquotes && c == '"') //beginquotechar
		{
			inquotes = true;
		}
		else if (inquotes && c == '"') //quotechar
		{
			if (in.peek() == '"')//2 consecutive quotes resolve to 1
			{
				ss << (char)in.get();
			}
			else //endquotechar
			{
				inquotes = false;
			}
		}
		else if (!inquotes && c == delimiter) //end of field
		{
			row.push_back(ss.str());
			ss.str("");

		}
		else if (!inquotes && (c == '\r' || c == '\n'))
		{
			if (in.peek() == '\n') { in.get(); }
			row.push_back(ss.str());
			return row;
		}
		else
		{
			ss << c;
		}
	}
}

std::wstring s2ws(const std::string& s)
{
	int len;
	int slength = (int)s.length() + 1;
	len = MultiByteToWideChar(CP_ACP, 0, s.c_str(), slength, 0, 0);
	wchar_t* buf = new wchar_t[len];
	MultiByteToWideChar(CP_ACP, 0, s.c_str(), slength, buf, len);
	std::wstring r(buf);
	delete[] buf;
	return r;
}

bool letSing(dynamixel::PacketHandler* packetHandler, dynamixel::PortHandler* portHandler, string songname)
{
	dynamixel::GroupSyncWrite groupSyncWritePosition(portHandler, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION);

	INFO_STREAM("Start Singing~♬");
	int DXL_ID[5] = { DXL1_ID, DXL2_ID, DXL3_ID, DXL4_ID, DXL5_ID };
	//ofstream outData_timecheck;
	string songpath = "Sound/";
	int output = 1;
	double roll_f, pitch_f, yaw_f, delta_mouth; // filtered roll,pitch,yaw,mouth values
	int iter = 0;
	int dxl_goal_position[5] = { default_PITCH,default_ROLLR,default_ROLLL,default_YAW, default_MOUTH };
	bool firstRun = true;
	std::chrono::high_resolution_clock::time_point startTime;
	//long long begin, millisec_since_epoch1;
	//outData_timecheck.open("_CSV/Video/openface.csv", ios::app);
	std::ifstream headGesture("_CSV/Video/head2.csv"); //revision: museum, stay, momobic_2-1_
	std::ifstream mouthMotion(songpath + songname + "_mr없음-delta-big_less_test_10.csv");
	if (headGesture.fail())
		return(cout << "HeadGesture File not found" << endl) && 1;
	if (mouthMotion.fail())
		return(cout << "MouthMotion File not found" << endl) && 1;

	cv::namedWindow("img", cv::WND_PROP_FULLSCREEN);
	cv::setWindowProperty("img", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
	cv::Mat img = cv::imread("Image\\" + songname + ".jpg", cv::IMREAD_COLOR);
	if (img.empty())
	{
		std::cout << "Could not read the image: " << songname << std::endl;
		return 1;
	}
	cv::imshow("img", img);
	cv::waitKey(1);

	//----- HOMING MODE
	setDXLGoalPosition(dxl_goal_position, default_PITCH, default_ROLLR, default_ROLLL, default_YAW, default_MOUTH);
	if (!moveDXLtoDesiredPosition(portHandler, packetHandler, dxl_goal_position, DXL_PROFILE_VELOCITY_HOMING)) { return 0; }

	Sleep(3000);

	while (headGesture.good() && mouthMotion.good())
	{
		if (_kbhit()) {

			char c = _getch();
			FlushConsoleInputBuffer(GetStdHandle(STD_INPUT_HANDLE));

			if (c == 'q' || c == 'Q') {
				output = 0;
				PlaySound(TEXT("Sound\\empty.wav"), NULL, SND_FILENAME || SND_ASYNC || SND_NODEFAULT);
				break;
			}
			if (c == 's' || c == 'S') {
				output = 1;
				PlaySound(TEXT("Sound\\empty.wav"), NULL, SND_FILENAME || SND_ASYNC || SND_NODEFAULT);
				break;
			}
		}

		std::vector<std::string> headRow = csv_read_row(headGesture, ',');
		std::vector<std::string> mouthRow = csv_read_row(mouthMotion, ',');

		roll_f = stof(headRow[2]);
		pitch_f = stof(headRow[4]);
		yaw_f = stof(headRow[6]);
		delta_mouth = stof(mouthRow[0]);
		//delta_mouth = 0;



		vector<int> DXL = RPY2DXL4singing(roll_f, pitch_f, yaw_f, delta_mouth);
		setDXLGoalPosition(dxl_goal_position, DXL[0], DXL[1], DXL[2], DXL[3], DXL[4]);
		//setDXLGoalPosition(dxl_goal_position, DXL[0], DXL[1], DXL[2], DXL[3], default_MOUTH); //mouth 안 움직일 때 test

		// Allocate goal position value into byte array and add Dynamixel goal position value to the Syncwrite storage
		for (int i = 0; i < 5; i++)
		{
			to_SyncWrite_addParam(groupSyncWritePosition, DXL_ID[i], dxl_goal_position[i]);
		}




		if (firstRun) {
			std::wstring stemp = s2ws(songpath + songname + ".wav");
			LPCWSTR songname_result = stemp.c_str();
			PlaySound(songname_result, NULL, SND_FILENAME || SND_ASYNC || SND_NODEFAULT); //revision: museum, stay

			startTime = std::chrono::high_resolution_clock::now();
			//begin = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
			//millisec_since_epoch1 = begin;
			firstRun = false;
		}

		//auto tic = std::chrono::high_resolution_clock::now();

		auto awaketime = startTime + iter * 40ms;
		while (std::chrono::high_resolution_clock::now() < awaketime) { std::this_thread::yield(); }
		//std::this_thread::sleep_for(30ms);
		//Sleep(30);
		int dxl_comm_result = groupSyncWritePosition.txPacket();

		//auto toc = std::chrono::high_resolution_clock::now();
		//std::cout << "Elapsed time: " << duration_cast<milliseconds>(toc - tic).count() << std::endl;


		//millisec_since_epoch1 = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
		if (dxl_comm_result != COMM_SUCCESS) { printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result)); return false; };
		iter++;

		// Clear Syncwrite parameter storage
		groupSyncWritePosition.clearParam();
		//outData_timecheck << millisec_since_epoch1 - begin << endl;

	}
	return output;
}


////////////////////////////////////////////////////////////////////////
// KEYBOARD FUNCTIONS
////////////////////////////////////////////////////////////////////////

int KeyBoardStopMove(int calculatedDXL[], int afterKeyboard[])
{
	static int mode_webcamTrackingStop = 0; // 0: tracking, 1: stop			
	static bool changedFromStop2Move = false;
	static int dxl_goal_position_WHENSTOP[5] = { 0,0,0,0,0 };

	if (_kbhit()) {
		char c = _getch();
		if (c == 's' || c == 'S') {
			if (mode_webcamTrackingStop == 0) {
				mode_webcamTrackingStop = 1;

				for (int k = 0; k < 5; k++)
					dxl_goal_position_WHENSTOP[k] = calculatedDXL[k];
				//for (int k = 0; k < 4; k++)
				//	rpym_whenStopped[k] = rpym_present[k];

				INFO_STREAM("ROBOT MODE CHANGED TO STOP");
			}
			else {
				mode_webcamTrackingStop = 0;
				changedFromStop2Move = true;
				INFO_STREAM("ROBOT MODE CHANGED TO MOVE");
			}
		}
		if (c == 'q' || c == 'Q') {
			mode_webcamTrackingStop = 2;
		}
	}

	if (mode_webcamTrackingStop == 0) // webcam tracking
	{
		if (changedFromStop2Move) // change from stop to move
		{
			for (int i = 0; i < 5; i++)
				calculatedDXL[i] = (calculatedDXL[i] + dxl_goal_position_WHENSTOP[i]) / 2;
			changedFromStop2Move = false;
			for (int i = 0; i < 5; i++)
				afterKeyboard[i] = calculatedDXL[i];
		}
		else // originally tracking
		{
			for (int i = 0; i < 5; i++)
				afterKeyboard[i] = calculatedDXL[i];
		}
	}
	else if (mode_webcamTrackingStop == 1) // stop
	{
		for (int i = 0; i < 5; i++)
			afterKeyboard[i] = dxl_goal_position_WHENSTOP[i];
	}

	return mode_webcamTrackingStop; // 0: tracking, 1: stop	
}

vector<bool> KeyBoardControl(double& roll, double& pitch, double& yaw, double& mouth, bool isStop)
{
	static int speed_step = 4;
	double yaw_speed[10] = { 0.01, 0.015, 0.02, 0.025, 0.03, 0.035, 0.04, 0.045, 0.05, 0.055 };
	double pitch_speed[10] = { 0.01, 0.015, 0.02, 0.025, 0.03, 0.035, 0.04, 0.045, 0.05, 0.055 };
	double roll_speed[10] = { 0.01, 0.015, 0.02, 0.025, 0.03, 0.035, 0.04, 0.045, 0.05, 0.055 };
	double mouth_speed[10] = { 0.01, 0.0244, 0.0389, 0.0533, 0.0678, 0.0822, 0.0967, 0.1111, 0.1256, 0.14 };

	static double yaw_step = yaw_speed[speed_step];
	static double pitch_step = pitch_speed[speed_step];
	static double roll_step = roll_speed[speed_step];
	static double mouth_step = mouth_speed[speed_step];

	static int iter = 0;
	if (iter % 1000 == 0) {
		cout << "============================================================================" << endl;
		cout << "|                                                                          |" << endl;
		cout << "| Q: shutdown                                                              |" << endl;
		cout << "|                                          U: roll left    O: roll right   |" << endl;
		cout << "|                                                                          |" << endl;
		cout << "|                                                 I: raise head            |" << endl;
		cout << "|                                                                          |" << endl;
		cout << "|               H: go to home mode       J: rotate left    L: rotate right |" << endl;
		cout << "|                                                                          |" << endl;
		cout << "|                                                 K: lower head            |" << endl;
		cout << "|                                                                          |" << endl;
		cout << "| A: mouth close  /  S: mouth open                                         |" << endl;
		cout << "| Z: speed down   /  X: speed up                                           |" << endl;
		cout << "|                                                                          |" << endl;
		cout << "============================================================================" << endl << endl;
		cout << "Current speed: " << speed_step + 1 << " (1~10)" << endl;
	}
	iter++;

	bool hitKeyboard = false;
	bool goHome = false;
	bool exit = false;

	if (!isStop) {
		vector<bool> out(3);
		out[0] = false;
		out[1] = false;
		out[2] = false;
		return out;
	}

	if (_kbhit()) {

		//cout << "roll  = " << roll << endl;
		//cout << "pitch = " << pitch << endl;
		//cout << "yaw   = " << yaw << endl;

		hitKeyboard = true;

		char c = _getch();

		if (c == 'q' || c == 'Q')
			exit = true;

		if (c == 'h' || c == 'H') {
			roll = 0; pitch = 0; yaw = 0;
			goHome = true;
			cout << "Going to home position..." << endl;
		}

		// YAW
		if (c == 'j' || c == 'J') {
			if (yaw > -1.0) // left
				yaw -= yaw_step;
		}
		if (c == 'l' || c == 'L') {
			if (yaw < 1.0) // right
				yaw += yaw_step;
		}

		// PITCH
		if (c == 'i' || c == 'I') {
			if (pitch > -0.7) // up
				pitch -= pitch_step;
		}
		if (c == 'k' || c == 'K') {
			if (pitch < 0.7) // down
				pitch += pitch_step;
		}

		// ROLL
		if (c == 'u' || c == 'U') {
			if (roll < 0.8) // left
				roll += roll_step;
		}
		if (c == 'o' || c == 'O') {
			if (roll > -0.8) // right
				roll -= roll_step;
		}

		// MOUTH
		if (c == 's' || c == 'S') {
			if (mouth < 2.2) // up
				mouth += mouth_step;
		}
		if (c == 'a' || c == 'A') {
			if (mouth > 0.0) // down
				mouth -= mouth_step;
		}

		// SPEED
		if (c == 'x' || c == 'X') {
			if (speed_step < 9) {
				speed_step++;
				yaw_step = yaw_speed[speed_step];
				pitch_step = pitch_speed[speed_step];
				roll_step = roll_speed[speed_step];
				mouth_step = mouth_speed[speed_step];
				cout << "Current speed: " << speed_step + 1 << " (1~10)" << endl;
			}
		}
		if (c == 'z' || c == 'Z') {
			if (speed_step > 0) {
				speed_step--;
				yaw_step = yaw_speed[speed_step];
				pitch_step = pitch_speed[speed_step];
				roll_step = roll_speed[speed_step];
				mouth_step = mouth_speed[speed_step];
				cout << "Current speed: " << speed_step + 1 << " (1~10)" << endl;
			}
		}

		//// SPEED
		//if (c == 'x' || c == 'X') {
		//	cout << "SPEED UP" << endl;
		//	if (yaw_step <= 0.055)		yaw_step += 0.005;
		//	if (pitch_step <= 0.055)	pitch_step += 0.005;
		//	if (roll_step <= 0.055)		roll_step += 0.005;
		//	if (mouth_step <= 0.13)		mouth_step += 0.01;			
		//	cout << "yaw   : " << yaw_step / PI * 180 << "deg/click" << endl;
		//	cout << "pitch : " << pitch_step / PI * 180 << "deg/click" << endl;
		//	cout << "roll  : " << roll_step / PI * 180 << "deg/click" << endl;
		//	cout << "mouth : " << mouth_step / PI * 180 << "deg/click" << endl;
		//}
		//if (c == 'z' || c == 'Z') {
		//	cout << "SPEED DOWN" << endl;
		//	if (yaw_step >= 0.005)		yaw_step -= 0.005;
		//	if (pitch_step >= 0.005)	pitch_step -= 0.005;
		//	if (roll_step >= 0.005)		roll_step -= 0.005;
		//	if (mouth_step >= 0.01)		mouth_step -= 0.01;
		//	cout << "yaw   : " << yaw_step / PI * 180 << "deg/click" << endl;
		//	cout << "pitch : " << pitch_step / PI * 180 << "deg/click" << endl;
		//	cout << "roll  : " << roll_step / PI * 180 << "deg/click" << endl;
		//	cout << "mouth : " << mouth_step / PI * 180 << "deg/click" << endl;
		//}
	}

	vector<bool> output(3);
	output[0] = hitKeyboard;
	output[1] = goHome;
	output[2] = exit;

	return output;
}

bool PauseAndQuit()
{
	bool quit = false;
	bool pause = false;

	if (_kbhit()) {
		char c = _getch();
		if (c == 'q' || c == 'Q') {
			cout << "Shut off" << endl;
			quit = true;
		}
		if (c == 'p' || c == 'P') {
			pause = true;
			cout << "Paused" << endl;
		}
	}

	if (pause) {
		while (true) {
			if (_kbhit()) {
				char c = _getch();
				if (c == 'q' || c == 'Q') {
					cout << "Shut off" << endl;
					quit = true;
					break;
				}
				if (c == 'p' || c == 'P') {
					pause = false;
					cout << "Continue" << endl;
					break;
				}
			}
		}
	}

	return quit;
}


///////////
/////////// 221027 new letsing KM
///////////

void playwav(string audiofilepath)
{
	// https://m.blog.naver.com/noksek0615/221593719530
	std::wstring stemp = s2ws(audiofilepath);
	LPCWSTR songname_result = stemp.c_str();
	PlaySound(songname_result, NULL, SND_FILENAME || SND_ASYNC || SND_NODEFAULT);
}

void to_DXL_param_km(uint8_t output[], int input)
{
	// Allocate motor control value into byte array for GroupSyncWrite (ex. goal position)	
	output[0] = DXL_LOBYTE(DXL_LOWORD(input));
	output[1] = DXL_HIBYTE(DXL_LOWORD(input));
	output[2] = DXL_LOBYTE(DXL_HIWORD(input));
	output[3] = DXL_LOBYTE(DXL_HIWORD(input));
}

bool moveDXLtoDesiredPosition_km(dynamixel::GroupSyncWrite& groupSyncWriteVelocity, dynamixel::GroupSyncWrite& groupSyncWritePosition, int velocity, int position[])
{
	uint8_t param_profile_velocity[4];
	int DXL_ID[5] = { DXL1_ID, DXL2_ID, DXL3_ID, DXL4_ID, DXL5_ID };

	to_DXL_param_km(param_profile_velocity, velocity);
	// Add Dynamixel profile velocity value to the Syncwrite storage
	for (int i = 0; i < 5; i++) {
		if (!groupSyncWriteVelocity.addParam(DXL_ID[i], param_profile_velocity)) {
			fprintf(stderr, "[ID:%03d] groupSyncWriteVelocity addparam failed", DXL_ID[i]);
			return false;
		}
	}

	// Syncwrite profile velocity
	int dxl_comm_result = groupSyncWriteVelocity.txPacket();

	//roll, pitch, yaw parameter goal position
	uint8_t param_zero_position_pitch[4], param_zero_position_rollr[4], param_zero_position_rolll[4], param_zero_position_yaw[4], param_zero_position_mouth[4];

	//Allocate goal position value into byte array for pitch, rollr, rolll, yaw, mouth
	to_DXL_param_km(param_zero_position_pitch, position[0]);
	to_DXL_param_km(param_zero_position_rollr, position[1]);
	to_DXL_param_km(param_zero_position_rolll, position[2]);
	to_DXL_param_km(param_zero_position_yaw, position[3]);
	to_DXL_param_km(param_zero_position_mouth, position[4]);

	// Add Dynamixel #1(pitch),#2(roll_right),#3(roll_left),#4(yaw),#5(mouth) goal position values to the Syncwrite parameter storage	
	if (!groupSyncWritePosition.addParam(DXL_ID[0], param_zero_position_pitch)) { fprintf(stderr, "[ID:%03d] groupSyncWritePosition addparam failed", DXL_ID[0]); return 0; }
	if (!groupSyncWritePosition.addParam(DXL_ID[1], param_zero_position_rollr)) { fprintf(stderr, "[ID:%03d] groupSyncWritePosition addparam failed", DXL_ID[1]); return 0; }
	if (!groupSyncWritePosition.addParam(DXL_ID[2], param_zero_position_rolll)) { fprintf(stderr, "[ID:%03d] groupSyncWritePosition addparam failed", DXL_ID[2]); return 0; }
	if (!groupSyncWritePosition.addParam(DXL_ID[3], param_zero_position_yaw)) { fprintf(stderr, "[ID:%03d] groupSyncWritePosition addparam failed", DXL_ID[3]); return 0; }
	if (!groupSyncWritePosition.addParam(DXL_ID[4], param_zero_position_mouth)) { fprintf(stderr, "[ID:%03d] groupSyncWritePosition addparam failed", DXL_ID[4]); return 0; }

	// Syncwrite goal position
	dxl_comm_result = groupSyncWritePosition.txPacket();

	// Clear syncwrite parameter storage
	groupSyncWriteVelocity.clearParam();
	groupSyncWritePosition.clearParam();

	to_DXL_param_km(param_profile_velocity, DXL_PROFILE_VELOCITY); // unlimit motor velocity (주의)

	// Add Dynamixel profile velocity value to the Syncwrite storage
	for (int i = 0; i < 5; i++) {
		if (!groupSyncWriteVelocity.addParam(DXL_ID[i], param_profile_velocity)) {
			fprintf(stderr, "[ID:%03d] groupSyncWriteVelocity addparam failed", DXL_ID[i]);
			return false;
		}
	}

	// Syncwrite goal velocity
	dxl_comm_result = groupSyncWriteVelocity.txPacket();

	groupSyncWriteVelocity.clearParam();

	return true;
}

bool moveDXLtoDesiredPosition_NoVelLimit_km(dynamixel::PacketHandler* packetHandler, dynamixel::GroupSyncWrite& groupSyncWritePosition, int dxl_goal_position[])
{
	int DXL_ID[5] = { DXL1_ID, DXL2_ID, DXL3_ID, DXL4_ID, DXL5_ID };

	//roll, pitch, yaw parameter goal position
	uint8_t param_goal_position_pitch[4], param_goal_position_rollr[4], param_goal_position_rolll[4], param_goal_position_yaw[4], param_goal_position_mouth[4];

	// Allocate goal position value into byte array for pitch, rollr, rolll, yaw, mouth
	to_DXL_param_km(param_goal_position_pitch, dxl_goal_position[0]);
	to_DXL_param_km(param_goal_position_rollr, dxl_goal_position[1]);
	to_DXL_param_km(param_goal_position_rolll, dxl_goal_position[2]);
	to_DXL_param_km(param_goal_position_yaw, dxl_goal_position[3]);
	to_DXL_param_km(param_goal_position_mouth, dxl_goal_position[4]);

	// Add Dynamixel #1~#5 goal position values to the Syncwrite parameter storage (pith,rollright,rollleft,yaw,mouth)
	if (!groupSyncWritePosition.addParam(DXL_ID[0], param_goal_position_pitch)) { fprintf(stderr, "[ID:%03d] groupSyncWritePosition addparam failed", DXL_ID[0]); return 0; }
	if (!groupSyncWritePosition.addParam(DXL_ID[1], param_goal_position_rollr)) { fprintf(stderr, "[ID:%03d] groupSyncWritePosition addparam failed", DXL_ID[1]); return 0; }
	if (!groupSyncWritePosition.addParam(DXL_ID[2], param_goal_position_rolll)) { fprintf(stderr, "[ID:%03d] groupSyncWritePosition addparam failed", DXL_ID[2]); return 0; }
	if (!groupSyncWritePosition.addParam(DXL_ID[3], param_goal_position_yaw)) { fprintf(stderr, "[ID:%03d] groupSyncWritePosition addparam failed", DXL_ID[3]); return 0; }
	if (!groupSyncWritePosition.addParam(DXL_ID[4], param_goal_position_mouth)) { fprintf(stderr, "[ID:%03d] groupSyncWritePosition addparam failed", DXL_ID[4]); return 0; }

	// Syncwrite goal position
	int dxl_comm_result = groupSyncWritePosition.txPacket();
	if (dxl_comm_result != COMM_SUCCESS) { printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result)); return false; }

	groupSyncWritePosition.clearParam();

	return true;
}

bool letSing_km(dynamixel::PacketHandler* packetHandler, dynamixel::PortHandler* portHandler, string songname)
{
	// file input
	string audiopath = "Sound/" + songname + ".wav";
	string headmotionpath = "_CSV/Video/" + songname + ".csv";
	string mouthmotionpath = "Sound/" + songname + "-delta-big.csv";
	
	std::ifstream headGesture(headmotionpath);
	std::ifstream mouthMotion(mouthmotionpath);

	//string songpath = "Sound/";
	//std::ifstream headGesture("_CSV/Video/head2.csv"); //revision: museum, stay, momobic_2-1_
	//std::ifstream mouthMotion(songpath + songname + "_mr없음-delta-big_less_test_10.csv");

	if (headGesture.fail())
		return(cout << "HeadGesture File not found" << endl) && 1;
	if (mouthMotion.fail())
		return(cout << "MouthMotion File not found" << endl) && 1;

	// dxl
	dynamixel::GroupSyncWrite groupSyncWritePosition(portHandler, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION);		// POSITION
	dynamixel::GroupSyncWrite groupSyncWriteVelocity(portHandler, packetHandler, ADDR_PRO_PROFILE_VELOCITY, LEN_PRO_PROFILE_VELOCITY);	// VELOCITY

	INFO_STREAM("Start Singing");

	int output = 1;
	int DXL_ID[5] = { DXL1_ID, DXL2_ID, DXL3_ID, DXL4_ID, DXL5_ID };		
	double roll_f, pitch_f, yaw_f, delta_mouth; // filtered roll,pitch,yaw,mouth values	
	int dxl_goal_position[5] = { default_PITCH,default_ROLLR,default_ROLLL,default_YAW, default_MOUTH };

	// image
	cv::namedWindow("img", cv::WND_PROP_FULLSCREEN);
	cv::setWindowProperty("img", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
	cv::Mat img = cv::imread("Image\\" + songname + ".jpg", cv::IMREAD_COLOR);
	if (img.empty()) {
		std::cout << "Could not read the image: " << songname << std::endl;
		return 1;
	}
	cv::imshow("img", img);
	cv::waitKey(1);

	//----- HOMING MODE
	std::chrono::high_resolution_clock::time_point initiateTime = std::chrono::high_resolution_clock::now();
	auto awaketime = initiateTime + 3000ms;	
	setDXLGoalPosition(dxl_goal_position, default_PITCH, default_ROLLR, default_ROLLL, default_YAW, default_MOUTH);
	if (!moveDXLtoDesiredPosition_km(groupSyncWriteVelocity, groupSyncWritePosition, DXL_PROFILE_VELOCITY_HOMING, dxl_goal_position)) { return 0; }
	while (std::chrono::high_resolution_clock::now() < awaketime) { std::this_thread::yield(); }

	// loop
	std::chrono::high_resolution_clock::time_point startTime;
	int iter = 0;
	bool firstRun = true;
	while (headGesture.good() && mouthMotion.good())
	{
		if (_kbhit()) {

			char c = _getch();
			FlushConsoleInputBuffer(GetStdHandle(STD_INPUT_HANDLE));

			if (c == 'q' || c == 'Q') {
				output = 0;
				PlaySound(TEXT("Sound\\empty.wav"), NULL, SND_FILENAME || SND_ASYNC || SND_NODEFAULT);
				break;
			}
			if (c == 's' || c == 'S') {
				output = 1;
				PlaySound(TEXT("Sound\\empty.wav"), NULL, SND_FILENAME || SND_ASYNC || SND_NODEFAULT);
				break;
			}
		}

		std::vector<std::string> headRow = csv_read_row(headGesture, ',');
		std::vector<std::string> mouthRow = csv_read_row(mouthMotion, ',');

		roll_f = stof(headRow[0]);
		pitch_f = stof(headRow[1]);
		yaw_f = stof(headRow[2]);
		delta_mouth = stof(mouthRow[0]);
		//delta_mouth = 0;

		vector<int> DXL = RPY2DXL4singing(roll_f, pitch_f, yaw_f, delta_mouth);
		setDXLGoalPosition(dxl_goal_position, DXL[0], DXL[1], DXL[2], DXL[3], DXL[4]);

		if (firstRun) {
			moveDXLtoDesiredPosition_km(groupSyncWriteVelocity, groupSyncWritePosition, DXL_PROFILE_VELOCITY_HOMING, dxl_goal_position);
			std::this_thread::sleep_for(500ms);

			playwav(audiopath);

			startTime = std::chrono::high_resolution_clock::now();
			firstRun = false;
		}
		else {
			if (!moveDXLtoDesiredPosition_NoVelLimit_km(packetHandler, groupSyncWritePosition, dxl_goal_position)) break;
		}

		auto awaketime = startTime + iter * 40ms;
		while (std::chrono::high_resolution_clock::now() < awaketime) { std::this_thread::yield(); }

		iter++;
	}

	////----- HOMING MODE
	//setDXLGoalPosition(dxl_goal_position, default_PITCH, default_ROLLR, default_ROLLL, default_YAW, default_MOUTH);
	//if (!moveDXLtoDesiredPosition_km(groupSyncWriteVelocity, groupSyncWritePosition, DXL_PROFILE_VELOCITY_HOMING, dxl_goal_position)) { return 0; }
	//std::this_thread::sleep_for(500ms);

	return output;
}