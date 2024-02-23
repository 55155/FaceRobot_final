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
#include <filesystem>
#include <vector>

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
#define ADDR_PRO_TORQUE_ENABLE          64                 // Control table address is different in Dynamixel model
#define ADDR_PRO_GOAL_POSITION          116
#define ADDR_PRO_PRESENT_POSITION       132
#define ADDR_PRO_PROFILE_VELOCITY		112
#define ADDR_PRO_PRESENT_CURRENT		126
#define ADDR_PRO_FEEDFORWARD_1ST_GAIN	90
#define ADDR_PRO_POSITION_P_GAIN		84

// Data Byte Length
#define LEN_PRO_GOAL_POSITION           4
#define LEN_PRO_PRESENT_POSITION        4
#define LEN_PRO_PROFILE_VELOCITY		4
#define LEN_PRO_PRESENT_CURRENT			2
#define LEN_PRO_FEEDFORWARD_1ST_GAIN	2
#define LEN_PRO_POSITION_P_GAIN		2

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

#define TORQUE_ENABLE                   1					// Value for enabling the torque
#define TORQUE_DISABLE                  0					// Value for disabling the torque

#define DXL_PROFILE_VELOCITY_HOMING			500
#define DXL_PROFILE_VELOCITY				40					// NO LIMIT
#define DXL_PROFILE_VELOCITY_CONFIGCHANGE	70

#define MAX_ROLL						0.7					// + left, - right, (rad)
#define MAX_PITCH						0.6					// + down, - up, (rad)
#define MAX_YAW							3.14				// + right, - left, (rad)

// ========================================== ROBOT SETTINGS

// Default setting
#define DXL1_ID                         1					// pitch
#define DXL2_ID                         2					// right(from observer)
#define DXL3_ID                         3					// left(from observer)
#define DXL4_ID                         4					// yaw
#define DXL5_ID                         5					// mouth

#define DXL_NUM							5

// starting positions of motors
#define default_PITCH					700 // 1780
#define default_ROLLR					1600 // 1325
#define default_ROLLL					1100 // 2630
#define default_YAW						3650 // 2030
#define default_MOUTH					3670 // 1300

// ending positions of motors; position value decrease -> string tension up
#define END_PITCH						default_PITCH + 500
#define END_ROLLR						default_ROLLR + 500
#define END_ROLLL						default_ROLLL + 500
#define END_YAW							default_YAW
#define END_MOUTH						default_MOUTH - 100

#define BAUDRATE                        57600				// bps?
#define DEVICENAME                      "COM3"				// Check which port is being used on your controller, ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
#define FREQUENCY						40ms

#define p_d								50					// pulley_diameter 40,50
#define ROBOT_HEIGHT					180					// small: 100, large: 180 (mm) -> 베이스부터 실이 연결된 레이어 까지의 높이!
//#define ROBOT_LAYERS					13					// number of layers
#define ROBOT_HOLE_RADIUS				50					// 머리 중간 빈 부분 반지름 / small: 25, large: 50 (mm) radius of string hole
#define ROBOT_YAW_GEAR_RATIO			2					// yaw gear ratio -> yaw 모터(4번)가 direct하게 머리를 회전시키면 1로 설정
#define ROBOT_MOUTH_TUNE				350					// 최대 mouth movement size in DXL dimension -> initial position에서 dynamixel wizard로 입 모터 움직여보면서 결정
//#define ROBOT_MOUTH_TUNE				1					// mouth movement size
#define ROBOT_MOUTH_BACK_COMPENSATION	1.5					// 입에 대한 뒷쪽 보상(small: 1.2, large: 1.5) -> TRO 논문 참조
#define ROBOT_MOUTH_PITCH_COMPENSATION	1.5					// 입에 대한 피치 보상
// Gaze control

// 대충 자로 잰 값 + 실제 값 보면서 calibration 
#define Poc_x 200   + 200
#define Poc_y -150	+ 20
#define Poc_z 20	
#define EYE_LAYER_HEIGHT 200

// ========================================== ETC SETTINGS
#define PI								3.141592
#define FLAG_SAVE_DXL_PRESENT_POSITION	false
#define FLAG_SAVE_DXL_PRESENT_CURRENT	true

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

// Define the return status of the main function
enum ReturnStatus {
	SUCCESS = 0,
	FAIL_OPEN_PORT = 1,
	FAIL_SET_BAUD_RATE = 2,
	FAIL_ENABLE_TORQUE = 3,
	FAIL_GO_TO_END_POSITION = 4,
	FAIL_DISABLE_TORQUE = 5,
};

enum RobotStatus {
	QUIT_PROGRAM = -1,// quit all		
	WAITING_FOR_USER_INPUT = 0, // wait for next command
	READY_TO_SPEAK = 1, // play one audio
	SING_A_SONG = 2,
	COMPLETE_SINGING = 3,
	SLEEP_MODE = 4
};

string ToString(RobotStatus robot_status) {
	switch (robot_status) {
	case QUIT_PROGRAM: return "QUIT_PROGRAM";
	case WAITING_FOR_USER_INPUT: return "WAITING_FOR_USER_INPUT";
	case SING_A_SONG: return "SING_A_SONG";
	case COMPLETE_SINGING: return "COMPLETE_SINGING";
	case SLEEP_MODE: return "SLEEP_MODE";
	default: return "Unkown";
	}
}

void setDXLGoalPosition(int out[], int in1, int in2, int in3, int in4, int in5)
{
	out[0] = in1;
	out[1] = in2;
	out[2] = in3;
	out[3] = in4;
	out[4] = in5;
}
void to_DXL_param_km(uint8_t output[], int input)
{
	// Allocate motor control value into byte array for GroupSyncWrite (ex. goal position)	
	output[0] = DXL_LOBYTE(DXL_LOWORD(input));
	output[1] = DXL_HIBYTE(DXL_LOWORD(input));
	output[2] = DXL_LOBYTE(DXL_HIWORD(input));
	output[3] = DXL_LOBYTE(DXL_HIWORD(input));
}

void to_DXL_Param(uint8_t output[], int goalpostion)
{
	// Allocate value into byte array
	output[0] = DXL_LOBYTE(DXL_LOWORD(goalpostion));
	output[1] = DXL_HIBYTE(DXL_LOWORD(goalpostion));
	output[2] = DXL_LOBYTE(DXL_HIWORD(goalpostion));
	output[3] = DXL_HIBYTE(DXL_HIWORD(goalpostion));

}
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
bool moveDXLtoDesiredPosition_NoVelLimit_smallrobot(dynamixel::PacketHandler* packetHandler, dynamixel::GroupSyncWrite& groupSyncWritePosition, int dxl_goal_position[])
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
bool moveDXLtoDesiredPosition_smallrobot(dynamixel::PortHandler* portHandler, dynamixel::PacketHandler* packetHandler, int position[], int velocity)
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

bool moveDXLtoDesiredPosition(dynamixel::PortHandler* portHandler, dynamixel::PacketHandler* packetHandler, int position[], int velocity = DXL_PROFILE_VELOCITY)
{
	// With the velocity option, the dxl profile velocity returns to zero(no limit) after the dxl action.

	uint8_t param_profile_velocity[4];
	uint8_t param_goal_position[5][4];

	int DXL_ID[5] = { DXL1_ID, DXL2_ID, DXL3_ID, DXL4_ID, DXL5_ID };
	int dxl_comm_result;

	dynamixel::GroupSyncWrite groupSyncWritePosition(portHandler, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION);
	dynamixel::GroupSyncWrite groupSyncWriteVelocity(portHandler, packetHandler, ADDR_PRO_PROFILE_VELOCITY, LEN_PRO_PROFILE_VELOCITY);

	if (velocity != DXL_PROFILE_VELOCITY)
	{
		// Allocate profile velocity value into byte array and add Dynamixel profile velocity value to the Syncwrite storage
		for (int i = 0; i < 5; i++) {
			to_DXL_Param(param_profile_velocity, velocity);
			if (!groupSyncWriteVelocity.addParam(DXL_ID[i], param_profile_velocity)) {
				fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL_ID[i]);
				return 0;
			}
		}
		// Syncwrite goal position
		dxl_comm_result = groupSyncWriteVelocity.txPacket();
		if (dxl_comm_result != COMM_SUCCESS)
		{
			printf("%s\nOccured from profile velocity setting\n", packetHandler->getTxRxResult(dxl_comm_result));
			return false;
		}
		// Clear syncwrite parameter storage
		groupSyncWriteVelocity.clearParam();
	}

	// Allocate goal position value into byte array and add Dynamixel goal position value to the Syncwrite storage
	for (int i = 0; i < 5; i++) {
		to_DXL_Param(param_goal_position[i], position[i]);
		if (!groupSyncWritePosition.addParam(DXL_ID[i], param_goal_position[i])) { fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL_ID[i]); return 0; }
	}
	// Syncwrite goal position
	dxl_comm_result = groupSyncWritePosition.txPacket();
	if (dxl_comm_result != COMM_SUCCESS)
	{
		printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
		return false;
	}
	// Clear syncwrite parameter storage
	groupSyncWritePosition.clearParam();

	if (velocity != DXL_PROFILE_VELOCITY)
	{
		// Allocate profile velocity value into byte array and add Dynamixel profile velocity value to the Syncwrite storage
		for (int i = 0; i < 5; i++) {
			to_DXL_Param(param_profile_velocity, DXL_PROFILE_VELOCITY);
			if (!groupSyncWriteVelocity.addParam(DXL_ID[i], param_profile_velocity)) {
				fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL_ID[i]);
				return 0;
			}
		}
		// Syncwrite goal position
		dxl_comm_result = groupSyncWriteVelocity.txPacket();
		if (dxl_comm_result != COMM_SUCCESS)
		{
			printf("%s\nOccured from profile velocity setting\n", packetHandler->getTxRxResult(dxl_comm_result));
			return false;
		}
		// Clear syncwrite parameter storage
		groupSyncWriteVelocity.clearParam();
	}
	return true;
}


vector<int> RPY2DXL(double roll_f, double pitch_f, double yaw_f, double mouth_f, int mode)
{
	// CHANGE ROLL PITCH YAW MOUTH VALUES TO DXL POSITIONS

	//-------- change roll pitch yaw angle to string length L1 L2 L3
	//--------

	double yaw_degree = yaw_f * 180 / PI;// +0.05;
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



	VectorXd u_r3(3), p13(3), p23(3);
	u_r3 << cos(alpha), sin(alpha), 0;
	p13 << ROBOT_HOLE_RADIUS * cos(0), ROBOT_HOLE_RADIUS* sin(0), 0;
	p23 << ROBOT_HOLE_RADIUS * cos(2 * PI / 3), ROBOT_HOLE_RADIUS* sin(2 * PI / 3), 0;

	VectorXd n0(3), n1p(3), n2p(3), n01p(3), n02p(3);
	n0 = r * (1 - cos(theta)) * u_r3 + r * sin(theta) * zp;
	n1p = p13 + r1 * (1 - cos(theta)) * u_r3 + r1 * sin(theta) * zp;
	n2p = p23 + r2 * (1 - cos(theta)) * u_r3 + r2 * sin(theta) * zp;

	n01p = n1p - n0;
	n02p = n2p - n0;

	MatrixXd Rypr(3, 3), Pypr(3, 3), Nypr(3, 3);
	Pypr << p13(0), p23(0), zp(0),
		p13(1), p23(1), zp(1),
		p13(2), p23(2), zp(2);
	Nypr << n01p(0), n02p(0), zn(0),
		n01p(1), n02p(1), zn(1),
		n01p(2), n02p(2), zn(2);

	Rypr = Nypr * Pypr.inverse();

	double pitch_real = asin(-Rypr(2, 0));
	double roll_real = asin(Rypr(2, 1) / cos(pitch_real));
	double yaw_real = asin(Rypr(1, 0) / cos(pitch_real));

	//cout << "roll error = " << (roll_real - roll) / PI * 180 << endl;
	//cout << "pitch error = " << (pitch_real - pitch) / PI * 180 << endl;
	//cout << "yaw error = " << (yaw_real - 0) / PI * 180 << endl;

	// yaw compensation
	double yaw_real_degree = yaw_real / PI * 180;

	//cout << "yaw_real_degree = " << yaw_real_degree << endl;

	if (abs(yaw_real_degree) < 60)
		yaw_degree -= yaw_real_degree;
	else
		cout << "Check kinematics yaw compensation value!" << endl;

	//--------- string length to DXL position
	//---------

	double dxl_goal_position_pitch_double, dxl_goal_position_rollr_double, dxl_goal_position_rolll_double, dxl_goal_position_yaw_double, dxl_goal_position_mouth_double;

	double pitch_diff = (ROBOT_HEIGHT - L1) * (4096 / (p_d * PI));

	//if (pitch < 0) // pitch down -> pitch tension too loose
	//	pitch_diff *= 0.1; // pitch tension up (220805) 0.3->0.1
	//else if (pitch < 0.1) // pitch tension up (220805)
	//	pitch_diff *= 1.2; // pitch tension up (220805)

	dxl_goal_position_pitch_double = default_PITCH - pitch_diff;

	double delta_mouth = (float)mouth_f * ROBOT_MOUTH_TUNE; // 250
	dxl_goal_position_mouth_double = default_MOUTH - delta_mouth - pitch_diff / ROBOT_MOUTH_PITCH_COMPENSATION;

	if (mode == 0) // mirroring
	{
		//dxl_goal_position_yaw_double = (-1) * static_cast<double> (yaw_degree) * ROBOT_YAW_GEAR_RATIO * 4096.0 / 360.0 + default_YAW; // 2
		dxl_goal_position_yaw_double = static_cast<double> (yaw_degree) * 1 * 4096.0 / 360.0 + default_YAW; // 2

		double rollR_diff = (ROBOT_HEIGHT - L2) * (4096 / (p_d * PI));
		double rollL_diff = (ROBOT_HEIGHT - L3) * (4096 / (p_d * PI));

		if (pitch < 0) {
			pitch_diff *= 2;
			//rollR_diff *= 1.5;
			//rollL_diff *= 1.5;
		}

		//cout << "rollR_diff = " << rollR_diff << endl;
		//cout << "rollL_diff = " << rollL_diff << endl;

		//// pitch tension up (220805)
		//if (pitch < 0.2) {
		//	if (rollR_diff > 0)
		//		rollR_diff *= 1.3;
		//	if (rollL_diff > 0)
		//		rollL_diff *= 1.3;
		//}

		dxl_goal_position_rollr_double = default_ROLLR - rollR_diff - (delta_mouth / ROBOT_MOUTH_BACK_COMPENSATION); // 1.5
		dxl_goal_position_rolll_double = default_ROLLL - rollL_diff - (delta_mouth / ROBOT_MOUTH_BACK_COMPENSATION);

		//// R, L이 너무 기울어졌을 때 입 보상 끄는거
		//if (dxl_goal_position_rollr_double < 200) dxl_goal_position_rollr_double += delta_mouth / ROBOT_MOUTH_BACK_COMPENSATION;
		//if (dxl_goal_position_rolll_double < 200) dxl_goal_position_rolll_double += delta_mouth / ROBOT_MOUTH_BACK_COMPENSATION;
	}
	else if (mode == 1) // cloning
	{
		dxl_goal_position_yaw_double = static_cast<double> (yaw_degree) * ROBOT_YAW_GEAR_RATIO * 4096.0 / 360.0 + default_YAW; // 2

		double rollR_diff = (ROBOT_HEIGHT - L3) * (4096 / (p_d * PI));
		double rollL_diff = (ROBOT_HEIGHT - L2) * (4096 / (p_d * PI));

		// pitch tension up (220805)
		if (pitch < 0.2) {
			if (rollR_diff > 0)
				rollR_diff *= 1.3;
			if (rollL_diff > 0)
				rollL_diff *= 1.3;
		}

		// R L 변화량 change
		dxl_goal_position_rollr_double = default_ROLLR - rollR_diff - (delta_mouth / ROBOT_MOUTH_BACK_COMPENSATION);
		dxl_goal_position_rolll_double = default_ROLLL - rollL_diff - (delta_mouth / ROBOT_MOUTH_BACK_COMPENSATION);
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

	dxl_goal_position_pitch_double = default_PITCH - pitch_diff;
	dxl_goal_position_mouth_double = default_MOUTH - (delta_mouth / 2) - pitch_diff / ROBOT_MOUTH_PITCH_COMPENSATION;
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
	// https://ieeexplore.ieee.org/abstract/document/8172386/
	// https://digital-library.theiet.org/content/journals/10.1049/iet-cta.2009.0032

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
		//R = 100 * MatrixXd::Identity(3, 3);  //50
		R << 120, 0, 0,
			0, 120, 0,
			0, 0, 120;

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



vector<double> KalmanFiltering_Gaze(double roll, double pitch, double yaw)
{
	// https://ieeexplore.ieee.org/abstract/document/8172386/
	// https://digital-library.theiet.org/content/journals/10.1049/iet-cta.2009.0032

	static bool first = true;
	static MatrixXd A(6, 6), H(3, 6), Q(6, 6), R(3, 3), x(6, 1), P(6, 6), xp(6, 1), Pp(6, 6), K(6, 3), z(3, 1);				// roll pitch yaw	
	double dt = 0.04;

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
		//R = 100 * MatrixXd::Identity(3, 3);  //50
		R << 150, 0, 0,
			0, 150, 0,
			0, 0, 150;

		// x,P,xp,Pp,K,z: change
		x.setZero();
		P = 200 * MatrixXd::Identity(6, 6);

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

	double ep = 0.000001;
	if (abs(roll_f) < ep)
		roll_f = 0;
	if (abs(pitch_f) < ep)
		pitch_f = 0;
	if (abs(yaw_f) < ep)
		yaw_f = 0;

	vector<double> filtered_rpym(3);
	filtered_rpym[0] = roll_f;
	filtered_rpym[1] = pitch_f;
	filtered_rpym[2] = yaw_f;

	return filtered_rpym;
}


VectorXd KalmanFiltering_pH(VectorXd pH)
{
	// https://ieeexplore.ieee.org/abstract/document/8172386/
	// https://digital-library.theiet.org/content/journals/10.1049/iet-cta.2009.0032

	static bool first = true;
	static MatrixXd A(6, 6), H(3, 6), Q(6, 6), R(3, 3), x(6, 1), P(6, 6), xp(6, 1), Pp(6, 6), K(6, 3), z(3, 1);				// roll pitch yaw	
	double dt = 0.04;

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
		R = 200 * MatrixXd::Identity(3, 3);  //50

		// x,P,xp,Pp,K,z: change
		x.setZero();
		P = 200 * MatrixXd::Identity(6, 6);

		first = false;
	}

	//filtering pitch,yaw,roll values
	xp = A * x;
	Pp = A * P * A.transpose() + Q;
	K = Pp * H.transpose() * (H * Pp * H.transpose() + R).inverse();
	z << pH(0), pH(1), pH(2);
	x = xp + K * (z - H * xp);
	P = Pp - K * H * Pp;

	double xf = x(0, 0);	// -0.8 ~ +0.8
	double yf = x(2, 0);	// -0.7 ~ +0.7
	double zf = x(4, 0);	// -1.0 ~ +1.0

	double ep = 0.000001;
	if (abs(xf) < ep)
		xf = 0;
	if (abs(yf) < ep)
		yf = 0;
	if (abs(zf) < ep)
		zf = 0;

	VectorXd filtered_xyz(3);
	filtered_xyz[0] = xf;
	filtered_xyz[1] = yf;
	filtered_xyz[2] = zf;

	return filtered_xyz;
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

int PauseQuitNext()
{
	int state = 0;

	bool quit = false;
	bool next = false;
	bool pause = false;

	if (_kbhit()) {
		char c = _getch();
		if (c == 'q' || c == 'Q') {
			cout << "Shut off" << endl;
			quit = true;
		}
		if (c == 'n' || c == 'N') {
			cout << "Next video" << endl;
			next = true;
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
				if (c == 'n' || c == 'N') {
					cout << "Next video" << endl;
					next = true;
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

	if (quit)
		state = 1;
	if (next)
		state = 2;

	return state;
}

vector<bool> KeyBoardControl(double& roll, double& pitch, double& yaw, double& mouth, bool isStop)
{
	static int speed_step = 4;
	double yaw_speed[10] = { 0.001, 0.005, 0.01, 0.025, 0.03, 0.035, 0.04, 0.045, 0.05, 0.055 };
	double pitch_speed[10] = { 0.001, 0.005, 0.01, 0.025, 0.03, 0.035, 0.04, 0.045, 0.05, 0.055 };
	double roll_speed[10] = { 0.001, 0.005, 0.01, 0.025, 0.03, 0.035, 0.04, 0.045, 0.05, 0.055 };
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

		cout << "roll (deg)  = " << roll / PI * 180 << endl;
		cout << "pitch (deg) = " << pitch / PI * 180 << endl;
		cout << "yaw (deg)   = " << yaw / PI * 180 << endl;

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

	//cout << "roll (deg)  = " << roll / PI * 180 << endl;
	//cout << "pitch (deg) = " << pitch / PI * 180 << endl;
	//cout << "yaw (deg)   = " << yaw / PI * 180 << endl;

	vector<bool> output(3);
	output[0] = hitKeyboard;
	output[1] = goHome;
	output[2] = exit;

	return output;
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

void write_robot_status(string data) {

	ofstream outfile("AIspeaker_JA\\robot_status.txt");
	outfile << data << endl;
	outfile.close();

}

string read_robot_status(void) {
	string data_sync = "init";
	ifstream infile("AIspeaker_JA\\robot_status.txt");

	if (infile.is_open()) {
		string line;
		getline(infile, data_sync);
		infile.close();
	}
	else {
		std::cerr << "robot_status 열기 실패!" << std::endl;
		return "FAIL_TO_READ_ROBOT_STATUS";
	}
	return data_sync;
}

////////////////////////////////////////////////////////////////////////
// SOUND FUNCTIONS
////////////////////////////////////////////////////////////////////////

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

void playwav(string audiofilepath)
{
	// https://m.blog.naver.com/noksek0615/221593719530
	std::wstring stemp = s2ws(audiofilepath);
	LPCWSTR songname_result = stemp.c_str();
	PlaySound(songname_result, NULL, SND_FILENAME || SND_ASYNC || SND_NODEFAULT);
}

////////////////////////////////////////////////////////////////////////
// HEAD MOTION GENERATE FUNCTIONS
////////////////////////////////////////////////////////////////////////

bool HeadMotion(dynamixel::PortHandler* portHandler, dynamixel::PacketHandler* packetHandler, string audiofilepath, string csvfilepath)
{
	dynamixel::GroupSyncWrite groupSyncWritePosition(portHandler, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION);		// POSITION
	dynamixel::GroupSyncWrite groupSyncWriteVelocity(portHandler, packetHandler, ADDR_PRO_PROFILE_VELOCITY, LEN_PRO_PROFILE_VELOCITY);	// VELOCITY

	//--- MOTOR PARAMETWRS
	int dxl_comm_result = COMM_TX_FAIL;					// Communication result
	int dxl_goal_position[5] = { 0,0,0,0,0 };
	int32_t dxl_present_position[5] = { 0,0,0,0,0 };
	int16_t dxl_present_current[5] = { 0,0,0,0,0 };
	uint8_t dxl_error = 0;								// Dynamixel error
	int DXL_ID[5] = { DXL1_ID, DXL2_ID, DXL3_ID, DXL4_ID, DXL5_ID };

	double roll_f, pitch_f, yaw_f, mouth_f; // filtered roll,pitch,yaw,mouth values

	std::ifstream in;
	if (csvfilepath != "NONE")
		in.open(csvfilepath);
	//else {

	//}

	if (in.fail())
		return(cout << "File not found" << endl) && 0;

	int iter = 0;
	bool firstRun = true;
	std::chrono::high_resolution_clock::time_point startTime;
	while (in.good())
	{
		if (PauseQuitNext() == 1)
			break;

		std::vector<std::string> row = csv_read_row(in, ',');

		roll_f = stof(row[0]);
		pitch_f = stof(row[1]) - 0.1;
		yaw_f = stof(row[2]);
		mouth_f = 0.0;

		int mode = 1; // 0: mirroring, 1: cloning
		vector<int> DXL = RPY2DXL(roll_f, pitch_f, yaw_f, mouth_f, mode);
		setDXLGoalPosition(dxl_goal_position, DXL[0], DXL[1], DXL[2], DXL[3], DXL[4]);

		if (firstRun) {
			moveDXLtoDesiredPosition(portHandler, packetHandler, dxl_goal_position, DXL_PROFILE_VELOCITY_HOMING);
			std::this_thread::sleep_for(1s);

			playwav(audiofilepath);
			startTime = std::chrono::high_resolution_clock::now();
			firstRun = false;
		}
		else {
			if (!moveDXLtoDesiredPosition(portHandler, packetHandler, dxl_goal_position)) break;
		}

		auto awaketime = startTime + iter * 40ms;
		while (std::chrono::high_resolution_clock::now() < awaketime) { std::this_thread::yield(); }

		iter++;
	}
}

int kinematics_test(dynamixel::PacketHandler* packetHandler, dynamixel::PortHandler* portHandler, string songname)
{
	string motionpath = "head_mouth_generation/kinematics/" + songname + ".csv";

	std::ifstream Motion(motionpath);

	if (Motion.fail())
		return(cout << "Motion File not found" << endl) && true;

	// dxl
	dynamixel::GroupSyncWrite groupSyncWritePosition(portHandler, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION);		// POSITION
	dynamixel::GroupSyncWrite groupSyncWriteVelocity(portHandler, packetHandler, ADDR_PRO_PROFILE_VELOCITY, LEN_PRO_PROFILE_VELOCITY);	// VELOCITY

	//INFO_STREAM("Start Talking");

	int output = 2;
	double roll_f, pitch_f, yaw_f, delta_mouth; // filtered roll,pitch,yaw,mouth values	
	int dxl_goal_position[5] = { default_PITCH,default_ROLLR,default_ROLLL,default_YAW, default_MOUTH };

	//----- HOMING MODE
	std::chrono::high_resolution_clock::time_point initiateTime = std::chrono::high_resolution_clock::now();
	auto awaketime = initiateTime + 500ms;
	setDXLGoalPosition(dxl_goal_position, default_PITCH, default_ROLLR, default_ROLLL, default_YAW, default_MOUTH);
	//if (!moveDXLtoDesiredPosition_km(groupSyncWriteVelocity, groupSyncWritePosition, DXL_PROFILE_VELOCITY_HOMING, dxl_goal_position)) { return 0; }
	if (!moveDXLtoDesiredPosition(portHandler, packetHandler, dxl_goal_position, DXL_PROFILE_VELOCITY_HOMING)) { return 0; }
	while (std::chrono::high_resolution_clock::now() < awaketime) { std::this_thread::yield(); }

	// loop
	std::chrono::high_resolution_clock::time_point startTime;
	int __iter = 0;
	bool firstRun = true;
	while (Motion.good())
	{
		if (_kbhit()) {

			char c = _getch();
			FlushConsoleInputBuffer(GetStdHandle(STD_INPUT_HANDLE));

			// quit
			if (c == 'q' || c == 'Q') {
				output = 0;
				break;
			}
			// skip
			if (c == 's' || c == 'S') {
				output = 1;
				break;
			}
			// pause
			if (c == 'p' || c == 'P') {
				output = 2;
				break;
			}
		}

		std::vector<std::string> MotionRow = csv_read_row(Motion, ',');

		roll_f = stof(MotionRow[1]);
		pitch_f = stof(MotionRow[2]);
		yaw_f = stof(MotionRow[3]);
		delta_mouth = stof(MotionRow[4]);

		vector<int> DXL = RPY2DXL(roll_f, pitch_f, yaw_f, delta_mouth, 0);
		setDXLGoalPosition(dxl_goal_position, DXL[0], DXL[1], DXL[2], DXL[3], DXL[4]);

		if (firstRun) {
			startTime = std::chrono::high_resolution_clock::now();
			firstRun = false;
		}
		else {
			if (!moveDXLtoDesiredPosition(portHandler, packetHandler, dxl_goal_position)) break;
		}

		auto awaketime = startTime + __iter * 40ms;
		while (std::chrono::high_resolution_clock::now() < awaketime) { std::this_thread::yield(); }

		__iter++;
	}

	if (output == 2) {
		//----- HOMING MODE
		initiateTime = std::chrono::high_resolution_clock::now();
		awaketime = initiateTime + 1000ms;
		setDXLGoalPosition(dxl_goal_position, default_PITCH, default_ROLLR, default_ROLLL, default_YAW, default_MOUTH);
		//if (!moveDXLtoDesiredPosition_km(groupSyncWriteVelocity, groupSyncWritePosition, DXL_PROFILE_VELOCITY_HOMING / 3, dxl_goal_position)) { return 0; }
		if (!moveDXLtoDesiredPosition(portHandler, packetHandler, dxl_goal_position, DXL_PROFILE_VELOCITY_HOMING)) { return 0; }
		while (std::chrono::high_resolution_clock::now() < awaketime) { std::this_thread::yield(); }
	}

	return output;
}

int waitfornextcommand(int& iter)
{
	int output = 2;

	while (true)
	{
		if (_kbhit()) {

			char c = _getch();
			FlushConsoleInputBuffer(GetStdHandle(STD_INPUT_HANDLE));

			// quit
			if (c == 'q' || c == 'Q') {
				output = 0;
				break;
			}
			// play next 
			if (c == 'p' || c == 'P') {
				output = 1;
				break;
			}
			// iteration down
			if (c == 'n' || c == 'N') {
				iter--;
				cout << "(next play iter = " << iter << ")" << endl;
				//break;
			}
			// iteration up
			if (c == 'm' || c == 'M') {
				iter++;
				cout << "(next play iter = " << iter << ")" << endl;
				//break;
			}

			if (c == '0') { iter = 0;  cout << "(next play iter = " << iter << ")" << endl; }
			if (c == '1') { iter = 10; cout << "(next play iter = " << iter << ")" << endl; }
			if (c == '2') { iter = 20; cout << "(next play iter = " << iter << ")" << endl; }
			if (c == '3') { iter = 30; cout << "(next play iter = " << iter << ")" << endl; }
			if (c == '4') { iter = 40; cout << "(next play iter = " << iter << ")" << endl; }
			if (c == '5') { iter = 50; cout << "(next play iter = " << iter << ")" << endl; }
			if (c == '6') { iter = 60; cout << "(next play iter = " << iter << ")" << endl; }
			if (c == '7') { iter = 70; cout << "(next play iter = " << iter << ")" << endl; }
			if (c == '8') { iter = 80; cout << "(next play iter = " << iter << ")" << endl; }
			if (c == '9') { iter = 90; cout << "(next play iter = " << iter << ")" << endl; }
		}
	}

	return output;
}

//////////////////////////// Gaze Motion

VectorXd get_fx(double h, double rh, double beta, VectorXd PG, double Hxy, double Hz) {
	VectorXd fx(2);
	fx(0) = h / PG(0) * (1 - cos(beta * PG(0))) + PG(1) * rh * cos(beta * PG(0)) - Hxy;
	fx(1) = h / PG(0) * sin(beta * PG(0)) - PG(1) * rh * sin(beta * PG(0)) - Hz;
	return fx;
}

MatrixXd get_Jx(double h, double rh, double beta, VectorXd PG) {
	MatrixXd J(2, 2);
	J(0, 0) = -h / (PG(0) * PG(0)) * (1 - cos(beta * PG(0))) + h / PG(0) * beta * sin(beta * PG(0)) - beta * PG(1) * rh * sin(beta * PG(0));
	J(0, 1) = rh * cos(beta * PG(0));
	J(1, 0) = -h / (PG(0) * PG(0)) * sin(beta * PG(0)) + h / PG(0) * beta * cos(beta * PG(0)) - beta * PG(1) * rh * cos(beta * PG(0));
	J(1, 1) = -rh * sin(beta * PG(0));
	return J;
}

MatrixXd get_Jx_inv(double h, double rh, double beta, VectorXd PG) {
	MatrixXd J(2, 2);
	J(0, 0) = -rh * sin(beta * PG(0));
	J(0, 1) = -(rh * cos(beta * PG(0)));
	J(1, 0) = -(-h / (PG(0) * PG(0)) * sin(beta * PG(0)) + h / PG(0) * beta * cos(beta * PG(0)) - beta * PG(1) * rh * cos(beta * PG(0)));
	J(1, 1) = -h / (PG(0) * PG(0)) * (1 - cos(beta * PG(0))) + h / PG(0) * beta * sin(beta * PG(0)) - beta * PG(1) * rh * sin(beta * PG(0));
	//double det = J(0, 0) * J(1, 1) - J(0, 1) * J(1, 0);
	double det = rh * (h / (PG(0) * PG(0)) * sin(beta * PG(0)) - h / PG(0) * beta + beta * PG(1) * rh);
	return J / det;
}

VectorXd get_AnalyticPG(double h, double rh, double beta, double Hxy, double Hz) {
	VectorXd PG_analyticGuess(2);
	PG_analyticGuess(0) = atan2(beta * h - Hz, Hxy);
	PG_analyticGuess(1) = sqrt(Hxy * Hxy + (beta * h - Hz) * (beta * h - Hz)) / rh;
	return PG_analyticGuess;
}

void rangeCheck(VectorXd& RPYG, double roll_max, double pitch_max, double yaw_max) {
	if (RPYG(0) > roll_max)
		RPYG(0) = roll_max;
	else if (RPYG(0) < -roll_max)
		RPYG(0) = -roll_max;
	if (RPYG(1) > pitch_max)
		RPYG(1) = pitch_max;
	else if (RPYG(1) < -pitch_max)
		RPYG(1) = -pitch_max;
	if (RPYG(2) > yaw_max)
		RPYG(2) = yaw_max;
	else if (RPYG(2) < -yaw_max)
		RPYG(2) = -yaw_max;
}

void rangeCheckOne(double& angle, double maxangle) {
	if (angle > maxangle)
		angle = maxangle;
	else if (angle < -maxangle)
		angle = -maxangle;
}

VectorXd findGaze_RPY_Gamma(VectorXd pH, bool reset = false)
{
	double h = ROBOT_HEIGHT, rh = ROBOT_HOLE_RADIUS, beta = 10.0 / 13.0;

	// solver parameters
	int maxIter = 100;
	double stopCrit = 0.000001;

	static VectorXd PitchGamma_Intial(2); // store initial guess
	static bool resetInitialGuess = true;

	double Hxy = sqrt(pH(0) * pH(0) + pH(1) * pH(1)), Hz = pH(2);

	if (Hxy < 200)
		cout << "-------------- WARNING: TOO CLOSE!" << endl;

	if (reset)
		resetInitialGuess = true;

	VectorXd PG(2); // pitch, gamma
	VectorXd fx(2);
	if (resetInitialGuess) {
		PG = get_AnalyticPG(h, rh, beta, Hxy, Hz); // inaccurate analytic guess
		resetInitialGuess = false;
	}
	else {
		PG = PitchGamma_Intial;
	}

	int iter = 0; // 보통 1~2회 안에 끝남		
	while (true) {

		if (abs(PG(0)) < 0.0001)
			PG(0) = 0.001;

		if (iter == 0) {
			fx = get_fx(h, rh, beta, PG, Hxy, Hz);
			if (fx.norm() < stopCrit)
				break;
		}

		// update
		//PG = PG - get_Jx(h, rh, beta, PG).inverse() * fx;
		PG = PG - get_Jx_inv(h, rh, beta, PG) * fx;
		fx = get_fx(h, rh, beta, PG, Hxy, Hz);

		if (fx.norm() < stopCrit)
			break;

		if (iter == maxIter) {
			cout << "-------------- WARNING: MAX ITERATION!" << endl;
			VectorXd PG_analyticGuess = get_AnalyticPG(h, rh, beta, Hxy, Hz);
			if (fx.norm() > get_fx(h, rh, beta, PG_analyticGuess, Hxy, Hz).norm())
				PG = PG_analyticGuess;
			break;
		}

		iter++;
	}

	// update initial guess
	PitchGamma_Intial = PG;

	VectorXd RPYG(4);
	RPYG(0) = 0;						// roll
	RPYG(1) = PG(0);					// pitch
	RPYG(2) = atan2(pH(1), pH(0));		// yaw
	RPYG(3) = PG(1);					// gamma

	rangeCheck(RPYG, MAX_ROLL, MAX_PITCH, MAX_YAW);

	return RPYG;
}

VectorXd getGazePosition(double pitch, double yaw, double gamma) {

	double h = ROBOT_HEIGHT, rh = ROBOT_HOLE_RADIUS, beta = 10.0 / 13.0;

	VectorXd p1(3);
	p1 << rh, 0, 0;
	VectorXd zp(3);
	zp << 0, 0, 1;

	MatrixXd Rz(3, 3);
	Rz << cos(yaw), -sin(yaw), 0,
		sin(yaw), cos(yaw), 0,
		0, 0, 1;

	VectorXd n0(3), n1p(3);
	if (abs(pitch) < 0.0001) {
		n0 = Rz * beta * h * zp;
		n1p = Rz * beta * h * zp + p1;
		return n0 + gamma * (n1p - n0);
	}

	double r = h / pitch;
	VectorXd tmp = VectorXd::Zero(3);

	tmp(0) = r * (1 - cos(pitch * beta));
	tmp(2) = r * sin(pitch * beta);
	n0 = Rz * tmp;

	tmp(0) = r - (r - rh) * cos(pitch * beta);
	tmp(2) = (r - rh) * sin(pitch * beta);
	n1p = Rz * tmp;

	return n0 + gamma * (n1p - n0);
}


#include <chrono>
using namespace chrono;

VectorXd GazeTest()
{
	static bool first = true;
	static VectorXd pH_xyz = VectorXd::Zero(3);
	if (first) {
		pH_xyz(0) = 200;
		first = false;
	}

	bool exit = false;
	bool resetInitialGuess = false;
	static VectorXd RollPitchYawGamma = VectorXd::Zero(4);
	if (_kbhit()) {

		char c = _getch();

		if (c == 'q' || c == 'Q')
			exit = true;

		// x
		if (c == 'n' || c == 'N')
			pH_xyz(0) -= 10;
		if (c == 'm' || c == 'M')
			pH_xyz(0) += 10;
		// y
		if (c == 'j' || c == 'J')
			pH_xyz(1) -= 20;
		if (c == 'k' || c == 'K')
			pH_xyz(1) += 20;
		// z
		if (c == 'u' || c == 'U')
			pH_xyz(2) -= 20;
		if (c == 'i' || c == 'I')
			pH_xyz(2) += 20;
		// reset
		if (c == 'r' || c == 'R')
			resetInitialGuess = true;
		// stopped
		if (c == 's' || c == 'S') {
			pH_xyz(1) = -pH_xyz(1);
			pH_xyz(2) = -pH_xyz(2);
		}

		chrono::system_clock::time_point StartTime = chrono::system_clock::now();

		RollPitchYawGamma = findGaze_RPY_Gamma(pH_xyz, resetInitialGuess);

		chrono::system_clock::time_point EndTime = chrono::system_clock::now();
		chrono::microseconds micro = chrono::duration_cast<chrono::microseconds>(EndTime - StartTime);

		cout << "(microseconds) : " << micro.count() << endl;

		if (resetInitialGuess)
			resetInitialGuess = false;

		VectorXd Gazepose = getGazePosition(RollPitchYawGamma(1), RollPitchYawGamma(2), RollPitchYawGamma(3));

		cout << "Hx (in)  = " << pH_xyz(0) << endl;
		cout << "Hx (out) = " << Gazepose(0) << endl;
		cout << "Hy (in)  = " << pH_xyz(1) << endl;
		cout << "Hy (out) = " << Gazepose(1) << endl;
		cout << "Hz (in)  = " << pH_xyz(2) << endl;
		cout << "Hz (out) = " << Gazepose(2) << endl;
		cout << "roll (deg)  = " << RollPitchYawGamma(0) / PI * 180 << endl;
		cout << "pitch (deg) = " << RollPitchYawGamma(1) / PI * 180 << endl;
		cout << "yaw (deg)   = " << RollPitchYawGamma(2) / PI * 180 << endl;
		cout << "gamma       = " << RollPitchYawGamma(3) << endl;
		cout << " " << endl;
	}

	VectorXd output(4);

	output(0) = RollPitchYawGamma(0);
	output(1) = RollPitchYawGamma(1);
	output(2) = RollPitchYawGamma(2);
	output(3) = exit;

	return output;
}

int getSlowingIdx_mod(deque<double> _ang) {
	if (_ang.size() < 7)
		cout << "getSlowingIdx_mod ERROR" << endl;

	double delta_41 = _ang[_ang.size() - 1] - _ang[_ang.size() - 5];
	double delta_42 = _ang[_ang.size() - 2] - _ang[_ang.size() - 6];
	double delta_43 = _ang[_ang.size() - 3] - _ang[_ang.size() - 7];

	int idx = 0;
	if (delta_41 > 0 && delta_42 > 0 && delta_43 > 0 && abs(delta_41) * 1.0 < abs(delta_42) && abs(delta_41) * 1.1 < abs(delta_43))
		idx = 1;
	if (delta_41 < 0 && delta_42 < 0 && delta_43 < 0 && abs(delta_41) * 1.0 < abs(delta_42) && abs(delta_41) * 1.1 < abs(delta_43))
		idx = 1;
	if (abs(delta_41) / 160 < 0.015 / 160) // rad/s
		idx = 0;
	return idx;
}

double dur2mult_mod(double angle, int dur, int rpy) {

	double mult = 1.0;
	if (rpy == 0)
		mult = max(1 - 0.1 * dur, 0.5);
	else if (rpy == 1 && dur > 1 && angle < 0) {
		mult = max(1.1 - 0.1 * dur, 0.5);
	}
	else if (rpy == 1 && angle > 0) {
		mult = max(1.0 - 0.1 * dur, 0.5);
	}
	//else if (rpy == 1 && dur > 1) {
	//	if (angle < 0)
	//		mult = max(1.1 - 0.1 * dur, 0.5);
	//	else
	//		mult = max(1.15 - 0.15 * dur, 0.5);
	//}
	else if (rpy == 2 && dur > 1)
		mult = max(1.15 - 0.15 * dur, 0.5);

	return mult;
}

void predictOneAngle(int rpy, deque<double>& _ang, double angle, int& iter, VectorXd& RPY_out, int FollowOriginal, int PredictSteps, double w1, double w2, int& dur)
{
	// rpy - 0: roll, 1: pitch, 2: yaw

	_ang.push_back(angle);
	if (iter < FollowOriginal - (PredictSteps - 1)) {
		RPY_out(0 + rpy) = angle;
		RPY_out(3 + rpy) = angle;
	}
	else {
		_ang.pop_front();
		int sz = _ang.size();

		if (sz < PredictSteps + 2)
			cout << "predictOneAngle ERROR" << endl;

		double delta = 0;
		bool flag = true;
		for (int j = PredictSteps - 1; j > 0; j--) {
			if (iter == FollowOriginal - j) {
				delta = w1 * (_ang[sz - 1] - _ang[sz - (PredictSteps + 1) + j]) + w2 * (_ang[sz - 2] - _ang[sz - (PredictSteps + 2) + j]);
				flag = false;
			}
		}
		if (flag)
			delta = w1 * (_ang[sz - 1] - _ang[sz - (PredictSteps + 1)]) + w2 * (_ang[sz - 2] - _ang[sz - (PredictSteps + 2)]);

		int idx = getSlowingIdx_mod(_ang);
		if (idx != 0)
			dur++;
		if (idx == 0 && dur > 0) {
			dur -= 1;
			if (dur < 0)
				dur = 0;
		}

		RPY_out(0 + rpy) = _ang[sz - 1] + delta;
		RPY_out(3 + rpy) = _ang[sz - 1] + dur2mult_mod(angle, dur, rpy) * delta;
		//RPY_out(0 + rpy) = _ang[sz - 1] + dur2mult_mod(angle, dur, rpy) * delta;
	}
	if (iter < FollowOriginal)
		iter++;

	rangeCheckOne(RPY_out(0), MAX_ROLL);
	rangeCheckOne(RPY_out(1), MAX_PITCH);
	// rangeCheckOne(RPY_out(2), MAX_YAW);
	rangeCheckOne(RPY_out(3), MAX_ROLL);
	rangeCheckOne(RPY_out(4), MAX_PITCH);
	// rangeCheckOne(RPY_out(5), MAX_YAW);
}

VectorXd predictRollPitchYaw_mod(double roll, double pitch, double yaw, bool reset, int r_PredictSteps = 3, int p_PredictSteps = 3, int y_PredictSteps = 2)
{
	// PredictSteps step 후의 값 예측 (최대 4)
	// FollowOriginal * 40 ms 동안은 predict 없이 원래 값으로 (최소 8) (이동안 새로 stack)
	// w1: 현재 grad값 weight, w2: 직전 grad값 weight

	static int r_iter = 0, p_iter = 0, y_iter = 0;
	static deque<double> _r, _p, _y;
	static int dur_r = 0, dur_p = 0, dur_y = 0;

	VectorXd RPY_out = VectorXd::Zero(6);
	//VectorXd RPY_out = VectorXd::Zero(3);

	int r_FollowOriginal = 8, p_FollowOriginal = 8, y_FollowOriginal = 8;
	r_FollowOriginal += (r_PredictSteps - 1); p_FollowOriginal += (p_PredictSteps - 1); y_FollowOriginal += (y_PredictSteps - 1);

	double w1 = 0.70, w2 = 1 - w1;

	if (reset) {
		r_iter = 0, p_iter = 0, y_iter = 0;
		_r.clear(); _p.clear(); _y.clear();
		dur_r = 0; dur_p = 0; dur_y = 0;
	}

	predictOneAngle(0, _r, roll, r_iter, RPY_out, r_FollowOriginal, r_PredictSteps, w1, w2, dur_r);
	predictOneAngle(1, _p, pitch, p_iter, RPY_out, p_FollowOriginal, p_PredictSteps, w1, w2, dur_p);
	predictOneAngle(2, _y, yaw, y_iter, RPY_out, y_FollowOriginal, y_PredictSteps, w1, w2, dur_y);

	return RPY_out;
}

VectorXd predictRollPitchYaw(double roll, double pitch, double yaw, bool reset, int PredictSteps_ = 4)
{
	VectorXd RPY_out(6);

	int PredictSteps = 3;							// 4 step 후의 값 예측 (최대 4)
	int FollowOriginal = 8; 						// 8*40 ms 동안은 predict 없이 원래 값으로 (최소 8) (이동안 새로 stack)
	FollowOriginal += (PredictSteps - 1);
	double w1 = 0.7, w2 = 1 - w1;					// w1: 현재 grad값 weight, w2: 직전 grad값 weight

	static int iter = 0;
	static deque<double> _r, _p, _y;
	static int dur_r = 0, dur_p = 0, dur_y = 0;

	if (reset) {
		iter = 0;
		_r.clear();
		_p.clear();
		_y.clear();
		dur_r = 0;
		dur_p = 0;
		dur_y = 0;
	}

	_r.push_back(roll);
	_p.push_back(pitch);
	_y.push_back(yaw);

	if (iter < FollowOriginal - (PredictSteps - 1)) {
		RPY_out(0) = roll;
		RPY_out(1) = pitch;
		RPY_out(2) = yaw;
		RPY_out(3) = roll;
		RPY_out(4) = pitch;
		RPY_out(5) = yaw;
	}
	else {
		_r.pop_front();
		_p.pop_front();
		_y.pop_front();

		int sz = _r.size();
		if (sz != _p.size() || sz != _y.size()) {
			cout << "PREDICT: DEQUE SIZE ERROR" << endl;
			return VectorXd::Zero(3);
		}

		double delta_r = 0, delta_p = 0, delta_y = 0;
		bool flag = true;
		for (int j = PredictSteps - 1; j > 0; j--) {
			if (iter == FollowOriginal - j) {
				delta_r = w1 * (_r[sz - 1] - _r[sz - (PredictSteps + 1) + j]) + w2 * (_r[sz - 2] - _r[sz - (PredictSteps + 2) + j]);
				delta_p = w1 * (_p[sz - 1] - _p[sz - (PredictSteps + 1) + j]) + w2 * (_p[sz - 2] - _p[sz - (PredictSteps + 2) + j]);
				delta_y = w1 * (_y[sz - 1] - _y[sz - (PredictSteps + 1) + j]) + w2 * (_y[sz - 2] - _y[sz - (PredictSteps + 2) + j]);
				flag = false;
			}
		}
		if (flag) {
			delta_r = w1 * (_r[sz - 1] - _r[sz - (PredictSteps + 1)]) + w2 * (_r[sz - 2] - _r[sz - (PredictSteps + 2)]);
			delta_p = w1 * (_p[sz - 1] - _p[sz - (PredictSteps + 1)]) + w2 * (_p[sz - 2] - _p[sz - (PredictSteps + 2)]);
			delta_y = w1 * (_y[sz - 1] - _y[sz - (PredictSteps + 1)]) + w2 * (_y[sz - 2] - _y[sz - (PredictSteps + 2)]);
		}

		int idx_r = getSlowingIdx_mod(_r);
		if (idx_r != 0)
			dur_r++;
		if (idx_r == 0 && dur_r > 0) {
			dur_r -= 2;
			if (dur_r < 0)
				dur_r = 0;
		}
		int idx_p = getSlowingIdx_mod(_p);
		if (idx_p != 0)
			dur_p++;
		if (idx_p == 0 && dur_p > 0) {
			dur_p -= 2;
			if (dur_p < 0)
				dur_p = 0;
		}
		int idx_y = getSlowingIdx_mod(_y);
		if (idx_y != 0)
			dur_y++;
		if (idx_y == 0 && dur_y > 0) {
			dur_y -= 2;
			if (dur_y < 0)
				dur_y = 0;
		}

		RPY_out(0) = _r[sz - 1] + delta_r;
		RPY_out(1) = _p[sz - 1] + delta_p;
		RPY_out(2) = _y[sz - 1] + delta_y;// *3 / 4;

		RPY_out(3) = _r[sz - 1] + dur2mult_mod(roll, dur_r, 0) * delta_r;
		RPY_out(4) = _p[sz - 1] + dur2mult_mod(pitch, dur_p, 1) * delta_p;
		RPY_out(5) = _y[sz - 1] + dur2mult_mod(yaw, dur_y, 2) * delta_y;// *3 / 4;				
	}

	if (iter < FollowOriginal)
		iter++;

	rangeCheck(RPY_out, MAX_ROLL, MAX_PITCH, PI / 2);

	return RPY_out;
}

RobotStatus play_one_audio(dynamixel::PortHandler* portHandler, dynamixel::PacketHandler* packetHandler, string path, string songname, bool image = false)
{

	string audiopath = path + "Audio\\" + songname + ".wav";
	string headmotionpath = path + "Headmotion\\" + songname + ".csv";
	string mouthmotionpath = path + "Mouthmotion\\" + songname + "-delta-big.csv";


	std::ifstream headGesture(headmotionpath);
	std::ifstream mouthMotion(mouthmotionpath);

	if (headGesture.fail()) {
		cout << "HeadGesture File not found" << endl;
		write_robot_status(ToString(SLEEP_MODE));
		return WAITING_FOR_USER_INPUT;
	}
	if (mouthMotion.fail()) {
		cout << "MouthMotion File not found" << endl;
		write_robot_status(ToString(SLEEP_MODE));
		return WAITING_FOR_USER_INPUT;
	}
	// dxl
	dynamixel::GroupSyncWrite groupSyncWritePosition(portHandler, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION);		// POSITION
	dynamixel::GroupSyncWrite groupSyncWriteVelocity(portHandler, packetHandler, ADDR_PRO_PROFILE_VELOCITY, LEN_PRO_PROFILE_VELOCITY);	// VELOCITY

	RobotStatus output_status = WAITING_FOR_USER_INPUT;
	double roll_f, pitch_f, yaw_f, delta_mouth; // filtered roll,pitch,yaw,mouth values	
	int dxl_goal_position[DXL_NUM] = { default_PITCH,default_ROLLR,default_ROLLL,default_YAW, default_MOUTH };

	if (image == true)
	{
		cv::namedWindow("img", cv::WND_PROP_FULLSCREEN);
		cv::setWindowProperty("img", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
		cv::Mat img = cv::imread("Image\\" + songname + ".jpg", cv::IMREAD_COLOR);
		if (img.empty()) {
			std::cout << "Could not read the image: " << songname << std::endl;
			return WAITING_FOR_USER_INPUT;
		}
		cv::imshow("img", img);
		cv::waitKey(1);
	}

	//----- HOMING MODE
	std::chrono::high_resolution_clock::time_point initiateTime = std::chrono::high_resolution_clock::now();
	auto awaketime = initiateTime + 500ms;
	setDXLGoalPosition(dxl_goal_position, default_PITCH, default_ROLLR, default_ROLLL, default_YAW, default_MOUTH);
	if (!moveDXLtoDesiredPosition(portHandler, packetHandler, dxl_goal_position, DXL_PROFILE_VELOCITY_HOMING)) { return QUIT_PROGRAM; }
	while (std::chrono::high_resolution_clock::now() < awaketime) { std::this_thread::yield(); }

	// loop
	std::chrono::high_resolution_clock::time_point startTime;
	int __iter = 0;
	bool firstRun = true;
	while (headGesture.good() && mouthMotion.good())
	{
		if (_kbhit()) {

			char c = _getch();
			FlushConsoleInputBuffer(GetStdHandle(STD_INPUT_HANDLE));

			// quit
			if (c == 'q' || c == 'Q') {
				playwav(path + "Audio\\empty.wav");
				output_status = QUIT_PROGRAM;
				write_robot_status(ToString(output_status));
				break;
			}
			// sleep
			if (c == 's' || c == 'S') {
				playwav(path + "Audio\\empty.wav");
				write_robot_status(ToString(SLEEP_MODE));
				output_status = WAITING_FOR_USER_INPUT;
				break;
			}
			// pause
			if (c == 'p' || c == 'P') {
				playwav(path + "Audio\\empty.wav");
				output_status = WAITING_FOR_USER_INPUT;
				break;
			}
		}

		std::vector<std::string> headRow = csv_read_row(headGesture, ',');
		std::vector<std::string> mouthRow = csv_read_row(mouthMotion, ',');

		roll_f = stof(headRow[0]);
		pitch_f = stof(headRow[1]);
		yaw_f = stof(headRow[2]);
		delta_mouth = stof(mouthRow[0]);


		vector<int> DXL = RPY2DXL4singing(roll_f, pitch_f, yaw_f, delta_mouth);
		setDXLGoalPosition(dxl_goal_position, DXL[0], DXL[1], DXL[2], DXL[3], DXL[4]);

		if (firstRun) {

			playwav(audiopath);

			startTime = std::chrono::high_resolution_clock::now();
			firstRun = false;
		}
		else {
			if (!moveDXLtoDesiredPosition(portHandler, packetHandler, dxl_goal_position)) break;
		}
		auto awaketime = startTime + __iter * 40ms;
		while (std::chrono::high_resolution_clock::now() < awaketime) { std::this_thread::yield(); }

		__iter++;
	}


	if (output_status == WAITING_FOR_USER_INPUT) {
		//----- HOMING MODE
		std::chrono::high_resolution_clock::time_point initiateTime_end = std::chrono::high_resolution_clock::now();
		auto awaketime_end = initiateTime_end + 500ms;
		setDXLGoalPosition(dxl_goal_position, default_PITCH, default_ROLLR, default_ROLLL, default_YAW, default_MOUTH);
		if (!moveDXLtoDesiredPosition(portHandler, packetHandler, dxl_goal_position, DXL_PROFILE_VELOCITY_HOMING / 2)) { return QUIT_PROGRAM; }
		while (std::chrono::high_resolution_clock::now() < awaketime_end) { std::this_thread::yield(); }
	}

	return output_status;
}


RobotStatus wait4processing(dynamixel::PortHandler* portHandler, dynamixel::PacketHandler* packetHandler)
{
	// file input	
	string HeadMotionPath = "AIspeaker_JA/Headmotion/empty_10min.csv";

	std::ifstream headGesture(HeadMotionPath);

	if (headGesture.fail()) {
		cout << "Empty HeadGesture File not found" << endl;
		return QUIT_PROGRAM;
	}


	// dxl
	dynamixel::GroupSyncWrite groupSyncWritePosition(portHandler, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION);		// POSITION
	dynamixel::GroupSyncWrite groupSyncWriteVelocity(portHandler, packetHandler, ADDR_PRO_PROFILE_VELOCITY, LEN_PRO_PROFILE_VELOCITY);	// VELOCITY

	RobotStatus output = WAITING_FOR_USER_INPUT;
	double roll_f, pitch_f, yaw_f, delta_mouth; // filtered roll,pitch,yaw,mouth values	
	int dxl_goal_position[DXL_NUM] = { default_PITCH,default_ROLLR,default_ROLLL,default_YAW, default_MOUTH };



	static int veryfirst = 0;
	if (veryfirst == 0) {
		//----- HOMING MODE
		std::chrono::high_resolution_clock::time_point initiateTime_veryfirst = std::chrono::high_resolution_clock::now();
		auto awaketime_veryfirst = initiateTime_veryfirst + 2000ms;
		setDXLGoalPosition(dxl_goal_position, default_PITCH, default_ROLLR, default_ROLLL, default_YAW, default_MOUTH);
		if (!moveDXLtoDesiredPosition(portHandler, packetHandler, dxl_goal_position, DXL_PROFILE_VELOCITY_HOMING / 2)) { return QUIT_PROGRAM; }
		while (std::chrono::high_resolution_clock::now() < awaketime_veryfirst) { std::this_thread::yield(); }
		veryfirst = 1;
	}


	// loop
	std::chrono::high_resolution_clock::time_point startTime;
	int __iter = 0;
	bool firstRun = true;
	string robot_status = "WAITING_FOR_USER_INPUT";
	while (headGesture.good())
	{
		robot_status = read_robot_status();

		if (_kbhit()) {
			char c = _getch();
			FlushConsoleInputBuffer(GetStdHandle(STD_INPUT_HANDLE));

			// quit
			if (c == 'q' || c == 'Q') {
				output = QUIT_PROGRAM;
				write_robot_status(ToString(output));
				break;
			}
			else if (c == 's' || c == 'S') { //sleep
				output = WAITING_FOR_USER_INPUT;
				write_robot_status(ToString(SLEEP_MODE));
				break;
			}
			else if (c == 'a' || c == 'A') {
				output = WAITING_FOR_USER_INPUT; //awake
				write_robot_status("AWAKE");
				break;
			}
		}
		else if (robot_status == "PYTHON_ERROR") {
			output = QUIT_PROGRAM;
			cout << "PYTHON_ERROR" << endl;
			break;
		}
		else if (robot_status == "READY_TO_SPEAK") {
			output = READY_TO_SPEAK;
			break;
		}
		else if (robot_status == "SING_A_SONG") {
			output = SING_A_SONG;
			break;
		}
		else if (robot_status == "QUIT_PROGRAM") {
			output = READY_TO_SPEAK;
			break;
		}


		std::vector<std::string> headRow = csv_read_row(headGesture, ',');

		roll_f = stof(headRow[0]);
		pitch_f = stof(headRow[1]);
		yaw_f = stof(headRow[2]);
		delta_mouth = 0;

		double ratiooo = 1.3;
		vector<int> DXL = RPY2DXL4singing(roll_f * ratiooo, pitch_f * ratiooo, yaw_f * ratiooo, delta_mouth);
		setDXLGoalPosition(dxl_goal_position, DXL[0], DXL[1], DXL[2], DXL[3], DXL[4]);

		if (!moveDXLtoDesiredPosition(portHandler, packetHandler, dxl_goal_position)) break;


		if (firstRun) {
			startTime = std::chrono::high_resolution_clock::now();
			firstRun = false;
		}

		auto awaketime = startTime + __iter * 40ms;
		while (std::chrono::high_resolution_clock::now() < awaketime) { std::this_thread::yield(); }

		__iter++;
	}

	if (output == WAITING_FOR_USER_INPUT) {
		//----- HOMING MODE
		std::chrono::high_resolution_clock::time_point initiateTime_end = std::chrono::high_resolution_clock::now();
		auto awaketime_end = initiateTime_end + 1500ms;
		setDXLGoalPosition(dxl_goal_position, default_PITCH, default_ROLLR, default_ROLLL, default_YAW, default_MOUTH);
		if (!moveDXLtoDesiredPosition(portHandler, packetHandler, dxl_goal_position, DXL_PROFILE_VELOCITY_HOMING / 2)) { return QUIT_PROGRAM; }
		while (std::chrono::high_resolution_clock::now() < awaketime_end) { std::this_thread::yield(); }
	}
	return output;
}

int MC_ment_km(dynamixel::PortHandler* portHandler, dynamixel::PacketHandler* packetHandler, string songname, bool image = false, bool AIspeaker = false)
{

	string audiopath = songname + ".wav";
	string headmotionpath = songname + ".csv";
	string mouthmotionpath = songname + "-delta-big.csv";


	std::ifstream headGesture(headmotionpath);
	std::ifstream mouthMotion(mouthmotionpath);

	if (headGesture.fail())
		return(cout << "HeadGesture File not found" << endl) && true;
	if (mouthMotion.fail())
		return(cout << "MouthMotion File not found" << endl) && true;

	// dxl
	dynamixel::GroupSyncWrite groupSyncWritePosition(portHandler, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION);		// POSITION
	dynamixel::GroupSyncWrite groupSyncWriteVelocity(portHandler, packetHandler, ADDR_PRO_PROFILE_VELOCITY, LEN_PRO_PROFILE_VELOCITY);	// VELOCITY

	//INFO_STREAM("Start Talking");

	int output = 2;
	double roll_f, pitch_f, yaw_f, delta_mouth; // filtered roll,pitch,yaw,mouth values	
	int dxl_goal_position[5] = { default_PITCH,default_ROLLR,default_ROLLL,default_YAW, default_MOUTH };

	if (image == true)
	{
		cv::namedWindow("img", cv::WND_PROP_FULLSCREEN);
		cv::setWindowProperty("img", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
		cv::Mat img = cv::imread(songname + ".jpg", cv::IMREAD_COLOR);
		if (img.empty()) {
			std::cout << "Could not read the image: " << songname << std::endl;
			return 1;
		}
		cv::imshow("img", img);
		cv::waitKey(1);
	}

	//----- HOMING MODE
	std::chrono::high_resolution_clock::time_point initiateTime = std::chrono::high_resolution_clock::now();
	auto awaketime = initiateTime + 500ms;
	setDXLGoalPosition(dxl_goal_position, default_PITCH, default_ROLLR, default_ROLLL, default_YAW, default_MOUTH);
	if (!moveDXLtoDesiredPosition(portHandler, packetHandler, dxl_goal_position, DXL_PROFILE_VELOCITY_HOMING)) { return 0; }
	while (std::chrono::high_resolution_clock::now() < awaketime) { std::this_thread::yield(); }

	// loop
	std::chrono::high_resolution_clock::time_point startTime;
	int __iter = 0;
	bool firstRun = true;
	while (headGesture.good() && mouthMotion.good())
	{
		if (_kbhit()) {

			char c = _getch();
			FlushConsoleInputBuffer(GetStdHandle(STD_INPUT_HANDLE));

			// quit
			if (c == 'q' || c == 'Q') {
				if (AIspeaker == true) output = -1;
				else output = 0;
				//PlaySound(TEXT("Sound\\empty.wav"), NULL, SND_FILENAME || SND_ASYNC || SND_NODEFAULT);
				playwav("head_mouth_generation/mc/empty.wav");
				break;
			}
			// skip
			if (c == 's' || c == 'S') {
				output = 1;
				//PlaySound(TEXT("Sound\\empty.wav"), NULL, SND_FILENAME || SND_ASYNC || SND_NODEFAULT);
				playwav("head_mouth_generation/mc/empty.wav");
				break;
			}
			// pause
			if (c == 'p' || c == 'P') {
				output = 2;
				//PlaySound(TEXT("Sound\\empty.wav"), NULL, SND_FILENAME || SND_ASYNC || SND_NODEFAULT);
				playwav("head_mouth_generation/mc/empty.wav");
				break;
			}
		}

		std::vector<std::string> headRow = csv_read_row(headGesture, ',');
		std::vector<std::string> mouthRow = csv_read_row(mouthMotion, ',');

		roll_f = stof(headRow[0]);
		pitch_f = stof(headRow[1]);
		yaw_f = stof(headRow[2]);
		//roll_f = 0;
		//pitch_f = 0;
		//yaw_f = 0;
		delta_mouth = stof(mouthRow[0]);
		//delta_mouth = 0;

		vector<int> DXL = RPY2DXL4singing(roll_f, pitch_f, yaw_f, delta_mouth);
		setDXLGoalPosition(dxl_goal_position, DXL[0], DXL[1], DXL[2], DXL[3], DXL[4]);

		if (firstRun) {
			//moveDXLtoDesiredPosition_km(groupSyncWriteVelocity, groupSyncWritePosition, DXL_PROFILE_VELOCITY_HOMING / 2, dxl_goal_position);
			//std::this_thread::sleep_for(50ms);

			playwav(audiopath);

			startTime = std::chrono::high_resolution_clock::now();
			firstRun = false;
		}
		else {
			if (!moveDXLtoDesiredPosition(portHandler, packetHandler, dxl_goal_position)) break;
		}
		auto awaketime = startTime + __iter * 40ms;
		while (std::chrono::high_resolution_clock::now() < awaketime) { std::this_thread::yield(); }

		__iter++;
	}

	if (output == 2) {
		//----- HOMING MODE
		initiateTime = std::chrono::high_resolution_clock::now();
		awaketime = initiateTime + 1000ms;
		setDXLGoalPosition(dxl_goal_position, default_PITCH, default_ROLLR, default_ROLLL, default_YAW, default_MOUTH);
		if (!moveDXLtoDesiredPosition(portHandler, packetHandler, dxl_goal_position, DXL_PROFILE_VELOCITY_HOMING / 3)) { return 0; }
		while (std::chrono::high_resolution_clock::now() < awaketime) { std::this_thread::yield(); }
	}

	////----- HOMING MODE
	//setDXLGoalPosition(dxl_goal_position, default_PITCH, default_ROLLR, default_ROLLL, default_YAW, default_MOUTH);
	//if (!moveDXLtoDesiredPosition_km(groupSyncWriteVelocity, groupSyncWritePosition, DXL_PROFILE_VELOCITY_HOMING, dxl_goal_position)) { return 0; }
	//std::this_thread::sleep_for(500ms);
	return output;
}

int waitfornextcommand_withmoving(dynamixel::PacketHandler* packetHandler, dynamixel::PortHandler* portHandler, int& iter)
{
	// file input	
	string headmotionpath = "head_mouth_generation/empty_10min.csv";

	std::ifstream headGesture(headmotionpath);

	if (headGesture.fail())
		return(cout << "Empty HeadGesture File not found" << endl) && true;

	// dxl
	dynamixel::GroupSyncWrite groupSyncWritePosition(portHandler, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION);		// POSITION
	dynamixel::GroupSyncWrite groupSyncWriteVelocity(portHandler, packetHandler, ADDR_PRO_PROFILE_VELOCITY, LEN_PRO_PROFILE_VELOCITY);	// VELOCITY

	int output = 2;
	double roll_f, pitch_f, yaw_f, delta_mouth; // filtered roll,pitch,yaw,mouth values	
	int dxl_goal_position[5] = { default_PITCH,default_ROLLR,default_ROLLL,default_YAW, default_MOUTH };



	static int veryfirst = 0;
	if (veryfirst == 0) {
		//----- HOMING MODE
		std::chrono::high_resolution_clock::time_point initiateTime_veryfirst = std::chrono::high_resolution_clock::now();
		auto awaketime_veryfirst = initiateTime_veryfirst + 2000ms;
		setDXLGoalPosition(dxl_goal_position, default_PITCH, default_ROLLR, default_ROLLL, default_YAW, default_MOUTH);
		if (!moveDXLtoDesiredPosition(portHandler, packetHandler, dxl_goal_position, DXL_PROFILE_VELOCITY_HOMING / 2)) { return 0; }
		while (std::chrono::high_resolution_clock::now() < awaketime_veryfirst) { std::this_thread::yield(); }
		veryfirst = 1;
	}




	// loop
	std::chrono::high_resolution_clock::time_point startTime;
	int __iter = 0;
	bool firstRun = true;
	while (headGesture.good())
	{
		if (_kbhit()) {

			char c = _getch();
			FlushConsoleInputBuffer(GetStdHandle(STD_INPUT_HANDLE));

			// quit
			if (c == 'q' || c == 'Q') {
				output = 0;
				break;
			}
			// play next 
			if (c == 'p' || c == 'P') {
				output = 1;
				break;
			}
			// iteration down
			if (c == 'n' || c == 'N') {
				iter--;
				cout << "(next play iter = " << iter << ")" << endl;
				//break;
			}
			// iteration up
			if (c == 'm' || c == 'M') {
				iter++;
				cout << "(next play iter = " << iter << ")" << endl;
				//break;
			}
			//// reset 10 minutes
			if (c == 'z' || c == 'Z')
				break;

			// unexpected ment input
			//if (c == 'u' || c == 'U') {
			//	output = 3;
			//	break;
			//}

			if (c == '0') { iter = 0;  cout << "(next play iter = " << iter << ")" << endl; }
			if (c == '1') { iter = 10; cout << "(next play iter = " << iter << ")" << endl; }
			if (c == '2') { iter = 20; cout << "(next play iter = " << iter << ")" << endl; }
			if (c == '3') { iter = 30; cout << "(next play iter = " << iter << ")" << endl; }
			if (c == '4') { iter = 40; cout << "(next play iter = " << iter << ")" << endl; }
			if (c == '5') { iter = 50; cout << "(next play iter = " << iter << ")" << endl; }
			if (c == '6') { iter = 60; cout << "(next play iter = " << iter << ")" << endl; }
			if (c == '7') { iter = 70; cout << "(next play iter = " << iter << ")" << endl; }
			if (c == '8') { iter = 80; cout << "(next play iter = " << iter << ")" << endl; }
			if (c == '9') { iter = 90; cout << "(next play iter = " << iter << ")" << endl; }
		}

		std::vector<std::string> headRow = csv_read_row(headGesture, ',');


		//if (__iter % 2 == 0) {
		//	roll_f = stof(headRow[0]);
		//	pitch_f = stof(headRow[1]);
		//	yaw_f = stof(headRow[2]);
		//	delta_mouth = 0;

		//	double ratiooo = 2;
		//	vector<int> DXL = RPY2DXL4singing(roll_f * ratiooo, pitch_f * ratiooo, yaw_f * ratiooo, delta_mouth);
		//	setDXLGoalPosition(dxl_goal_position, DXL[0], DXL[1], DXL[2], DXL[3], DXL[4]);
		//}

		roll_f = stof(headRow[0]);
		pitch_f = stof(headRow[1]);
		yaw_f = stof(headRow[2]);
		delta_mouth = 0;

		double ratiooo = 1.3;
		vector<int> DXL = RPY2DXL4singing(roll_f * ratiooo, pitch_f * ratiooo, yaw_f * ratiooo, delta_mouth);
		setDXLGoalPosition(dxl_goal_position, DXL[0], DXL[1], DXL[2], DXL[3], DXL[4]);

		if (!moveDXLtoDesiredPosition(portHandler, packetHandler, dxl_goal_position)) break;

		//if (firstRun) {
		//	moveDXLtoDesiredPosition_km(groupSyncWriteVelocity, groupSyncWritePosition, DXL_PROFILE_VELOCITY_HOMING, dxl_goal_position);
		//	std::this_thread::sleep_for(100ms);

		//	startTime = std::chrono::high_resolution_clock::now();
		//	firstRun = false;
		//}
		//else {
		//	if (!moveDXLtoDesiredPosition_NoVelLimit_km(packetHandler, groupSyncWritePosition, dxl_goal_position)) break;
		//}

		//cout << roll_f << endl;



		if (firstRun) {
			startTime = std::chrono::high_resolution_clock::now();
			firstRun = false;
		}

		auto awaketime = startTime + __iter * 40ms;
		while (std::chrono::high_resolution_clock::now() < awaketime) { std::this_thread::yield(); }

		//std::this_thread::sleep_for(40ms);

		__iter++;
	}

	if (output == 2) {
		//----- HOMING MODE
		std::chrono::high_resolution_clock::time_point initiateTime_end = std::chrono::high_resolution_clock::now();
		auto awaketime_end = initiateTime_end + 1500ms;
		setDXLGoalPosition(dxl_goal_position, default_PITCH, default_ROLLR, default_ROLLL, default_YAW, default_MOUTH);
		if (!moveDXLtoDesiredPosition(portHandler, packetHandler, dxl_goal_position, DXL_PROFILE_VELOCITY_HOMING / 2)) { return 0; }
		while (std::chrono::high_resolution_clock::now() < awaketime_end) { std::this_thread::yield(); }
	}

	//cout << "output == " << output << endl;

	return output;
}


int unexpectedInput()
{
	//PlaySound(TEXT("Sound\\t_15sec.wav"), NULL, SND_FILENAME || SND_ASYNC || SND_NODEFAULT);
	playwav("head_mouth_generation/empty_10min.csv");
	return 2;
}

void test(int& status) {
	//for 소스1
	while (true) {
		if (status == 1) {
			return;
		}
		printf("in loop\n");
		Sleep(2000);
	}
}

VectorXd robot_to_cam(double x, double y, double z) {
	// 실제값 * 보정값까지 고려
	VectorXd pH(3);
	// VectorXd 를 쓰는 경우와 Mat1f, Point3f 를 쓰는 경우 자료형을 맞춰주어야하기 때문에 다 뜯어서 쓰기.

	// define P_cs (cam to robot)
	VectorXd P_cs(3);
	P_cs(0) = double(Poc_y);
	P_cs(1) = double(Poc_z);
	P_cs(2) = -double(Poc_x);

	// define P_s (robot frame)
	VectorXd P_s(3);
	P_s(0) = x;
	P_s(1) = y;
	P_s(2) = z;

	// rotation (x R_cs)
	VectorXd rotate_P_s(3);
	rotate_P_s(0) = -P_s(1);
	rotate_P_s(1) = -P_s(2);
	rotate_P_s(2) = P_s(0);

	// define P_c (cam frame)
	VectorXd P_c(3);
	P_c = P_cs + rotate_P_s;

	return P_c;
}