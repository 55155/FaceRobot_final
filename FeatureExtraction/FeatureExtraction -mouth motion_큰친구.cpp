///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2017, Carnegie Mellon University and University of Cambridge,
// all rights reserved.
//
// ACADEMIC OR NON-PROFIT ORGANIZATION NONCOMMERCIAL RESEARCH USE ONLY
//
// BY USING OR DOWNLOADING THE SOFTWARE, YOU ARE AGREEING TO THE TERMS OF THIS LICENSE AGREEMENT.  
// IF YOU DO NOT AGREE WITH THESE TERMS, YOU MAY NOT USE OR DOWNLOAD THE SOFTWARE.
//
// License can be found in OpenFace-license.txt

//     * Any publications arising from the use of this software, including but
//       not limited to academic journal and conference publications, technical
//       reports and manuals, must cite at least one of the following works:
//
//       OpenFace 2.0: Facial Behavior Analysis Toolkit
//       Tadas Baltru큄aitis, Amir Zadeh, Yao Chong Lim, and Louis-Philippe Morency
//       in IEEE International Conference on Automatic Face and Gesture Recognition, 2018  
//
//       Convolutional experts constrained local model for facial landmark detection.
//       A. Zadeh, T. Baltru큄aitis, and Louis-Philippe Morency,
//       in Computer Vision and Pattern Recognition Workshops, 2017.    
//
//       Rendering of Eyes for Eye-Shape Registration and Gaze Estimation
//       Erroll Wood, Tadas Baltru큄aitis, Xucong Zhang, Yusuke Sugano, Peter Robinson, and Andreas Bulling 
//       in IEEE International. Conference on Computer Vision (ICCV),  2015 
//
//       Cross-dataset learning and person-specific normalisation for automatic Action Unit detection
//       Tadas Baltru큄aitis, Marwa Mahmoud, and Peter Robinson 
//       in Facial Expression Recognition and Analysis Challenge, 
//       IEEE International Conference on Automatic Face and Gesture Recognition, 2015 
//
///////////////////////////////////////////////////////////////////////////////


// FeatureExtraction.cpp : Defines the entry point for the feature extraction console application.

// Local includes
#include <conio.h>
#include <VisualizationUtils.h>
#include <chrono>
#include "dynamixel_sdk.h"
#include <dense>
#include <string>
#include <fstream>
#include <cstdio>
#include <iostream>
#include <thread>
#include <mutex>
#include <stdio.h>
#include<conio.h>
#include<vector>
//#include <Eigen/Core>;
using namespace Eigen;

// Control table address
#define ADDR_PRO_TORQUE_ENABLE          64                 // Control table address is different in Dynamixel model
#define ADDR_PRO_GOAL_POSITION          116
#define ADDR_PRO_PRESENT_POSITION       132
#define ADDR_PRO_PROFILE_VELOCITY		112
#define ADDR_PRO_PRESENT_CURRENT		126

// Data Byte Length
#define LEN_PRO_GOAL_POSITION           4
#define LEN_PRO_PRESENT_POSITION        4
#define LEN_PRO_PROFILE_VELOCITY        4
#define LEN_PRO_PRESENT_CURRENT			2

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

#define DXL1_ID                         1                  // 앞쪽
#define DXL2_ID                         2                  // 오른쪽(관찰자 시점)
#define DXL3_ID                         3                  // 왼쪽(관찰자 시점)
#define DXL4_ID                         4                 // yaw
#define DXL5_ID                         5               // mouth

#define default_PITCH					2000
#define default_ROLLR					1970
#define default_ROLLL					1900
#define default_YAW						2000
#define default_MOUTH					2000

#define BAUDRATE                        57600
#define DEVICENAME                      "COM6"             // Check which port is being used on your controller, ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"


#define TORQUE_ENABLE                   1                // Value for enabling the torque
#define TORQUE_DISABLE                  0                // Value for disabling the torque

#define PROFILE_VELOCITY_DEFAULT		0
#define PROFILE_VELOCITY_HOMING			50

using std::cout;
using std::endl;

#ifndef CONFIG_DIR
#define CONFIG_DIR "~"
#endif

#define INFO_STREAM( stream ) \
std::cout << stream << std::endl

#define WARN_STREAM( stream ) \
std::cout << "Warning: " << stream << std::endl

#define ERROR_STREAM( stream ) \
std::cout << "Error: " << stream << std::endl
using std::chrono::duration_cast;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::chrono::system_clock;
using namespace std;
#include <conio.h>
#include <Windows.h>
#pragma comment(lib,"winmm.lib")
#include <MMSystem.h>
static void printErrorAndAbort(const std::string& error)
{
	std::cout << error << std::endl;
}
#include <windows.h>
#define FATAL_STREAM( stream ) \
printErrorAndAbort( std::string( "Fatal error: " ) + stream )

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

void to_dxl_param(uint8_t output[], int input)
{
	output[0] = DXL_LOBYTE(DXL_LOWORD(input));
	output[1] = DXL_HIBYTE(DXL_LOWORD(input));
	output[2] = DXL_LOBYTE(DXL_HIWORD(input));
	output[3] = DXL_HIBYTE(DXL_HIWORD(input));
}

std::wstring s2ws(const std::string& s) //
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

void homing_mode(dynamixel::PacketHandler* packetHandler, dynamixel::PortHandler* portHandler)
{
	dynamixel::GroupSyncWrite groupSyncWriteVelocity(portHandler, packetHandler, ADDR_PRO_PROFILE_VELOCITY, LEN_PRO_PROFILE_VELOCITY);
	dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION);

	uint8_t param_profile_velocity_default[4], param_profile_velocity_homing[4]
		, param_zero_position_pitch[4], param_zero_position_rollr[4], param_zero_position_rolll[4], param_zero_position_yaw[4], param_zero_position_mouth[4];
	int DXL_ID[5] = { DXL1_ID, DXL2_ID, DXL3_ID, DXL4_ID, DXL5_ID };
	int dxl_comm_result;
	
	// Allocate profile velocity value into byte array
	to_dxl_param(param_profile_velocity_homing, PROFILE_VELOCITY_HOMING);

	// Add Dynamixel profile velocity value to the Syncwrite storage
	for (int i = 0; i < 5; i++) { if (!groupSyncWriteVelocity.addParam(DXL_ID[i], param_profile_velocity_homing)) { fprintf(stderr, "[ID:%03d] groupSyncWriteVelocity addparam failed", DXL_ID[i]); return; } }

	// Syncwrite profile velocity
	dxl_comm_result = groupSyncWriteVelocity.txPacket();
	if (dxl_comm_result != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));

	//Allocate goal position value into byte array
	to_dxl_param(param_zero_position_pitch, default_PITCH);
	to_dxl_param(param_zero_position_rollr, default_ROLLR);
	to_dxl_param(param_zero_position_rolll, default_ROLLL);
	to_dxl_param(param_zero_position_yaw, default_YAW);
	to_dxl_param(param_zero_position_mouth, default_MOUTH);


	// Add Dynamixel goal position value to the Syncwrite parameter storage
	if (!groupSyncWrite.addParam(DXL1_ID, param_zero_position_pitch)) { fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL1_ID); return; }
	if (!groupSyncWrite.addParam(DXL2_ID, param_zero_position_rollr)) { fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL2_ID); return; }
	if (!groupSyncWrite.addParam(DXL3_ID, param_zero_position_rolll)) { fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL3_ID); return; }
	if (!groupSyncWrite.addParam(DXL4_ID, param_zero_position_yaw)) { fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL4_ID); return; }
	if (!groupSyncWrite.addParam(DXL5_ID, param_zero_position_mouth)) { fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL5_ID); return; }


	// Syncwrite goal position
	dxl_comm_result = groupSyncWrite.txPacket();
	if (dxl_comm_result != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));

	Sleep(3000);

	// Clear syncwrite parameter storage
	groupSyncWriteVelocity.clearParam();
	groupSyncWrite.clearParam();

	// Allocate profile velocity value into byte array
	to_dxl_param(param_profile_velocity_default, PROFILE_VELOCITY_DEFAULT);

	// Add Dynamixel profile velocity value to the Syncwrite storage
	for (int i = 0; i < 5; i++) { if (!groupSyncWriteVelocity.addParam(DXL_ID[i], param_profile_velocity_default)) { fprintf(stderr, "[ID:%03d] groupSyncWriteVelocity addparam failed", DXL_ID[i]); return; } }

	// Syncwrite goal profile velocity
	dxl_comm_result = groupSyncWriteVelocity.txPacket();
	if (dxl_comm_result != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
}

int let_sing(dynamixel::PacketHandler* packetHandler, dynamixel::GroupSyncWrite groupSyncWrite, string songname, string songcsv)
{
	//ofstream outData;
	//outData.open("outData.csv", ios::app);

	INFO_STREAM("Start Singing~♬");

	int output = 1;

	std::ifstream in(songcsv); //revision: museum, stay, momobic_2-1_10sec dxl
	std::ifstream head("전시sound\\머리움직임2-delta.csv");
	if (in.fail()) { cout << "Mouth csv File not found" << endl; return 1; }
	if (head.fail()) { cout << "Head csv File not found" << endl; return 1; }

	double dxl_goal_postion[5] = { default_PITCH, default_ROLLR, default_ROLLL, default_YAW, default_MOUTH };
	double delta_dxl[5];		//pitch,rollr,rolll,yaw,mouth 순
	
	auto begin = std::chrono::high_resolution_clock::now();
	int iter = 0;
	int sound = 0;	//sound flag

	//roll, pitch, yaw parameter goal position
	uint8_t param_goal_position_pitch[4], param_goal_position_rollr[4], param_goal_position_rolll[4], param_goal_position_yaw[4], param_goal_position_mouth[4];


	while (in.good() && head.good()) {

		if (_kbhit()) {
			char c = _getch();
			if (c == 'q' || c == 'Q') {
				output = 0;
				PlaySound(TEXT("전시sound\\empty.wav"), NULL, SND_FILENAME || SND_ASYNC || SND_NODEFAULT);
				break;
			}
			if (c == 's' || c == 'S') {
				output = 1;
				PlaySound(TEXT("전시sound\\empty.wav"), NULL, SND_FILENAME || SND_ASYNC || SND_NODEFAULT);
				break;
			}
		}

		std::vector<std::string> mouthrow = csv_read_row(in, ',');
		std::vector<std::string> headrow = csv_read_row(head, ',');

		delta_dxl[0] = 1.2 * stof(headrow[1]);  //pitch
		delta_dxl[1] = 1.4 * stof(headrow[2]);  //rollr
		delta_dxl[2] = 1.4 * stof(headrow[3]);  //rolll
		delta_dxl[3] = 1.2 * stof(headrow[4]);  //yaw
		delta_dxl[4] = 1.4 * stof(mouthrow[0]); //mouth

		//cout << delta_dxl[0] <<"," << delta_dxl[1] << "," << delta_dxl[2] << "," << delta_dxl[3] <<","<< delta_dxl[4] << endl;
		//DXL 값들: offset고려, dynamixel로 실험

		//cout << delta_dxl[4] << endl;
		dxl_goal_postion[0] = default_PITCH - delta_dxl[0];
		dxl_goal_postion[1] = default_ROLLR - delta_dxl[1] - (delta_dxl[4] / 1);
		dxl_goal_postion[2] = default_ROLLL - delta_dxl[2] - (delta_dxl[4] / 1);
		dxl_goal_postion[3] = default_YAW - delta_dxl[3];
		dxl_goal_postion[4] = default_MOUTH - delta_dxl[4] - (delta_dxl[0]) ;

		//cout << dxl_goal_postion[0] << "," << dxl_goal_postion[1] << "," << dxl_goal_postion[2] << "," << dxl_goal_postion[3] << endl;

		//Allocate goal position value into byte array for pitch
		to_dxl_param(param_goal_position_pitch, (int)dxl_goal_postion[0]);
		to_dxl_param(param_goal_position_rollr, (int)dxl_goal_postion[1]);
		to_dxl_param(param_goal_position_rolll, (int)dxl_goal_postion[2]);
		to_dxl_param(param_goal_position_yaw, (int)dxl_goal_postion[3]);
		to_dxl_param(param_goal_position_mouth, (int)dxl_goal_postion[4]);

		// Add parameter storage for Dynamixel#1 goal position
		if (!groupSyncWrite.addParam(DXL1_ID, param_goal_position_pitch)) { fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL1_ID); return 0; }
		if (!groupSyncWrite.addParam(DXL2_ID, param_goal_position_rollr)) { fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL2_ID); return 0; }
		if (!groupSyncWrite.addParam(DXL3_ID, param_goal_position_rolll)) { fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL3_ID); return 0; }
		if (!groupSyncWrite.addParam(DXL4_ID, param_goal_position_yaw)) { fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL4_ID); return 0; }
		if (!groupSyncWrite.addParam(DXL5_ID, param_goal_position_mouth)) { fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL5_ID); return 0; }

		// Play Sound
		if (sound == 0) {
			std::wstring stemp = s2ws(songname);
			LPCWSTR songname_result = stemp.c_str();
			PlaySound(songname_result, NULL, SND_FILENAME || SND_ASYNC || SND_NODEFAULT); //revision: museum, stay
			//PlaySound(TEXT("C:\\Users\\USER\\Desktop\\jacheon\\sound\\전시 sound\\사라진모든것들에게.wav"), NULL, SND_FILENAME || SND_ASYNC || SND_NODEFAULT); //revision: museum, stay
			begin = std::chrono::steady_clock::now();
			sound = 1;
		}

		// Syncwrite goal position and profile velocity
		auto awaketime = begin + iter * 40ms;
		while (std::chrono::high_resolution_clock::now() < awaketime) { std::this_thread::yield(); }
		int dxl_comm_result = groupSyncWrite.txPacket();
		if (dxl_comm_result != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));

		//auto millisec_since_epoch_start2 = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
		//outData << millisec_since_epoch_start2 << endl;

		iter++;

		// Clear Syncwrite parameter storage
		groupSyncWrite.clearParam();

	}	
	return output;
}

int main(int argc, char** argv)
{

	// (1) Dynamixel setting (start)
	// Initialize PortHandler instance
	// Set the port path
	// Get methods and members of PortHandlerLinux or PortHandlerWindows
	dynamixel::PortHandler* portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

	// Initialize PacketHandler instance
	// Set the protocol version
	// Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
	dynamixel::PacketHandler* packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

	// Initialize GroupSyncWrite instance
	dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION);
	dynamixel::GroupSyncWrite groupSyncWriteVelocity(portHandler, packetHandler, ADDR_PRO_PROFILE_VELOCITY, LEN_PRO_PROFILE_VELOCITY);

	// Initialize Groupsyncread instance for Present Values
	dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
	dynamixel::GroupSyncRead groupSyncReadCurrent(portHandler, packetHandler, ADDR_PRO_PRESENT_CURRENT, LEN_PRO_PRESENT_CURRENT);

	int dxl_comm_result = COMM_TX_FAIL;               // Communication result
	bool dxl_getdata_result = false;                  // GetParam result

	// Read present position 변수 선언
	int32_t dxl_present_position[5] = { 0,0,0,0,0 }; 
	int16_t dxl_present_current[5] = { 0,0,0,0,0 };
	uint8_t param_profile_velocity_default[4], param_profile_velocity_homing[4]
		,param_zero_position_pitch[4], param_zero_position_rollr[4], param_zero_position_rolll[4], param_zero_position_yaw[4], param_zero_position_mouth[4];
	uint8_t dxl_error = 0;

	using namespace std::this_thread;     // sleep_for, sleep_until
	using namespace std::chrono_literals; // ns, us, ms, s, h, etc.
	using std::chrono::system_clock;
	time_t curr;
	struct tm* d;
	curr = time(NULL);
	d = localtime(&curr);
	ofstream outData1;
	ofstream outData2;
	//ofstream outData3;
	ofstream outDataPacket;
	ofstream outData_current;

	string year = std::to_string(d->tm_year + 1900);
	string month = "0" + std::to_string(d->tm_mon + 1);
	string date = std::to_string(d->tm_mday);
	string hour = std::to_string(d->tm_hour);
	string min = std::to_string(d->tm_min);
	string second = std::to_string(d->tm_sec);
	outData1.open(year + month + date + ";" + hour + "-" + min + "-" + second + " outData1.csv", ios::app);
	outData2.open(year + month + date + ";" + hour + "-" + min + "-" + second + " outData2" + ".csv", ios::app);
	//outData3.open(year + month + date + ";" + hour + "-" + min + "-" + second + " dxl" + ".csv", ios::app);
	outDataPacket.open(year + month + date + ";" + hour + "-" + min + "-" + second + " outDataPacket" + ".csv", ios::app);
	outData_current.open(year + month + date + ";" + hour + "-" + min + "-" + second + " current" + ".csv", ios::app);
	int DXL_ID[5] = { DXL1_ID, DXL2_ID, DXL3_ID, DXL4_ID, DXL5_ID };

	// Add parameter storage for present position
	for (int i = 0; i < 5; i++){if (!groupSyncRead.addParam(DXL_ID[i])) { fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", DXL_ID[i]);return 0; } }

	// Add parameter storage for present current
	for (int i = 0; i < 5; i++) { if (!groupSyncReadCurrent.addParam(DXL_ID[i])) { fprintf(stderr, "[ID:%03d] groupSyncReadCurrent addparam failed", DXL_ID[i]); return 0; } }


	// Open port
	if (portHandler->openPort())
	{
		printf("Succeeded to open the port!\n");
	}
	else
	{
		printf("Failed to open the port!\n");
		printf("Press any key to terminate...\n");
		//getch();
		return 0;
	}

	// Set port baudrate
	if (portHandler->setBaudRate(BAUDRATE))
	{
		printf("Succeeded to change the baudrate!\n");
	}
	else
	{
		printf("Failed to change the baudrate!\n");
		printf("Press any key to terminate...\n");
		//getch();
		return 0;
	}

	// Enable Dynamixel Torque
	for (int i = 0; i < 5; i++)
	{
		dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID[i], ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
		if (dxl_comm_result != COMM_SUCCESS)
		{
			printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
		}
		else if (dxl_error != 0)
		{
			printf("%s\n", packetHandler->getRxPacketError(dxl_error));
		}
		else
		{
			printf("Dynamixel#%d has been successfully connected \n", DXL_ID[i]);
		}
	}


	//// Syncread present position value
	//if (groupSyncRead.txRxPacket() != COMM_SUCCESS){ printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));}
	//for (int i = 0; i < 5; i++){if (groupSyncRead.getError(DXL_ID[i], &dxl_error)){printf("[ID:%03d] %s\n", DXL_ID[i], packetHandler->getRxPacketError(dxl_error));}
	//}

	//// Get Dynamixel present position value
	//for (int i = 0; i < 5; i++){ dxl_present_position[i] = groupSyncRead.getData(DXL_ID[i], ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION); printf("[ID:%03d] PresPos:%03d \n", DXL_ID[i], dxl_present_position[i]);}

	//// Syncread present current value
	//if (groupSyncReadCurrent.txRxPacket() != COMM_SUCCESS){ printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));}
	//for (int i = 0; i < 5; i++){if (groupSyncReadCurrent.getError(DXL_ID[i], &dxl_error)){printf("[ID:%03d] %s\n", DXL_ID[i], packetHandler->getRxPacketError(dxl_error));}
	//}

	//// Get Dynamixel present Current value
	//for (int i = 0; i < 5; i++){ dxl_present_current[i] = groupSyncReadCurrent.getData(DXL_ID[i], ADDR_PRO_PRESENT_CURRENT, LEN_PRO_PRESENT_CURRENT); printf("[ID:%03d] PresCurrent:%03d \n", DXL_ID[i], dxl_present_current[i]);}

	
	homing_mode(packetHandler, portHandler);
	// (1) Dynamixel setting (end)

	int isStop = 0;
	bool flag_breakWhile = false;

	//Playlist
	while (true) {
		string folderName = "전시sound\\";

		vector<string> PlayList_WAV;
		PlayList_WAV.push_back("tiny_riot");
		PlayList_WAV.push_back("거꾸로강을거슬러오르는저힘찬연어들처럼-강산에");
		PlayList_WAV.push_back("우리의꿈");		
		PlayList_WAV.push_back("라젠카");
		PlayList_WAV.push_back("신호등");
		PlayList_WAV.push_back("butter");
		PlayList_WAV.push_back("dynamite");
		PlayList_WAV.push_back("고백");
		PlayList_WAV.push_back("사라진모든것들에게");
//		PlayList_WAV.push_back("부럽지가않아");
		PlayList_WAV.push_back("과제곡");
		PlayList_WAV.push_back("나에게로떠나는여행");
		PlayList_WAV.push_back("중독된사랑");
		PlayList_WAV.push_back("사랑인가봐");
		PlayList_WAV.push_back("취기를빌려");
		PlayList_WAV.push_back("폰서트");
		PlayList_WAV.push_back("취중고백");

		for (int i = 0; i < PlayList_WAV.size(); i++) {
			cout << "Playlist number " << i + 1 << "/" << PlayList_WAV.size() << endl;
			if (!let_sing(packetHandler, groupSyncWrite, folderName + PlayList_WAV[i] + ".wav", folderName + PlayList_WAV[i] + "_mr없음-delta.csv")) {
				flag_breakWhile = true;
				break;
			}				
			homing_mode(packetHandler, portHandler);
		}
		if (flag_breakWhile)
			break;
	}

	homing_mode(packetHandler, portHandler);
	// Disable Dynamixel Torque
	for (int i = 0; i < 5; i++)
	{
		dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID[i], ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
		if (dxl_comm_result != COMM_SUCCESS)
		{
			printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
		}
		else if (dxl_error != 0)
		{
			printf("%s\n", packetHandler->getRxPacketError(dxl_error));
		} 
	}
	
	// Close port
	portHandler->closePort();
	INFO_STREAM("Close the port");

	return 0;

}