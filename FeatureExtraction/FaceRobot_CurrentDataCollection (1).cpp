/*
* made by KM 2022.06.14.
*/

// FACEROBOT KEYBOARD CONTROLLER

//#include "dynamixel_sdk.h"
//#include <dense>
//#include <string>
//#include <fstream>
//#include <iostream>
//#include <thread>
//#include <mutex>
//#include <chrono>
//#include <conio.h>
//#include <windows.h>
//#include <Eigen/Core>;

#include "MacrosAndFunctions.h"  

//using namespace Eigen;
//using namespace std;
//using std::cout;
//using std::endl;
//using std::chrono::duration_cast;
//using std::chrono::milliseconds;
//using std::chrono::seconds;
//using std::chrono::system_clock;
//using namespace std::this_thread;     // sleep_for, sleep_until
//using namespace std::chrono_literals; // ns, us, ms, s, h, etc.

// FUNCTIONS
void setDXLGoalPosition(int dxl_goal_position[], int pitch, int rollr, int rolll, int yaw, int mouth);
void to_DXL_param(uint8_t output[], int input);
bool moveDXLtoDesiredPosition(dynamixel::GroupSyncWrite& groupSyncWriteVelocity, dynamixel::GroupSyncWrite& groupSyncWritePosition, int velocity, int dxl_goal_position[]);
bool moveDXLtoDesiredPosition_NoVelLimit(dynamixel::PacketHandler* packetHandler, dynamixel::GroupSyncWrite& groupSyncWritePosition, int dxl_goal_position[]);
vector<int> RPY2DXL(double roll_f, double pitch_f, double yaw_f , double mouth_f, int mode);
vector<bool> KeyBoardControl(double& roll, double& pitch, double& yaw, double& mouth, bool isStop);

int main()
{
	// (1) Dynamixel setting (start): Initialize PortHandler instance, Set the port path, Get methods and members of PortHandlerLinux or PortHandlerWindows
	dynamixel::PortHandler* portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

	// Initialize PacketHandler instance: Set the protocol version, Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
	dynamixel::PacketHandler* packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

	// Initialize GroupSyncWrite instance (cpp to dxl)
	dynamixel::GroupSyncWrite groupSyncWritePosition(portHandler, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION);		// POSITION
	dynamixel::GroupSyncWrite groupSyncWriteVelocity(portHandler, packetHandler, ADDR_PRO_PROFILE_VELOCITY, LEN_PRO_PROFILE_VELOCITY);	// VELOCITY

	// Initialize Groupsyncread instance for Present Position (dxl to cpp)
	dynamixel::GroupSyncRead groupSyncReadPosition(portHandler, packetHandler, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);	// POSITION
	dynamixel::GroupSyncRead groupSyncReadCurrent(portHandler, packetHandler, ADDR_PRO_PRESENT_CURRENT, LEN_PRO_PRESENT_CURRENT);		// CURRENT


	//--- MOTOR PARAMETWRS
	int dxl_comm_result = COMM_TX_FAIL;					// Communication result
	int dxl_goal_position[5] = { 0,0,0,0,0 };
	int32_t dxl_present_position[5] = { 0,0,0,0,0 };
	int16_t dxl_present_current[5] = { 0,0,0,0,0 };
	uint8_t dxl_error = 0;								// Dynamixel error
	int DXL_ID[5] = { DXL1_ID, DXL2_ID, DXL3_ID, DXL4_ID, DXL5_ID };

	//--- FILE WRITE PARAMETERS
	ofstream outData_openface, outData_position, outData_current;
	time_t present_time;
	struct tm* d;
	present_time = time(NULL);
	d = localtime(&present_time);	
	string year = std::to_string(d->tm_year + 1900);	
	string month = (d->tm_mon < 9) ? "0" + std::to_string(d->tm_mon + 1) : std::to_string(d->tm_mon + 1);
	string date = std::to_string(d->tm_mday);
	string hour = std::to_string(d->tm_hour);
	string min = std::to_string(d->tm_min);
	string second = std::to_string(d->tm_sec);
	if (FLAG_SAVE_DXL_PRESENT_POSITION)
		outData_position.open("_CSV/"+year + month + date + ";" + hour + "-" + min + "-" + second + " pos.csv", ios::app);
	if (FLAG_SAVE_DXL_PRESENT_CURRENT)
		outData_current.open("_CSV/" + year + month + date + ";" + hour + "-" + min + "-" + second + " current" + ".csv", ios::app);


	
	// Add parameter storages for present position and current values
	for (int i = 0; i < 5; i++) {
		if (!groupSyncReadPosition.addParam(DXL_ID[i])) { fprintf(stderr, "[ID:%03d] groupSyncReadPosition addparam failed", DXL_ID[i]); return 0; }
		if (!groupSyncReadCurrent.addParam(DXL_ID[i])) { fprintf(stderr, "[ID:%03d] groupSyncReadCurrent addparam failed", DXL_ID[i]); return 0; }
	} 

	// Open port
	if (portHandler->openPort())
		printf("Succeeded to open the port!\n");	
	else {
		printf("Failed to open the port!\n");
		printf("Press any key to terminate...\n");
		return 0;
	}

	// Set port baudrate
	if (portHandler->setBaudRate(BAUDRATE))
		printf("Succeeded to change the baudrate!\n");	
	else {
		printf("Failed to change the baudrate!\n");
		printf("Press any key to terminate...\n");
		return 0;
	}

	// Enable Dynamixel Torque
	for (int i = 0; i < 5; i++) {
		dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID[i], ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
		if (dxl_comm_result != COMM_SUCCESS)
			printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));		
		else if (dxl_error != 0)
			printf("%s\n", packetHandler->getRxPacketError(dxl_error));		
		else
			printf("Dynamixel#%d has been successfully connected \n", DXL_ID[i]);		
	}

	//----- HOMING MODE
	setDXLGoalPosition(dxl_goal_position, default_PITCH, default_ROLLR, default_ROLLL, default_YAW, default_MOUTH);
	if (!moveDXLtoDesiredPosition(groupSyncWriteVelocity, groupSyncWritePosition, DXL_PROFILE_VELOCITY_HOMING, dxl_goal_position)) { return 0; }

	std::this_thread::sleep_for(4s);

	// ----------------------------------------------------------------------------------------
	// ----------------------------------------------------------------------------------------
	// ----------------------------------------------------------------------------------------
	while (true) // this is not a for loop as we might also be reading from a webcam
	{	
		INFO_STREAM("Starting control"); 

		int mode = 0; // 0: mirroring, 1: cloning

		double roll = 0, pitch = 0, yaw = 0, mouth = 0;		

		int n_roll = 3, n_pitch = 3, n_yaw = 3; // odd number	

		double delta_roll = MAX_ROLL / ((n_roll - 1) / 2.0);
		double delta_pitch = MAX_PITCH / ((n_pitch - 1) / 2.0);
		double delta_yaw = MAX_YAW / ((n_yaw - 1) / 2.0);

		//// home position: -MAX_ROLL + delta_roll * (n_roll - 1) / 2
		//for (int i = (n_roll - 1) / 2; i < n_roll; i++)
		//	roll_set.push_back(-MAX_ROLL + i * delta_roll);
		//for (int i = 0; i < (n_roll - 1) / 2; i++)
		//	roll_set.push_back(-MAX_ROLL + i * delta_roll);

		vector<double> roll_set, pitch_set, yaw_set;
		for (int i = 0; i < n_roll; i++)
			roll_set.push_back(-MAX_ROLL + i * delta_roll);
		for (int i = 0; i < n_pitch; i++)
			pitch_set.push_back(-MAX_PITCH + i * delta_pitch);
		for (int i = 0; i < n_yaw; i++)
			yaw_set.push_back(-MAX_YAW + i * delta_yaw);  

		vector<vector<double>> total_configs; // size n_roll * n_pitch * n_yaw
		vector<double> config(3);
		//for (int i = 0; i < n_yaw; i++) {
			for (int j = 0; j < n_pitch; j++) {
				for (int k = 0; k < n_roll; k++) {					
					config[0] = roll_set[k];
					config[1] = pitch_set[j];
					//config[2] = yaw_set[i];
					config[2] = -MAX_YAW + delta_yaw * (n_yaw - 1) / 2;
					total_configs.push_back(config);
				}
			}
		//}
		cout << "Total configurations: " << total_configs.size() << endl;

		//char stop;
		bool toNextConfig = false, pause = false, terminate = false;
		int iter = 0, max_iter = total_configs.size();

		auto dynamixel_keyboard_control = [&]()
		{
			INFO_STREAM("dynamixel_control_start");

			INFO_STREAM("Press 's' to start");
			INFO_STREAM("'n': next configuration");
			INFO_STREAM("'p': pause data collection");
			while (true) {				
				if (_kbhit()) {
					char c = _getch();
					if (c == 's' || c == 'S')
						break;
				}
			}
			INFO_STREAM("------------------------ COLLECTING DATA...");

			while (true)
			{
				auto millisec_since_epoch1 = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();				

				if (_kbhit()) {
					char c = _getch();
					if (c == 'q' || c == 'Q')
						terminate = true;
					if (c == 'n' || c == 'N') {
						
						if (iter >= max_iter) {
							INFO_STREAM("All configurations done.");
							terminate = true;
						}
						else {
							toNextConfig = true;
							roll = total_configs[iter][0];
							pitch = total_configs[iter][1];
							yaw = total_configs[iter][2];
							cout << "roll  = " << roll << endl;
							cout << "pitch = " << pitch << endl;
							cout << "yaw   = " << yaw << endl;
						}
						iter++;						
					}	
					if (c == 'p' || c == 'P')
						pause = true;
				}

				if (terminate) {
					INFO_STREAM("dynamixel_control break");
					break;
				}

				if (pause) {
					INFO_STREAM("PAUSED");
					while (true) {
						if (_kbhit()) {
							char p = _getch();
							if (p == 'p' || p == 'P') {
								pause = false;
								INFO_STREAM("------------------------ COLLECTING DATA...");
								break;
							}
						}
					}
				}

				vector<int> DXL = RPY2DXL(roll, pitch, yaw, mouth, mode);
				setDXLGoalPosition(dxl_goal_position, DXL[0], DXL[1], DXL[2], DXL[3], DXL[4]);

				if (toNextConfig) {
					cout << "MOVING TO NEXT CONFIGURATION...(" << iter << "/" << max_iter << ")" << endl;
					//INFO_STREAM("MOVING TO NEXT CONFIGURATION...");
					if (!moveDXLtoDesiredPosition(groupSyncWriteVelocity, groupSyncWritePosition, DXL_PROFILE_VELOCITY_CONFIGCHANGE, dxl_goal_position)) return 0;
					Sleep(3000);
					toNextConfig = false;
					cout << "MOVED TO NEXT CONFIGURATION (" << iter << "/" << max_iter << ")" << endl;
					//INFO_STREAM("MOVED TO NEXT CONFIGURATION");
					INFO_STREAM("------------------------ COLLECTING DATA...");
				}
				else {
					if (!moveDXLtoDesiredPosition_NoVelLimit(packetHandler, groupSyncWritePosition, dxl_goal_position)) return 0;
				}


				// Clear syncwrite parameter storage
				groupSyncWritePosition.clearParam();
				groupSyncWriteVelocity.clearParam();

				if (FLAG_SAVE_DXL_PRESENT_POSITION)
				{
					dxl_comm_result = groupSyncReadPosition.txRxPacket();
					if (dxl_comm_result != COMM_SUCCESS)
						printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
					for (int i = 0; i < 5; i++) {
						if (groupSyncReadPosition.getError(DXL_ID[i], &dxl_error))
							printf("[ID:%03d] %s\n", DXL_ID[i], packetHandler->getRxPacketError(dxl_error));
					}

					// Get Dynamixel present position value
					for (int i = 0; i < 5; i++)
						dxl_present_position[i] = groupSyncReadPosition.getData(DXL_ID[i], ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);

					outData_position << millisec_since_epoch1 << "," << dxl_goal_position[0] << "," << dxl_present_position[0]
						<< "," << dxl_goal_position[1] << "," << dxl_present_position[1]
						<< "," << dxl_goal_position[2] << "," << dxl_present_position[2]
						<< "," << dxl_goal_position[3] << "," << dxl_present_position[3] << "," << dxl_goal_position[4] << "," << dxl_present_position[4]
						<< endl;
				}

				if (FLAG_SAVE_DXL_PRESENT_CURRENT)
				{
					dxl_comm_result = groupSyncReadCurrent.txRxPacket();
					if (dxl_comm_result != COMM_SUCCESS)
						printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
					for (int i = 0; i < 5; i++) {
						if (groupSyncReadCurrent.getError(DXL_ID[i], &dxl_error))
							printf("[ID:%03d] %s\n", DXL_ID[i], packetHandler->getRxPacketError(dxl_error));
					}

					// Get Dynamixel present current value
					for (int i = 0; i < 5; i++)
						dxl_present_current[i] = groupSyncReadCurrent.getData(DXL_ID[i], ADDR_PRO_PRESENT_CURRENT, LEN_PRO_PRESENT_CURRENT);

					outData_current << millisec_since_epoch1 << "," << dxl_present_current[0] << "," << dxl_present_current[1]
						<< "," << dxl_present_current[2] << "," << dxl_present_current[3] << "," << dxl_present_current[4] << endl;
				}
			}
		};

		std::thread t1 = std::thread(dynamixel_keyboard_control);
		t1.join();

		INFO_STREAM("Closed successfully");		
		break;
	}

	groupSyncWriteVelocity.clearParam();

	//----- HOMING MODE
	setDXLGoalPosition(dxl_goal_position, END_PITCH, END_ROLLR, END_ROLLL, END_YAW, END_MOUTH);
	if (!moveDXLtoDesiredPosition(groupSyncWriteVelocity, groupSyncWritePosition, DXL_PROFILE_VELOCITY_HOMING, dxl_goal_position)) { return 0; }

	std::this_thread::sleep_for(4s);

	// Disable Dynamixel Torque
	for (int i = 0; i < 5; i++)
	{
		dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID[i], ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
		if (dxl_comm_result != COMM_SUCCESS)
			printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));		
		else if (dxl_error != 0)
			printf("%s\n", packetHandler->getRxPacketError(dxl_error));
		//here
	}
	
	
	// Close port
	portHandler->closePort();
	return 0;

}

void setDXLGoalPosition(int out[], int in1, int in2, int in3, int in4, int in5)
{
	out[0] = in1;
	out[1] = in2;
	out[2] = in3;
	out[3] = in4;
	out[4] = in5;
}

void to_DXL_param(uint8_t output[], int input) 
{
	// Allocate motor control value into byte array for GroupSyncWrite (ex. goal position)	
	output[0] = DXL_LOBYTE(DXL_LOWORD(input));
	output[1] = DXL_HIBYTE(DXL_LOWORD(input));
	output[2] = DXL_LOBYTE(DXL_HIWORD(input));
	output[3] = DXL_LOBYTE(DXL_HIWORD(input));
}
	
bool moveDXLtoDesiredPosition(dynamixel::GroupSyncWrite& groupSyncWriteVelocity, dynamixel::GroupSyncWrite& groupSyncWritePosition, int velocity, int position[])
{
	uint8_t param_profile_velocity[4];
	int DXL_ID[5] = { DXL1_ID, DXL2_ID, DXL3_ID, DXL4_ID, DXL5_ID };

	to_DXL_param(param_profile_velocity, velocity);

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
	to_DXL_param(param_zero_position_pitch, position[0]);
	to_DXL_param(param_zero_position_rollr, position[1]);
	to_DXL_param(param_zero_position_rolll, position[2]);
	to_DXL_param(param_zero_position_yaw, position[3]);
	to_DXL_param(param_zero_position_mouth, position[4]);

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

	to_DXL_param(param_profile_velocity, DXL_PROFILE_VELOCITY); // unlimit motor velocity

	// Add Dynamixel profile velocity value to the Syncwrite storage
	for (int i = 0; i < 5; i++) {
		if (!groupSyncWriteVelocity.addParam(DXL_ID[i], param_profile_velocity)) {
			fprintf(stderr, "[ID:%03d] groupSyncWriteVelocity addparam failed", DXL_ID[i]);
			return false;
		}
	}

	// Syncwrite goal velocity
	dxl_comm_result = groupSyncWriteVelocity.txPacket();

	return true;
}

bool moveDXLtoDesiredPosition_NoVelLimit(dynamixel::PacketHandler* packetHandler, dynamixel::GroupSyncWrite& groupSyncWritePosition, int dxl_goal_position[])
{
	int DXL_ID[5] = { DXL1_ID, DXL2_ID, DXL3_ID, DXL4_ID, DXL5_ID };

	//roll, pitch, yaw parameter goal position
	uint8_t param_goal_position_pitch[4], param_goal_position_rollr[4], param_goal_position_rolll[4], param_goal_position_yaw[4], param_goal_position_mouth[4];

	// Allocate goal position value into byte array for pitch, rollr, rolll, yaw, mouth
	to_DXL_param(param_goal_position_pitch, dxl_goal_position[0]);
	to_DXL_param(param_goal_position_rollr, dxl_goal_position[1]);
	to_DXL_param(param_goal_position_rolll, dxl_goal_position[2]);
	to_DXL_param(param_goal_position_yaw, dxl_goal_position[3]);
	to_DXL_param(param_goal_position_mouth, dxl_goal_position[4]);

	// Add Dynamixel #1~#5 goal position values to the Syncwrite parameter storage (pith,rollright,rollleft,yaw,mouth)
	if (!groupSyncWritePosition.addParam(DXL_ID[0], param_goal_position_pitch)) { fprintf(stderr, "[ID:%03d] groupSyncWritePosition addparam failed", DXL_ID[0]); return 0; }
	if (!groupSyncWritePosition.addParam(DXL_ID[1], param_goal_position_rollr)) { fprintf(stderr, "[ID:%03d] groupSyncWritePosition addparam failed", DXL_ID[1]); return 0; }
	if (!groupSyncWritePosition.addParam(DXL_ID[2], param_goal_position_rolll)) { fprintf(stderr, "[ID:%03d] groupSyncWritePosition addparam failed", DXL_ID[2]); return 0; }
	if (!groupSyncWritePosition.addParam(DXL_ID[3], param_goal_position_yaw)) { fprintf(stderr, "[ID:%03d] groupSyncWritePosition addparam failed", DXL_ID[3]); return 0; }
	if (!groupSyncWritePosition.addParam(DXL_ID[4], param_goal_position_mouth)) { fprintf(stderr, "[ID:%03d] groupSyncWritePosition addparam failed", DXL_ID[4]); return 0; }

	// Syncwrite goal position
	int dxl_comm_result = groupSyncWritePosition.txPacket();
	if (dxl_comm_result != COMM_SUCCESS) { printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result)); return false; }

	return true;
}

vector<int> RPY2DXL(double roll_f, double pitch_f, double yaw_f, double mouth_f, int mode)
{
	// CHANGE ROLL PITCH YAW MOUTH VALUES TO DXL POSITIONS

	//-------- change roll pitch yaw angle to string length L1 L2 L3
	//--------

	double yaw_degree = yaw_f * 180 / PI + 0.05;	// yaw
	float gamma = (float)(roll_f);					// roll
	float beta = (float)(pitch_f);					// pitch 

	MatrixXd T_X_gamma(4, 4), T_Y_beta(4, 4), T(4, 4);
	T_X_gamma << 1, 0, 0, 0, 0, cos(gamma), -sin(gamma), 0, 0, sin(gamma), cos(gamma), 0, 0, 0, 0, 1;	// x축을 기준으로 gamma 회전, roll
	T_Y_beta << cos(beta), 0, sin(beta), 0, 0, 1, 0, 0, -sin(beta), 0, cos(beta), 0, 0, 0, 0, 1;		// y축을 기준으로 beta 회전, pitch
	T = T_Y_beta * T_X_gamma;																			// 평행이동, roll, pitch 변환하는 행렬 

	MatrixXd Z_p(3, 1), Z_p_ext(4, 1), Z_n(4, 1), vec(4, 1);
	Z_p << 0, 0, 1;											//바닥면의 normal vector, z방향 유닛백터 
	Z_p_ext << 0, 0, 1, 1;
	vec << 0, 0, 0, 1;
	Z_n = T.inverse() * (Z_p_ext - vec);					//변환면 roll-pitch 후에 normal vector 
	double n1 = Z_n(0, 0), n2 = Z_n(1, 0), n3 = Z_n(2, 0);	//n1x + n2y + n3z + d = 0; 의 평면방정식을 가정 

	MatrixXd normalvec(1, 3);
	normalvec << n1, n2, n3;
	double theta = acos( (normalvec * Z_p).value() / (normalvec.norm() * Z_p.norm()) );
	double alpha = atan2(n2, n1);

	if (theta <= 0.00001) // theta 가 0이 되는 상황을 방지, 계산상으로 아주 작은 값을 넣어줌 
		theta = 0.001;

	double r = ROBOT_HEIGHT / theta;		// 문제, theta 가 0에 가까울수록, r이 커짐....
	double r_xy = r * cos(theta);

	double rcm_x = r * cos(alpha);			//  원격회전을 위한 중심
	double rcm_y = r * sin(alpha);

	double rx = rcm_x - r_xy * cos(alpha);	// 이동한 윗면의 중심 
	double ry = rcm_y - r_xy * sin(alpha);
	double rz = r * sin(theta);

	// (2) roll-pitch 가 될 때, 3개 실구멍의 위치 구하기
	double a = rx;	// roll-pitch에 의해 이동한 위치 x
	double b = ry;	// roll-pitch에 의해 이동한 위치 y
	double c = rz;	// roll-pitch에 의해 이동한 위치 z

	// 바닥면의 구멍과 방향백터 정의
	MatrixXd P1(3, 1), P2(3, 1), P3(3, 1);
	//, Q1(4, 1), Q2(4, 1), Q3(4, 1), E1(3, 1), E2(3, 1), E3(3, 1), abc(3, 1), P11(4, 1), P21(4, 1), P31(4, 1);
	P1 << ROBOT_STRING_HOLE_RADIUS * cos(0),		  ROBOT_STRING_HOLE_RADIUS* sin(0),			 0; // 1번 구멍의 바닥위치
	P2 << ROBOT_STRING_HOLE_RADIUS * cos(2 * PI / 3), ROBOT_STRING_HOLE_RADIUS* sin(2 * PI / 3), 0; // 2번 구멍의 바닥위치
	P3 << ROBOT_STRING_HOLE_RADIUS * cos(4 * PI / 3), ROBOT_STRING_HOLE_RADIUS* sin(4 * PI / 3), 0; // 3번 구멍의 바닥위치
	//P11 << P1, 1; //padding
	//P21 << P2, 1;
	//P31 << P3, 1;
	//abc << a, b, c;
	//Q1 << (T.inverse() * P11);
	//E1 << (Q1.block(0, 0, 3, 1) + abc);
	//Q2 << (T.inverse() * P21);
	//E2 << (Q2.block(0, 0, 3, 1) + abc);
	//Q3 << (T.inverse() * P31);
	//E3 << (Q3.block(0, 0, 3, 1) + abc);

	//실의 길이
	MatrixXd R(1, 2), u_rcm(1, 2), P1R(1, 2), P2R(1, 2), P3R(1, 2), P1R_1(1, 2), P2R_1(1, 2), P3R_1(1, 2);
	R << rcm_x, rcm_y;

	u_rcm << cos(alpha), sin(alpha);
	P1R_1 << P1(0), P1(1);
	P2R_1 << P2(0), P2(1);
	P3R_1 << P3(0), P3(1);

	P1R << (P1R_1 - R);
	P2R << (P2R_1 - R);
	P3R << (P3R_1 - R);

	double r1 = (P1R * u_rcm.transpose()).value();
	double r2 = (P2R * u_rcm.transpose()).value();
	double r3 = (P3R * u_rcm.transpose()).value();

	double L1 = (ROBOT_LAYERS * abs(r1) * (theta / ROBOT_LAYERS)); //앞쪽(DXL#1)
	double L2 = (ROBOT_LAYERS * abs(r2) * (theta / ROBOT_LAYERS)); //오른쪽(관찰자 시점//DXL#2)
	double L3 = (ROBOT_LAYERS * abs(r3) * (theta / ROBOT_LAYERS)); //왼쪽(관찰자 시점//DXL#3)

	//--------- string length to DXL position
	//---------

	double dxl_goal_position_pitch_double, dxl_goal_position_rollr_double, dxl_goal_position_rolll_double, dxl_goal_position_yaw_double, dxl_goal_position_mouth_double;
		
	if (mode == 0) // mirroring
	{
		double pitch_diff = (L1 - ROBOT_HEIGHT) * (4096 / (p_d * PI));

		// pitch up -> pitch tension too loose
		if (pitch_diff < 0)
			pitch_diff *= 0.5;

		dxl_goal_position_pitch_double = default_PITCH - pitch_diff;

		//dxl_goal_position_pitch_double = default_PITCH - (L1 - ROBOT_HEIGHT) * (4096 / (p_d * PI));

		double delta_mouth = (float)mouth_f * ROBOT_MOUTH_TUNE; // 입 크기180

		dxl_goal_position_rollr_double = default_ROLLR - (L2 - ROBOT_HEIGHT) * (4096 / (p_d * PI)) - (delta_mouth / ROBOT_MOUTH_BACK_COMPENSATION);			
		dxl_goal_position_rolll_double = default_ROLLL - (L3 - ROBOT_HEIGHT) * (4096 / (p_d * PI)) - (delta_mouth / ROBOT_MOUTH_BACK_COMPENSATION);
		
		//// R, L이 너무 기울어졌을 때 입 보상 끄는거
		//if (dxl_goal_position_rollr_double < 200)
		//	dxl_goal_position_rollr_double += (delta_mouth / 1.5);
		//if (dxl_goal_position_rolll_double < 200)
		//	dxl_goal_position_rolll_double += (delta_mouth / 1.5);
		
		dxl_goal_position_mouth_double = default_MOUTH + (-1) * delta_mouth + (-1) * (default_PITCH - dxl_goal_position_pitch_double) / ROBOT_MOUTH_PITCH_COMPENSATION;
		dxl_goal_position_yaw_double = (-1) * static_cast<double> (yaw_degree) * ROBOT_YAW_GEAR_RATIO * 4096.0 / 360.0 + default_YAW;
	}
	else if (mode == 1) // cloning
	{
		dxl_goal_position_pitch_double = default_PITCH - (L1 - ROBOT_HEIGHT) * (4096 / (p_d * PI));
		double delta_mouth = (float)mouth_f * ROBOT_MOUTH_TUNE;

		// R L change
		dxl_goal_position_rolll_double = default_ROLLR - (L2 - ROBOT_HEIGHT) * (4096 / (p_d * PI)) - (delta_mouth / ROBOT_MOUTH_BACK_COMPENSATION);
		dxl_goal_position_rollr_double = default_ROLLL - (L3 - ROBOT_HEIGHT) * (4096 / (p_d * PI)) - (delta_mouth / ROBOT_MOUTH_BACK_COMPENSATION);

		dxl_goal_position_mouth_double = default_MOUTH + (-1) * delta_mouth + (-1) * (default_PITCH - dxl_goal_position_pitch_double) / ROBOT_MOUTH_PITCH_COMPENSATION;

		// yaw_degree reverse
		dxl_goal_position_yaw_double = static_cast<double> (yaw_degree) * ROBOT_YAW_GEAR_RATIO * 4096.0 / 360.0 + default_YAW;
	}

	vector<int> DXL(5);
	DXL[0] = (int)dxl_goal_position_pitch_double;
	DXL[1] = (int)dxl_goal_position_rollr_double;
	DXL[2] = (int)dxl_goal_position_rolll_double;
	DXL[3] = (int)dxl_goal_position_yaw_double;
	DXL[4] = (int)dxl_goal_position_mouth_double;

	return DXL;
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
		//cout << "roll  = " << roll << endl;
		//cout << "pitch = " << pitch << endl;
		//cout << "yaw   = " << yaw << endl;		
	}

	vector<bool> output(3);
	output[0] = hitKeyboard;
	output[1] = goHome;
	output[2] = exit;

	return output;
}