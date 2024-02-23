
// FACEROBOT HEAD MOTION GENERATION

//#include "MacrosAndFunctions.h"
#include "MacrosAndFunctions_small.h"

vector<bool> getPathsFromArguments(string& folderpath, string& audioname, vector<string>& arguments, string& audiofilepath, string& Headcsvfilepath, string& Mouthcsvfilepath, string& originalMusicPath)
{
	vector<bool> isAudioHeadMouth = { false,false,false }; // audio wav, head csv, mouth csv order

	for (int i = 0; i < arguments.size() - 1; i++) {
		if (arguments[i] == "-f") {
			folderpath = arguments[i + 1];
			break;
		}
	}

	for (int i = 0; i < arguments.size(); i++) {
		if (arguments[i] == "-org") {
			originalMusicPath = arguments[i + 1];
			break;
		}
	}

	for (int i = 0; i < arguments.size() - 1; i++) {
		if (arguments[i] == "-a") {
			isAudioHeadMouth[0] = true;
			audioname = arguments[i + 1];
			audiofilepath = folderpath + "audio\\" + audioname + ".wav";
		}
	}
	Headcsvfilepath = folderpath + "csv\\" + audioname + ".csv";
	Mouthcsvfilepath = folderpath + "csv\\" + audioname + "-delta-big.csv";

	for (int i = 0; i < arguments.size() - 1; i++) {
		if (arguments[i] == "-h" && arguments[i + 1] == "notnew") {
			isAudioHeadMouth[1] = true;
			Headcsvfilepath = folderpath + "csv\\" + audioname + ".csv";
		}			
		if (arguments[i] == "-m" && arguments[i + 1] == "notnew") {
			isAudioHeadMouth[2] = true;
			Mouthcsvfilepath = folderpath + "csv\\" + audioname + "-delta-big.csv";
		}			
	}

	if (!isAudioHeadMouth[0]) 
		cout << "---------------- ERROR: NO AUDIO FILE" << endl;		
	else
		cout << "---------------- Audio wav file path = " << audiofilepath << endl;
	
	return isAudioHeadMouth;
}

int main(int argc, char** argv)
{

	string folderpath = "D:\\_data\\headmotiondata\\datasementation\\";
	//string folderpath = "D:\\headmotiongeneration\\";
	//string folderpath = "_motion\\";

	vector<string> arguments;
	for (int i = 0; i < argc; ++i)
		arguments.push_back(std::string(argv[i]));		
	
	string audioname = "NONE";
	string audiofilepath = "NONE", Headcsvfilepath = "NONE", Mouthcsvfilepath = "NONE";
	string segmentfolderpath = folderpath + "segments\\";
	string originalMusicPath = "NONE";

	cout << Mouthcsvfilepath << endl;

	vector<bool> isAudioHeadMouth = getPathsFromArguments(folderpath, audioname, arguments, audiofilepath, Headcsvfilepath, Mouthcsvfilepath, originalMusicPath);

	if (!isAudioHeadMouth[0])
		return 0;

	// Dynamixel setting (start): Initialize PortHandler instance, Set the port path, Get methods and members of PortHandlerLinux or PortHandlerWindows
	// Initialize PacketHandler instance: Set the protocol version, Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
	// Initialize GroupSyncWrite instance (cpp to dxl)
	dynamixel::PortHandler* portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
	dynamixel::PacketHandler* packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
	dynamixel::GroupSyncWrite groupSyncWritePosition(portHandler, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION);		// POSITION
	dynamixel::GroupSyncWrite groupSyncWriteVelocity(portHandler, packetHandler, ADDR_PRO_PROFILE_VELOCITY, LEN_PRO_PROFILE_VELOCITY);	// VELOCITY

	//--- MOTOR PARAMETWRS
	int dxl_comm_result = COMM_TX_FAIL;					// Communication result
	int dxl_goal_position[5] = { 0,0,0,0,0 };
	int32_t dxl_present_position[5] = { 0,0,0,0,0 };
	int16_t dxl_present_current[5] = { 0,0,0,0,0 };
	uint8_t dxl_error = 0;								// Dynamixel error
	int DXL_ID[5] = { DXL1_ID, DXL2_ID, DXL3_ID, DXL4_ID, DXL5_ID };

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

	std::chrono::high_resolution_clock::time_point initiateTime;
	initiateTime = std::chrono::high_resolution_clock::now();
	auto awaketime = initiateTime + 5000ms;

	//----- HOMING MODE
	setDXLGoalPosition(dxl_goal_position, default_PITCH, default_ROLLR, default_ROLLL, default_YAW, default_MOUTH);
	if (!moveDXLtoDesiredPosition(groupSyncWriteVelocity, groupSyncWritePosition, DXL_PROFILE_VELOCITY_HOMING, dxl_goal_position)) { return 0; }

	if (!isAudioHeadMouth[1]) {
		string command = "py makeHeadMotion.py " + audiofilepath + " " + Headcsvfilepath + " " + segmentfolderpath;
		system(command.c_str());
	}
	if (!isAudioHeadMouth[2]) {
		string command = "py makeMouthMotion.py " + audiofilepath + " " + Mouthcsvfilepath;
		system(command.c_str());
	}
	
	while (std::chrono::high_resolution_clock::now() < awaketime) { std::this_thread::yield(); }

	INFO_STREAM("Starting tracking (moving)");

	double roll_f, pitch_f, yaw_f, mouth_f; // filtered roll,pitch,yaw,mouth values


	// ------------------------------------------------- no thread
	INFO_STREAM("dynamixel_control_start");

	std::ifstream Head_in, Mouth_in;
	Head_in.open(Headcsvfilepath);
	Mouth_in.open(Mouthcsvfilepath);
	if (Head_in.fail())
		return(cout << "Head motion file not found" << endl) && 0;
	if (Mouth_in.fail())
		return(cout << "Mouth motion file not found" << endl) && 0;

	int iter = 0;
	bool firstRun = true;
	std::chrono::high_resolution_clock::time_point startTime;



	//// GAZE TEST
	int face_x = 400;	// mm
	int face_y = 0;		// mm
	int face_z = 80;	// mm
	double gaze_r = 0, gaze_p = 0, gaze_y = 0;

	std::ifstream Mouth_findMax;	
	Mouth_findMax.open(Mouthcsvfilepath);
	double max_mouth = 0;
	while (Mouth_findMax.good()) {
		std::vector<std::string> row_mouth = csv_read_row(Mouth_findMax, ',');
		double tmp_mouth = stof(row_mouth[0]);
		if (max_mouth < tmp_mouth)
			max_mouth = tmp_mouth;
	}
	cout << "max_mouth = " << max_mouth << endl;

	while (Head_in.good() && Mouth_in.good())
	{
		//if (PauseQuitNext() == 1)
		//	break;

		std::vector<std::string> row_head = csv_read_row(Head_in, ',');
		std::vector<std::string> row_mouth = csv_read_row(Mouth_in, ',');

		roll_f = stof(row_head[0]);
		pitch_f = stof(row_head[1]);
		yaw_f = stof(row_head[2]);
		//mouth_f = 0.0;
		mouth_f = stof(row_mouth[0]) / max_mouth; // python에서 이미 곱해져서 나옴 // 0~1로 변환

		if (_kbhit()) {
			char c = _getch();
			if (c == 'q' || c == 'Q') {
				break;
			}
		}

		////// GAZE TEST
		//bool hitKeyboard = false;		
		//if (_kbhit()) {

		//	hitKeyboard = true;

		//	char c = _getch();

		//	if (c == 'h' || c == 'H') {
		//		face_x = 400; face_y = 0; face_z = 80;
		//	}				

		//	// y: right left
		//	if (c == 'a' || c == 'A')
		//		face_y -= 20;
		//	if (c == 'f' || c == 'F')
		//		face_y += 20;

		//	// z: up down
		//	if (c == 'e' || c == 'E') {
		//		if (face_z < 200)
		//			face_z += 40;
		//	}
		//	if (c == 'x' || c == 'X') {
		//		if (face_z > -200)
		//			face_z -= 40;
		//	}

		//	// x: front
		//	if (c == 'd' || c == 'D')
		//		face_x += 100;			
		//	if (c == 's' || c == 'S') {
		//		if (face_x > 400)
		//			face_x -= 100;
		//	}

		//	if (c == 'q' || c == 'Q') {
		//		cout << "Shut off" << endl;
		//		break;
		//	}

		//	cout << "x (front) = " << face_x << endl;
		//	cout << "y (right) = " << face_y << endl;
		//	cout << "z (up)    = " << face_z << endl;
		//}

		//VectorXd xyz(3);
		//xyz(0) = face_x; xyz(1) = face_y; xyz(2) = face_z;
		//VectorXd RPY = findGaze_RPY_Gamma(xyz);
		//gaze_r = RPY(0);
		//gaze_p = RPY(1);
		//gaze_y = RPY(2);

		////cout << "gaze_r = " << gaze_r << endl;
		////cout << "gaze_p = " << gaze_p << endl;
		////cout << "gaze_y = " << gaze_y << endl;

		//roll_f += gaze_r;
		//pitch_f += gaze_p;
		//yaw_f += gaze_y;


		int mode = 0; // 0: mirroring, 1: cloning
		vector<int> DXL = TRO_RPY2DXL(roll_f, pitch_f, yaw_f, mouth_f, mode);
		setDXLGoalPosition(dxl_goal_position, DXL[0], DXL[1], DXL[2], DXL[3], DXL[4]);

		if (firstRun) {
			moveDXLtoDesiredPosition(groupSyncWriteVelocity, groupSyncWritePosition, DXL_PROFILE_VELOCITY_HOMING, dxl_goal_position);
			std::this_thread::sleep_for(500ms);

			if (originalMusicPath == "NONE")
				playwav(audiofilepath);
			else
				playwav(originalMusicPath);
			startTime = std::chrono::high_resolution_clock::now();
			firstRun = false;
		}
		else {
			if (!moveDXLtoDesiredPosition_NoVelLimit(packetHandler, groupSyncWritePosition, dxl_goal_position)) break;
		}

		auto awaketime = startTime + iter * 40ms;
		while (std::chrono::high_resolution_clock::now() < awaketime) { std::this_thread::yield(); }

		iter++;
	}

	setDXLGoalPosition(dxl_goal_position, default_PITCH, default_ROLLR, default_ROLLL, default_YAW, default_MOUTH);
	if (!moveDXLtoDesiredPosition(groupSyncWriteVelocity, groupSyncWritePosition, DXL_PROFILE_VELOCITY_HOMING, dxl_goal_position)) { return 0; }

	std::this_thread::sleep_for(500ms);

	//----- ENDING MODE
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

	//// --------------------- implemented as a function

	//// Dynamixel setting (start): Initialize PortHandler instance, Set the port path, Get methods and members of PortHandlerLinux or PortHandlerWindows
	//// Initialize PacketHandler instance: Set the protocol version, Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
	//// Initialize GroupSyncWrite instance (cpp to dxl)
	//dynamixel::PortHandler* portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);	
	//dynamixel::PacketHandler* packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
	//dynamixel::GroupSyncWrite groupSyncWritePosition(portHandler, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION);		// POSITION
	//dynamixel::GroupSyncWrite groupSyncWriteVelocity(portHandler, packetHandler, ADDR_PRO_PROFILE_VELOCITY, LEN_PRO_PROFILE_VELOCITY);	// VELOCITY
	//	
	////--- MOTOR PARAMETWRS
	//int dxl_comm_result = COMM_TX_FAIL;					// Communication result
	//int dxl_goal_position[5] = { 0,0,0,0,0 };
	//int32_t dxl_present_position[5] = { 0,0,0,0,0 };
	//int16_t dxl_present_current[5] = { 0,0,0,0,0 };
	//uint8_t dxl_error = 0;								// Dynamixel error
	//int DXL_ID[5] = { DXL1_ID, DXL2_ID, DXL3_ID, DXL4_ID, DXL5_ID };

	//// Open port
	//if (portHandler->openPort())
	//	printf("Succeeded to open the port!\n");
	//else {
	//	printf("Failed to open the port!\n");
	//	printf("Press any key to terminate...\n");
	//	return 0;
	//}

	//// Set port baudrate
	//if (portHandler->setBaudRate(BAUDRATE))
	//	printf("Succeeded to change the baudrate!\n");
	//else {
	//	printf("Failed to change the baudrate!\n");
	//	printf("Press any key to terminate...\n");
	//	return 0;
	//}

	//// Enable Dynamixel Torque
	//for (int i = 0; i < 5; i++) {
	//	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID[i], ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
	//	if (dxl_comm_result != COMM_SUCCESS)
	//		printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
	//	else if (dxl_error != 0)
	//		printf("%s\n", packetHandler->getRxPacketError(dxl_error));
	//	else
	//		printf("Dynamixel#%d has been successfully connected \n", DXL_ID[i]);
	//}

	////----- HOMING MODE
	//setDXLGoalPosition(dxl_goal_position, default_PITCH, default_ROLLR, default_ROLLL, default_YAW, default_MOUTH);
	//if (!moveDXLtoDesiredPosition(groupSyncWriteVelocity, groupSyncWritePosition, DXL_PROFILE_VELOCITY_HOMING, dxl_goal_position)) { return 0; }

	//std::this_thread::sleep_for(5s);

	//INFO_STREAM("Starting tracking (moving)");

	//HeadMotion(packetHandler, portHandler, audiofilepath, Headcsvfilepath);
	//std::this_thread::sleep_for(2s);

	////----- HOMING MODE
	//setDXLGoalPosition(dxl_goal_position, END_PITCH, END_ROLLR, END_ROLLL, END_YAW, END_MOUTH);
	//if (!moveDXLtoDesiredPosition(groupSyncWriteVelocity, groupSyncWritePosition, DXL_PROFILE_VELOCITY_HOMING, dxl_goal_position)) { return 0; }

	//std::this_thread::sleep_for(4s);

	//// Disable Dynamixel Torque
	//for (int i = 0; i < 5; i++)
	//{
	//	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID[i], ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
	//	if (dxl_comm_result != COMM_SUCCESS)
	//		printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
	//	else if (dxl_error != 0)
	//		printf("%s\n", packetHandler->getRxPacketError(dxl_error));
	//	//here
	//}

	//// Close port
	//portHandler->closePort();
	//return 0;
}