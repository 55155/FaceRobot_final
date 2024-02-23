
// FACEROBOT CSV READ

#include "MacrosAndFunctions_JA_0801.h"


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


	//--- MOTOR PARAMETWRS
	int dxl_comm_result = COMM_TX_FAIL;					// Communication result
	int dxl_goal_position[5] = { 0,0,0,0,0 };
	uint8_t dxl_error = 0;								// Dynamixel error
	int DXL_ID[5] = { DXL1_ID, DXL2_ID, DXL3_ID, DXL4_ID, DXL5_ID };

	//--- FILE WRITE PARAMETERS
	ofstream outData_position, outData_current;
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

	
	bool flag_breakWhile = false;
	//Playlist
	while (true) {

		vector<string> PlayList_WAV;
		PlayList_WAV.push_back("ThatThingYouDo");
		//PlayList_WAV.push_back("이세상살아가다보면");
		//PlayList_WAV.push_back("StandByYourMan");
		//PlayList_WAV.push_back("알수없는인생");
		//PlayList_WAV.push_back("고백");
		//PlayList_WAV.push_back("신호등");
		//PlayList_WAV.push_back("christmastree1min");
		//PlayList_WAV.push_back("tiny_riot");
		//PlayList_WAV.push_back("봄이좋냐");
		//PlayList_WAV.push_back("거꾸로강을거슬러오르는저힘찬연어들처럼-강산에");
		//PlayList_WAV.push_back("우리의꿈");		
		//PlayList_WAV.push_back("라젠카");
		//PlayList_WAV.push_back("butter");
		//PlayList_WAV.push_back("dynamite_acapella");
		//PlayList_WAV.push_back("사라진모든것들에게");
		//PlayList_WAV.push_back("부럽지가않아");
		//PlayList_WAV.push_back("과제곡");
		//PlayList_WAV.push_back("나에게로떠나는여행");
		//PlayList_WAV.push_back("중독된사랑");
		//PlayList_WAV.push_back("사랑인가봐음원");
		//PlayList_WAV.push_back("취기를빌려");
		//PlayList_WAV.push_back("폰서트");
		//PlayList_WAV.push_back("취중고백");

		for (int i = 0; i < PlayList_WAV.size(); i++) {
			cout << PlayList_WAV[i] << "        Playlist number " << i + 1 << "/" << PlayList_WAV.size() << endl;
			if (!letSing(packetHandler, portHandler, PlayList_WAV[i])) {
				flag_breakWhile = true;
				break;
			}
		}
		if (flag_breakWhile)
			break;
	}

	groupSyncWriteVelocity.clearParam();

	//----- HOMING MODE
	setDXLGoalPosition(dxl_goal_position, END_PITCH, END_ROLLR, END_ROLLL, END_YAW, END_MOUTH);
	if (!moveDXLtoDesiredPosition(portHandler, packetHandler, dxl_goal_position, DXL_PROFILE_VELOCITY_HOMING)) { return 0; }

	Sleep(3000);

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