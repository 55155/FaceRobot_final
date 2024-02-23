/*
* made by KM 2022.06.14.
*/

// FACEROBOT CONTACT DATASET GENERATION

//#include "MacrosAndFunctions.h"
#include "MacrosAndFunctions_small.h"

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
		outData_position.open("_CSV/Keyboard/"+year + month + date + ";" + hour + "-" + min + "-" + second + " pos.csv", ios::app);
	if (FLAG_SAVE_DXL_PRESENT_CURRENT)
		outData_current.open("_CSV/Keyboard/" + year + month + date + ";" + hour + "-" + min + "-" + second + " current" + ".csv", ios::app);


	
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

	INFO_STREAM("Starting control");

	double roll = 0, pitch = 0, yaw = 0, mouth = 0;
	char stop;

	auto dynamixel_keyboard_control = [&]()
	{
		INFO_STREAM("dynamixel_control_start");

		bool firstRun = true;
		long long startTime, millisec_since_epoch1;
		while (true)
		{
			int mode = 0; // mirroring
			//int mode = 1; // cloning 			
						
			//VectorXd RPYE = GazeTest();	
			//bool exit = RPYE(3);
			//bool goHome = false;
			//roll = RPYE(0);
			//pitch = RPYE(1);
			//yaw = RPYE(2);

			vector<bool> out = KeyBoardControl(roll, pitch, yaw, mouth, true);
			bool goHome = out[1];
			bool exit = out[2];

			//cout << "roll (deg)  = " << RPYE(0) / PI * 180 << endl;
			//cout << "pitch (deg) = " << RPYE(1) / PI * 180 << endl;
			//cout << "yaw (deg)   = " << RPYE(2) / PI * 180 << endl;

			vector<int> DXL = TRO_RPY2DXL(roll, pitch, yaw, mouth, 0);

			setDXLGoalPosition(dxl_goal_position, DXL[0], DXL[1], DXL[2], DXL[3], DXL[4]);

			if (goHome) {
				if (!moveDXLtoDesiredPosition(groupSyncWriteVelocity, groupSyncWritePosition, DXL_PROFILE_VELOCITY_HOMING, dxl_goal_position)) return 0;
				Sleep(2000);
				goHome = false;
				cout << "Moved to home position" << endl;
			}
			else {
				if (!moveDXLtoDesiredPosition_NoVelLimit(packetHandler, groupSyncWritePosition, dxl_goal_position)) return 0;
			}

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

				/*					cout << "Pitch: " << dxl_goal_position[0] << "," << dxl_present_position[0] << endl;
									cout << "Right: " << dxl_goal_position[1] << "," << dxl_present_position[1] << endl;
									cout << "Left : " << dxl_goal_position[2] << "," << dxl_present_position[2] << endl;*/

				outData_position << millisec_since_epoch1 - startTime << "," << dxl_goal_position[0] << "," << dxl_present_position[0]
					<< "," << dxl_goal_position[1] << "," << dxl_present_position[1]
					<< "," << dxl_goal_position[2] << "," << dxl_present_position[2]
					<< "," << dxl_goal_position[3] << "," << dxl_present_position[3] << "," << dxl_goal_position[4] << "," << dxl_present_position[4]
					<< endl;
			}

			if (firstRun) {
				startTime = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
				millisec_since_epoch1 = startTime;
				firstRun = false;
			}
			else
				millisec_since_epoch1 = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();

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

				outData_current << millisec_since_epoch1 - startTime << "," << dxl_present_current[0] << "," << dxl_present_current[1]
					<< "," << dxl_present_current[2] << "," << dxl_present_current[3] << "," << dxl_present_current[4] << endl;
			}

			if (exit)
				stop = 'q';
			if (stop == 'q') {
				INFO_STREAM("dynamixel_control break");
				break;
			}
		}
	};

	std::thread t1 = std::thread(dynamixel_keyboard_control);
	t1.join();

	INFO_STREAM("Closed successfully");

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