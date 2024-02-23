
#include "MacrosAndFunctions_small.h"

bool checkIter(int& iter, int maxIter) {

	bool okay = true;

	if (iter > maxIter) {
		cout << "END OF PLAYLIST" << endl;
		iter = maxIter;
		okay = false;
	}
	if (iter < 0) {
		cout << "START OF PLAYLIST" << endl;
		iter = 0;
		okay = false;
	}

	return okay;
}

int main()
{	
	dynamixel::PortHandler* portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
	dynamixel::PacketHandler* packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
	dynamixel::GroupSyncWrite groupSyncWritePosition(portHandler, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION);		// POSITION
	dynamixel::GroupSyncWrite groupSyncWriteVelocity(portHandler, packetHandler, ADDR_PRO_PROFILE_VELOCITY, LEN_PRO_PROFILE_VELOCITY);	// VELOCITY
	dynamixel::GroupSyncRead groupSyncReadPosition(portHandler, packetHandler, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);	// POSITION
	dynamixel::GroupSyncRead groupSyncReadCurrent(portHandler, packetHandler, ADDR_PRO_PRESENT_CURRENT, LEN_PRO_PRESENT_CURRENT);		// CURRENT
	int dxl_comm_result = COMM_TX_FAIL;					// Communication result
	int dxl_goal_position[5] = { 0,0,0,0,0 };
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

	// Add parameter storages for present position and current values
	for (int i = 0; i < 5; i++) {
		if (!groupSyncReadPosition.addParam(DXL_ID[i])) { fprintf(stderr, "[ID:%03d] groupSyncReadPosition addparam failed", DXL_ID[i]); return 0; }
		if (!groupSyncReadCurrent.addParam(DXL_ID[i])) { fprintf(stderr, "[ID:%03d] groupSyncReadCurrent addparam failed", DXL_ID[i]); return 0; }
	}

	//----- HOMING MODE
	setDXLGoalPosition(dxl_goal_position, default_PITCH, default_ROLLR, default_ROLLL, default_YAW, default_MOUTH);
	//if (!moveDXLtoDesiredPosition(portHandler, packetHandler, dxl_goal_position, DXL_PROFILE_VELOCITY_HOMING)) { return 0; }
	if (!moveDXLtoDesiredPosition(groupSyncWriteVelocity, groupSyncWritePosition, DXL_PROFILE_VELOCITY_HOMING, dxl_goal_position)) { return 0; }

	Sleep(2000);




	bool flag_breakWhile = false;

	vector<string> csvList;

	csvList.push_back("motion_tensioncheck");
	//csvList.push_back("motion_tensioncheck_moving");
	//csvList.push_back("motion_roll");
	//csvList.push_back("motion_pitch_plus");
	//csvList.push_back("motion_pitch_minus");
	//csvList.push_back("motion_yaw");
	//csvList.push_back("motion_rollM_pitchP");
	//csvList.push_back("motion_rollM_pitchM");

	for (int i = 0; i < csvList.size(); i++)
		cout << "Index: " << i << " - CSV: " << csvList[i] << endl;

	int iter = 0;
	int status = 2;
	bool quitprogram = false;
	while (true) {

		switch (status) {

		case 0: // quit all			
			cout << "Exit MC mode" << endl;
			quitprogram = true;
			break;

		case 1: // play one ment
			cout << "Playing ment index " << iter << endl;
			//status = kinematics_test(packetHandler, portHandler, csvList[iter]);
			//status = kinematics_test_presentPositionSave(packetHandler, portHandler, csvList[iter], groupSyncReadPosition, groupSyncReadCurrent);
			status = tensionCheck(packetHandler, portHandler, csvList[iter], groupSyncReadPosition, groupSyncReadCurrent);
			iter++;
			if (!checkIter(iter, csvList.size() - 1))
				status = 2;
			break;

		case 2: // wait for next command
			cout << "Waiting for next command... (next play iter = " << iter << ")" << endl;			
			status = waitfornextcommand(iter);			
			checkIter(iter, csvList.size() - 1);
			break;
		}

		if (quitprogram)
			break;
	}

	groupSyncWriteVelocity.clearParam();

	//----- HOMING MODE
	setDXLGoalPosition(dxl_goal_position, END_PITCH, END_ROLLR, END_ROLLL, END_YAW, END_MOUTH);
	//if (!moveDXLtoDesiredPosition(portHandler, packetHandler, dxl_goal_position, DXL_PROFILE_VELOCITY_HOMING)) { return 0; }
	if (!moveDXLtoDesiredPosition(groupSyncWriteVelocity, groupSyncWritePosition, DXL_PROFILE_VELOCITY_HOMING, dxl_goal_position)) { return 0; }

	Sleep(2000);

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