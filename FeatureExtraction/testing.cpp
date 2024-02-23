#include "MacrosAndFunctions_v0.0.0.h"
#include "includePython.h"

thread t1([]() {
	int output = execute_AIspeaker_py();
	if (!output) write_robot_status("PYTHON_ERROR");
	});

int main() {


	// (1) Dynamixel setting (start): Initialize PortHandler instance, Set the port path, Get methods and members of PortHandlerLinux or PortHandlerWindows
	dynamixel::PortHandler* portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
	cout << "Enter main function scope,,,"<< endl;
	
	// Initialize PacketHandler instance: Set the protocol version, Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
	dynamixel::PacketHandler* packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
	
	// Initialize GroupSyncWrite instance (cpp to dxl)
	dynamixel::GroupSyncWrite groupSyncWritePosition(portHandler, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION);		// POSITION
	dynamixel::GroupSyncWrite groupSyncWriteVelocity(portHandler, packetHandler, ADDR_PRO_PROFILE_VELOCITY, LEN_PRO_PROFILE_VELOCITY);	// VELOCITY

	dynamixel::GroupSyncRead groupSyncReadPosition(portHandler, packetHandler, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);	// POSITION
	//--- MOTOR PARAMETWRS
	int dxl_comm_result = COMM_TX_FAIL;					// Communication result
	int dxl_goal_position[5] = { default_PITCH,default_ROLLR,default_ROLLL,default_YAW, default_MOUTH };
	uint8_t dxl_error = 0;								// Dynamixel error
	int DXL_ID[5] = { DXL1_ID, DXL2_ID, DXL3_ID, DXL4_ID, DXL5_ID };

	// Open port
	if (portHandler->openPort())
		printf("Succeeded to open the port!\n");
	else {
		printf("Failed to open the port!\n");
		printf("Press any key to terminate...\n");
		return FAIL_OPEN_PORT;
	}

	// Set port baudrate
	if (portHandler->setBaudRate(BAUDRATE))
		printf("Succeeded to change the baudrate!\n");
	else {
		printf("Failed to change the baudrate!\n");
		printf("Press any key to terminate...\n");
		return FAIL_SET_BAUD_RATE;
	}

	// Enable Dynamixel Torque
	for (int i = 0; i < 5; i++) {
		dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID[i], ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
		if (dxl_comm_result != COMM_SUCCESS) {
			printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
			return 0;
		}
		else if (dxl_error != 0) {
			printf("%s\n", packetHandler->getRxPacketError(dxl_error));
			return 0;
		}
		else
			printf("Dynamixel#%d has been successfully connected \n", DXL_ID[i]);
	}
	write_robot_status("start");
	bool quitprogram = false;
	bool flag_breakWhile = false;
	RobotStatus status = WAITING_FOR_USER_INPUT;

	/*
	t1.detach(); // main 에서 공유되어야될 변수가 없음.
	*/
	string file_path = "AIspeaker_JA\\";

	string target; // 읽는 것을 담당하는 부분
	while (true) {
		switch (status) {
		case QUIT_PROGRAM:
			cout << "\n\nExit" << endl;
			quitprogram = true;
			break;
		case WAITING_FOR_USER_INPUT:
			status = wait4processing(portHandler, packetHandler);
			break;
		case READY_TO_SPEAK:
			status = play_one_audio(portHandler, packetHandler, file_path, "AIspeaker_answer");
			target = read_robot_status();
			if (target != "SLEEP_MODE") write_robot_status("WAITING_FOR_USER_INPUT");
			break;
		}
		if (quitprogram) break;
		
	}
	
	

	// Go to the end position
	setDXLGoalPosition(dxl_goal_position, END_PITCH, END_ROLLR, END_ROLLL, END_YAW, END_MOUTH);
	if (!moveDXLtoDesiredPosition(portHandler, packetHandler, dxl_goal_position, DXL_PROFILE_VELOCITY_HOMING / 2)) {
		return FAIL_GO_TO_END_POSITION;
	}

	Sleep(2000);

	// Disable Dynamixel Torque
	for (int i = 0; i < DXL_NUM; i++)
	{
		dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID[i], ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
		if (dxl_comm_result != COMM_SUCCESS) {
			printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
			return FAIL_DISABLE_TORQUE;
		}
		else if (dxl_error != 0) {
			printf("%s\n", packetHandler->getRxPacketError(dxl_error));
			return FAIL_DISABLE_TORQUE;
		}
	}

	// Close port
	portHandler->closePort();

	return SUCCESS;

}