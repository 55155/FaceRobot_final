
// FACEROBOT CSV READ

#include "MacrosAndFunctions_small_JA.h"
#include "includePython.h"


static int status = 2;


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
	int dxl_goal_position[5] = { default_PITCH,default_ROLLR,default_ROLLL,default_YAW, default_MOUTH };
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
	
	
	write_data_sync("start");

	bool quitprogram = false;
	bool flag_breakWhile = false;

	auto excute_python = [&]() {
		int output = execute_AIspeaker_py();
		if (!output) write_data_sync("-2");
	};

	thread t1(excute_python);
	string file_path = "AIspeaker\\";


	while (true) {		
		switch (status) {

		case -1: // quit all			
			cout << "Exit" << endl;
			quitprogram = true;
			break;

		case 1: // play one ment
			status = MC_ment_km(portHandler, packetHandler, file_path + "AIspeaker_answer", false, true);
			if (read_data_sync() == -1) status = -1;
			else write_data_sync("0");
			break;

		case 2: // wait for next command
			status = wait4processing(portHandler, packetHandler);
			break;
		}
	
		if (quitprogram)
			break;
	}


	//----- HOMING MODE
	setDXLGoalPosition(dxl_goal_position, END_PITCH, END_ROLLR, END_ROLLL, END_YAW, END_MOUTH);
	if (!moveDXLtoDesiredPosition(portHandler, packetHandler, dxl_goal_position, DXL_PROFILE_VELOCITY_HOMING/2)) { return 0; }

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