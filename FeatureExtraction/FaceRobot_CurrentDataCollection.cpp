/*
* made by KM 2022.06.14.
*/

// FACEROBOT KEYBOARD CONTROLLER

#include "MacrosAndFunctions.h"  
#include "FaceRobot_ContactHandling.h"

int main()
{
	ContactHandler CH = ContactHandler();

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
	int16_t dxl_present_current_filtered[5] = { 0,0,0,0,0 };
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
		outData_position.open("_CSV/Contact/"+year + month + date + ";" + hour + "-" + min + "-" + second + " pos.csv", ios::app);
	//if (FLAG_SAVE_DXL_PRESENT_CURRENT)
	//	outData_current.open("_CSV/Contact/" + year + month + date + ";" + hour + "-" + min + "-" + second + " current" + ".csv", ios::app);


	
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

	int mode = 0; // 0: mirroring, 1: cloning

	double roll = 0, pitch = 0, yaw = 0, mouth = 0;

	int n_roll = 9, n_pitch = 9, n_yaw = 9; // odd number	

	double delta_roll = MAX_ROLL / ((n_roll - 1) / 2.0);
	double delta_pitch = MAX_PITCH / ((n_pitch - 1) / 2.0);
	double delta_yaw = MAX_YAW / ((n_yaw - 1) / 2.0);

	vector<double> roll_set, pitch_set, yaw_set;
	
	// Training Data 1: 3,3,5 -> 01234
	//for (int i = 0; i < n_roll; i++)
	//	roll_set.push_back(-MAX_ROLL + i * delta_roll);
	//for (int i = 0; i < n_pitch; i++)
	//	pitch_set.push_back(-MAX_PITCH + i * delta_pitch);
	//for (int i = 0; i < n_yaw; i++)
	//	yaw_set.push_back(-MAX_YAW + i * delta_yaw);

	// Training Data 2: 3,3,9 -> 1357
	//for (int i = 1; i < n_roll; i = i + 2)
	//	roll_set.push_back(-MAX_ROLL + i * delta_roll);
	//for (int i = 1; i < n_pitch; i = i + 2)
	//	pitch_set.push_back(-MAX_PITCH + i * delta_pitch);
	//for (int i = 1; i < n_yaw; i = i + 2)
	//	yaw_set.push_back(-MAX_YAW + i * delta_yaw);

	// Validation Data
	roll_set.push_back(-MAX_ROLL + 1.5 * delta_roll);
	roll_set.push_back(-MAX_ROLL + 4 * delta_roll);
	roll_set.push_back(-MAX_ROLL + 6.5 * delta_roll);
	pitch_set.push_back(-MAX_PITCH + 1.5 * delta_pitch);
	pitch_set.push_back(-MAX_PITCH + 4 * delta_pitch);
	pitch_set.push_back(-MAX_PITCH + 6.5 * delta_pitch);
	yaw_set.push_back(-MAX_YAW + 1.5 * delta_yaw);
	yaw_set.push_back(-MAX_YAW + 4 * delta_yaw);
	yaw_set.push_back(-MAX_YAW + 6.5 * delta_yaw);

	vector<vector<double>> total_configs; // size: n_roll * n_pitch * n_yaw
	vector<double> config(3);
	for (int i = 0; i < yaw_set.size(); i++) {
		for (int j = 0; j < pitch_set.size(); j++) {
			for (int k = 0; k < roll_set.size(); k++) {
				config[0] = roll_set[k];
				config[1] = pitch_set[j];
				config[2] = yaw_set[i];
			
				//config[0] = -MAX_ROLL + delta_roll * (n_roll - 1) / 2;		// home
				//config[1] = -MAX_PITCH + delta_pitch * (n_pitch - 1) / 2;		// home
				//config[2] = -MAX_YAW + delta_yaw * (n_yaw - 1) / 2;			// home
			
				total_configs.push_back(config);
			}
		}
	}
	cout << "Total configurations: " << total_configs.size() << endl;

	//char stop;
	bool toNextConfig = false, pause = false, terminate = false, repeatPresentConfig = false, needCaution = false, secondContact = false;
	int iter = 0, max_iter = total_configs.size();

	auto dynamixel_keyboard_control = [&]()
	{
		INFO_STREAM("dynamixel_control_start");

		INFO_STREAM("Press 's' to start");
		INFO_STREAM("'n': next configuration");
		INFO_STREAM("'s': second contact for current configuration");
		INFO_STREAM("'p': pause data collection");
		INFO_STREAM("'r': reset current configuration");
		INFO_STREAM("'c': record caution to data");
		while (true) {
			if (_kbhit()) {
				char c = _getch();
				if (c == 's' || c == 'S')
					break;
			}
		}
		INFO_STREAM("------------------------ COLLECTING DATA...");

		bool firstRun = true;
		long long startTime = 0, millisec_since_epoch1 = 0;
		int itit = 1;

		bool goback = false;

		while (true)
		{
			//// for collecting contact-free data
			//long long present_time = millisec_since_epoch1 - startTime;
			//int changetime = 9500;
			//cout << present_time << endl;
			//if (present_time / changetime == itit)
			//{
			//	if (iter >= max_iter) {
			//		INFO_STREAM("All configurations done.");
			//		terminate = true;
			//	}
			//	else {
			//		toNextConfig = true;
			//		roll = total_configs[iter][0];
			//		pitch = total_configs[iter][1];
			//		yaw = total_configs[iter][2];

			//		cout << "roll  = " << roll << endl;
			//		cout << "pitch = " << pitch << endl;
			//		cout << "yaw   = " << yaw << endl;
			//	}
			//	iter++;
			//	itit++;
			//}

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

						cout << "iter: " << iter << endl;
						cout << "roll  = " << roll << endl;
						cout << "pitch = " << pitch << endl;
						cout << "yaw   = " << yaw << endl;
					}
					iter++;
				}
				if (c == 'p' || c == 'P')
					pause = true;
				if (c == 'r' || c == 'R') {
					repeatPresentConfig = true;

					cout << "roll  = " << roll << endl;
					cout << "pitch = " << pitch << endl;
					cout << "yaw   = " << yaw << endl;
				}
				if (c == 'c' || c == 'C') {
					INFO_STREAM("Need caution.");
					INFO_STREAM("------------------------ COLLECTING DATA...");
					needCaution = true;
				}
				if (c == 's' || c == 'S') {
					INFO_STREAM("Second contact.");
					INFO_STREAM("------------------------ COLLECTING DATA...");
					secondContact = true;
				}

				if (c == 'b' || c == 'B') {
					cout << "go back" << endl;
					goback = true;
				}
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
							needCaution = true;
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

				if (!moveDXLtoDesiredPosition(groupSyncWriteVelocity, groupSyncWritePosition, DXL_PROFILE_VELOCITY_CONFIGCHANGE, dxl_goal_position)) return 0;
				Sleep(3500);

				cout << "MOVED TO NEXT CONFIGURATION (" << iter << "/" << max_iter << ")" << endl;
				INFO_STREAM("------------------------ COLLECTING DATA...");

			}
			else {
				if (repeatPresentConfig) {
					INFO_STREAM("Repeat present configuration...");

					double prev_roll = 0.0, prev_pitch = 0.0, prev_yaw = 0.0;
					if (iter > 1) {
						prev_roll = total_configs[iter - 2][0];
						prev_pitch = total_configs[iter - 2][1];
						prev_yaw = total_configs[iter - 2][2];
					}
					vector<int> DXL_stepbefore = RPY2DXL(prev_roll, prev_pitch, prev_yaw, mouth, mode);
					int dxl_goal_position_stepbefore[5] = { 0,0,0,0,0 };
					setDXLGoalPosition(dxl_goal_position_stepbefore, DXL_stepbefore[0], DXL_stepbefore[1], DXL_stepbefore[2], DXL_stepbefore[3], DXL_stepbefore[4]);

					if (!moveDXLtoDesiredPosition(groupSyncWriteVelocity, groupSyncWritePosition, DXL_PROFILE_VELOCITY_CONFIGCHANGE, dxl_goal_position_stepbefore)) return 0;
					Sleep(1500);
					
					if (!moveDXLtoDesiredPosition(groupSyncWriteVelocity, groupSyncWritePosition, DXL_PROFILE_VELOCITY_CONFIGCHANGE, dxl_goal_position)) return 0;
					Sleep(2500);

					INFO_STREAM("------------------------ COLLECTING DATA...");
				}
				else
					if (!moveDXLtoDesiredPosition_NoVelLimit(packetHandler, groupSyncWritePosition, dxl_goal_position)) return 0;
			}

			if (firstRun) {
				startTime = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
				millisec_since_epoch1 = startTime;
				firstRun = false;
			}
			else
				millisec_since_epoch1 = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();

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

				//cout << "Pitch: " << dxl_goal_position[0] << "," << dxl_present_position[0] << endl;
				//cout << "Right: " << dxl_goal_position[1] << "," << dxl_present_position[1] << endl;
				//cout << "Left : " << dxl_goal_position[2] << "," << dxl_present_position[2] << endl;

				outData_position << millisec_since_epoch1 - startTime << "," << dxl_goal_position[0] << "," << dxl_present_position[0]
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

				MeanFiltering_Current(dxl_present_current, dxl_present_current_filtered, toNextConfig);
				int ConfigStatus = 0;
				if (toNextConfig)
					ConfigStatus = 111;
				if (secondContact)
					ConfigStatus = 222;
				if(repeatPresentConfig)
					ConfigStatus = 333;
				if (needCaution)
					ConfigStatus = -111;

				//// motor ¼ø¼­: pitch - right - left - yaw - mouth
				//outData_current << millisec_since_epoch1 - startTime << "," << ConfigStatus << ","
				//	<< dxl_present_current[0] << "," << dxl_present_current[1] << "," << dxl_present_current[2] << "," << dxl_present_current[3] << "," << dxl_present_current[4] << ","					
				//	<< roll << "," << pitch << "," << yaw
				//	<< endl;
								
				if (ConfigStatus != 0) {
					CH.resetContactHandler();
					cout << "CH reset" << endl;
				}					

				vector<int> presentCurrent(4);
				for (int m = 0; m < 4; m++) {
					presentCurrent[m] = dxl_present_current[m];
				}					
				int contact_class = CH.getContactResult(presentCurrent);
				cout << "contact_class = " << contact_class << endl;

				if (contact_class == 1)
					cout << "right contact detected" << endl;
				else if (contact_class == 2)
					cout << "left contact detected" << endl;
				else if (contact_class == 3)
					cout << "back contact detected" << endl;
				else if (contact_class == 4)
					cout << "front contact detected" << endl;
				else if (contact_class == 5)
					cout << "rotate to right detected" << endl;
				else if (contact_class == 6)
					cout << "rotate to left detected" << endl;
				else if (contact_class == -2)
					cout << "unknown contact detected" << endl;
				//else if (contact_class == -1)
				//	cout << "figuring out what contact this is" << endl;
				//else
				//	cout << "no contact" << endl;

				//if (CH.isContactDetectedAndClassified()) {
				//	CH.resetContactHandler();
				//}
				 
				ContactControl(contact_class, roll, pitch, yaw, mouth, goback, true, mode, groupSyncWriteVelocity, groupSyncWritePosition, DXL_PROFILE_VELOCITY_CONFIGCHANGE);
			}
			toNextConfig = false;
			repeatPresentConfig = false;
			needCaution = false;
			secondContact = false;
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