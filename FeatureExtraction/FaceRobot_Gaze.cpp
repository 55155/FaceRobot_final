
// FACEROBOT WEBCAM GAZE

//#include "MacrosAndFunctions.h"    
#include "MacrosAndFunctions_v0.h"

// 대충 자로 잰 값 + 실제 값 보면서 calibration 
// initial configuration 고정해놓고 (Poc_y + OpenFace y) 값이 0 유지하는지 확인 등
#define Poc_x 155    -100								// 로봇 베이스 -> 카메라 중심 x 거리
#define Poc_y -190   +20								// 로봇 베이스 -> 카메라 중심 y 거리
#define Poc_z 140     -10 - 90							// 로봇 베이스 -> 카메라 중심 z 거리

#define FLAG_SAVE_FACEPOSITION				true		// 얼굴 위치 좌표값 저장할지 말지
#define FLAG_SAVE_RPY						true		// 로봇 pose RPY 저장할지 말지

#undef  FLAG_SAVE_DXL_PRESENT_POSITION
#define FLAG_SAVE_DXL_PRESENT_POSITION		false		// 모터 위치 저장할지 말지

int main(int argc, char** argv)
{
	std::vector<std::string> arguments;
	for (int i = 0; i < argc; ++i)
		arguments.push_back(std::string(argv[i]));

	// no arguments: output usage
	if (arguments.size() == 1) {
		std::cout << "For command line arguments see:" << std::endl;
		std::cout << " https://github.com/TadasBaltrusaitis/OpenFace/wiki/Command-line-arguments";
		return 0;
	}
		
	/////////// OpenFace -> 각각 자세한 설명은 FaceRobot_Webcam.cpp 참조; 안 건드려도 됨
	LandmarkDetector::FaceModelParameters det_parameters(arguments);	
	LandmarkDetector::CLNF face_model(det_parameters.model_location);
	if (!face_model.loaded_successfully) {
		std::cout << "ERROR: Could not load the landmark detector" << std::endl;
		return 1;
	}		
	FaceAnalysis::FaceAnalyserParameters face_analysis_params(arguments);	
	FaceAnalysis::FaceAnalyser face_analyser(face_analysis_params);
	if (!face_model.eye_model)
		std::cout << "WARNING: no eye model found" << std::endl;
	Utilities::SequenceCapture sequence_reader;	
	Utilities::Visualizer visualizer(arguments);
	Utilities::FpsTracker fps_tracker;
	if (face_analyser.GetAUClassNames().size() == 0 && face_analyser.GetAUClassNames().size() == 0)
		std::cout << "WARNING: no Action Unit models found" << std::endl;
	fps_tracker.AddFrame();

	/////////// Dynamixel -> 각각 자세한 설명은 FaceRobot_Webcam.cpp 참조; 안 건드려도 됨
	dynamixel::PortHandler* portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
	dynamixel::PacketHandler* packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
	dynamixel::GroupSyncWrite groupSyncWritePosition(portHandler, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION);		// POSITION
	dynamixel::GroupSyncWrite groupSyncWriteVelocity(portHandler, packetHandler, ADDR_PRO_PROFILE_VELOCITY, LEN_PRO_PROFILE_VELOCITY);	// VELOCITY
	dynamixel::GroupSyncRead groupSyncReadPosition(portHandler, packetHandler, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);	// POSITION
	dynamixel::GroupSyncRead groupSyncReadCurrent(portHandler, packetHandler, ADDR_PRO_PRESENT_CURRENT, LEN_PRO_PRESENT_CURRENT);		// CURRENT
	//--- MOTOR PARAMETWRS
	int dxl_comm_result = COMM_TX_FAIL;
	int dxl_goal_position[5] = { 0,0,0,0,0 };					// DXL 모터에 보낼 desired position 저장
	int32_t dxl_present_position[5] = { 0,0,0,0,0 };			// DXL 모터에서 받아올 present position 저장
	uint8_t dxl_error = 0;	
	int DXL_ID[5] = { DXL1_ID, DXL2_ID, DXL3_ID, DXL4_ID, DXL5_ID };	
	for (int i = 0; i < 5; i++) { 
		if (!groupSyncReadPosition.addParam(DXL_ID[i])) { fprintf(stderr, "[ID:%03d] groupSyncReadPosition addparam failed", DXL_ID[i]); return 0; }
		if (!groupSyncReadCurrent.addParam(DXL_ID[i])) { fprintf(stderr, "[ID:%03d] groupSyncReadCurrent addparam failed", DXL_ID[i]); return 0; }
	}	
	if (portHandler->openPort()) // Open port		
		printf("Succeeded to open the port!\n");
	else {
		printf("Failed to open the port!\n");
		return 0;
	}	
	if (portHandler->setBaudRate(BAUDRATE)) // Set port baudrate
		printf("Succeeded to change the baudrate!\n");
	else {
		printf("Failed to change the baudrate!\n");		
		return 0;
	}	
	for (int i = 0; i < 5; i++) { // Enable Dynamixel Torque
		dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID[i], ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
		if (dxl_comm_result != COMM_SUCCESS)
			printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
		else if (dxl_error != 0)
			printf("%s\n", packetHandler->getRxPacketError(dxl_error));
		else
			printf("Dynamixel#%d has been successfully connected \n", DXL_ID[i]);
	}

	/////////// FILE WRITE PARAMETERS -> x64/Release/_CSV/Webcam/ 폴더에 날짜시간_파일명 이런 식으로 저장됨
	ofstream outData_openface, outData_FacePosition, outData_position, outData_RPY;
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
	if (FLAG_SAVE_FACEPOSITION)
		outData_FacePosition.open("_CSV/Webcam/" + year + month + date + ";" + hour + "-" + min + "-" + second + " facepos.csv", ios::app);
	if (FLAG_SAVE_DXL_PRESENT_POSITION)
		outData_position.open("_CSV/Webcam/" + year + month + date + ";" + hour + "-" + min + "-" + second + " pos.csv", ios::app);
	if (FLAG_SAVE_RPY)
		outData_RPY.open("_CSV/Webcam/" + year + month + date + ";" + hour + "-" + min + "-" + second + " rpy.csv", ios::app);

	//----- HOMING MODE -> 구동 시작 전, 후로 home position으로 보냄
	setDXLGoalPosition(dxl_goal_position, default_PITCH, default_ROLLR, default_ROLLL, default_YAW, default_MOUTH);
	//if (!moveDXLtoDesiredPosition(groupSyncWriteVelocity, groupSyncWritePosition, DXL_PROFILE_VELOCITY_HOMING, dxl_goal_position)) { return 0; }
	if (!moveDXLtoDesiredPosition_smallrobot(portHandler, packetHandler, dxl_goal_position, DXL_PROFILE_VELOCITY_HOMING)) { return 0; }

	std::this_thread::sleep_for(3s);

	// ----------------------------------------------------------------------------------------
	// ----------------------------------------------------------------------------------------
	// ----------------------------------------------------------------------------------------
	while (true)
	{
		//// OpenFace 시작하는 함수 여러 가지
		if (!sequence_reader.Open(arguments))
			break;
		INFO_STREAM("Device or file opened");
		if (sequence_reader.IsWebcam()) {
			INFO_STREAM("WARNING: using a webcam in feature extraction, Action Unit predictions will not be as accurate in real-time webcam mode");
			INFO_STREAM("WARNING: using a webcam in feature extraction, forcing visualization of tracking to allow quitting the application (press q)");
			visualizer.vis_track = true;
		}
		cv::Mat captured_image;
		Utilities::RecorderOpenFaceParameters recording_params(arguments, true, sequence_reader.IsWebcam(),
			sequence_reader.fx, sequence_reader.fy, sequence_reader.cx, sequence_reader.cy, sequence_reader.fps);
		if (!face_model.eye_model)
			recording_params.setOutputGaze(false);
		Utilities::RecorderOpenFace open_face_rec(sequence_reader.name, recording_params, arguments);
		if (recording_params.outputGaze() && !face_model.eye_model)
			std::cout << "WARNING: no eye model defined, but outputting gaze" << std::endl;
		captured_image = sequence_reader.GetNextFrame();		
		double reported_completion = 0; // For reporting progress
		INFO_STREAM("Starting tracking");

		/// -------------------------------------------------------------------------------------------------------------------------------------------------
		/// -------------------------------------------------------------------------------------------------------------------------------------------------
		/// -------------------------------------------------------------------------------------------------------------------------------------------------
		
		// open_face thread에서 나오는 변수들 중에 dynamixel_control thread에서 사용할 변수들
		double roll, pitch, yaw, mouth;			// gaze 로봇 pose
		double gamma;							// 논문 참조
		VectorXd pH(3), xyz_f(3), RPY(4);		// p_s[t], p_s[t] (논문 41번 식; xyz 좌표), gaze 위치에 따른 로봇의 RPY, gamma
		char stop;
		float confidence;						// OpenFace 신뢰도
		 
		// OpenFace 돌리면서 gaze 방향에 따른 로봇 configuration 계산하는 thread
		auto open_face = [&]()
		{
			INFO_STREAM("open_face_start");

			bool flag_initial = true;

			std::chrono::high_resolution_clock::time_point threadstartTime;
			int threaditer = 0;

			while (!captured_image.empty()) {

				// Converting to grayscale
				// x max: sequence_reader.frame_width, y max: sequence_reader.frame_height
				cv::Mat_<uchar> grayscale_image = sequence_reader.GetGrayFrame();

				//cout << "sequence_reader: " << sequence_reader.frame_width << " " << sequence_reader.frame_height << endl;

				// The actual facial landmark detection / tracking				
				bool detection_success = LandmarkDetector::DetectLandmarksInVideo(captured_image, face_model, det_parameters, grayscale_image);

				// Gaze tracking, absolute gaze direction
				cv::Point3f gazeDirection0(0, 0, 0); cv::Point3f gazeDirection1(0, 0, 0); cv::Vec2d gazeAngle(0, 0);

				if (detection_success && face_model.eye_model) {
					GazeAnalysis::EstimateGaze(face_model, gazeDirection0, sequence_reader.fx, sequence_reader.fy, sequence_reader.cx, sequence_reader.cy, true);
					GazeAnalysis::EstimateGaze(face_model, gazeDirection1, sequence_reader.fx, sequence_reader.fy, sequence_reader.cx, sequence_reader.cy, false);
					gazeAngle = GazeAnalysis::GetGazeAngle(gazeDirection0, gazeDirection1);
				}

				// Do face alignment
				cv::Mat sim_warped_img;
				cv::Mat_<double> hog_descriptor; int num_hog_rows = 0, num_hog_cols = 0;				
				
				// [0]: X, [1]: Y, [2]: Z, [3]: pitch, [4]: yaw, [5]: roll
				cv::Vec6d pose_estimate = LandmarkDetector::GetPose(face_model, sequence_reader.fx, sequence_reader.fy, sequence_reader.cx, sequence_reader.cy);

				//// https://github.com/TadasBaltrusaitis/OpenFace/wiki/Output-Format 
				//// https://github.com/TadasBaltrusaitis/OpenFace/wiki/API-calls
				//std::vector <cv::Point2f> eye_landmarks = LandmarkDetector::CalculateAllEyeLandmarks(face_model); // size 56				
				
				// size 136, [x1;x2;...xn;y1;y2...yn], n=68
				// face center x: all_landmarks(27), y: all_landmarks(27+68)				
				//cv::Mat1f all_landmarks = face_model.detected_landmarks; 

				// rows: 3, cols: 68
				cv::Mat1f eyescenter = face_model.GetShape(sequence_reader.fx, sequence_reader.fy, sequence_reader.cx, sequence_reader.cy);

				// x: front, y: right, z: up
				float camx = eyescenter(2, 27);  // OpenFace z-axis == Ray x-axis (논문 참조)
				float camy = -eyescenter(0, 27); // OpenFace -x-axis == Ray y-axis
				float camz = -eyescenter(1, 27); // OpenFace -y-axis == Ray z-axis
				cout << camx << endl;
				cout << camy << endl;
				cout << camz << endl;

				pH(0) = Poc_x + camx;
				pH(1) = (Poc_y + camy)*0.75;		// 카메라 or OpenFace가 조금 부정확한 거 같아서 임의로 튜닝

				if (Poc_z + camz > 0)
					pH(2) = (Poc_z + camz) * 0.9;	// 카메라 or OpenFace가 조금 부정확한 거 같아서 임의로 튜닝
				else
					pH(2) = (Poc_z + camz) * 0.7;	// 카메라 or OpenFace가 조금 부정확한 거 같아서 임의로 튜닝

				xyz_f = pH;
				// cout << "xyz : " << pH << endl;

				RPY = findGaze_RPY_Gamma(xyz_f);	// 논문 Algorithm 1 (gaze 위치 -> 로봇 configuration 변환)
				//RPY = findGaze_RPY_Gamma(pH);
				roll = RPY(0);
				pitch = RPY(1);
				yaw = RPY(2);
				gamma = RPY(3);

				if (flag_initial) {
					face_model.detected_landmarks = getInitialFaceLandmarks();
					flag_initial = false;
					threadstartTime = std::chrono::high_resolution_clock::now();
				}

				fps_tracker.AddFrame();

				// Displaying the tracking visualizations
				visualizer.SetImage(captured_image, sequence_reader.fx, sequence_reader.fy, sequence_reader.cx, sequence_reader.cy);
				visualizer.SetObservationFaceAlign(sim_warped_img);
				visualizer.SetObservationHOG(hog_descriptor, num_hog_rows, num_hog_cols);
				visualizer.SetObservationLandmarks(face_model.detected_landmarks, face_model.detection_certainty, face_model.GetVisibilities());
				visualizer.SetObservationPose(pose_estimate, face_model.detection_certainty);
				visualizer.SetObservationGaze(gazeDirection0, gazeDirection1, LandmarkDetector::CalculateAllEyeLandmarks(face_model), LandmarkDetector::Calculate3DEyeLandmarks(face_model, sequence_reader.fx, sequence_reader.fy, sequence_reader.cx, sequence_reader.cy), face_model.detection_certainty);
				visualizer.SetObservationActionUnits(face_analyser.GetCurrentAUsReg(), face_analyser.GetCurrentAUsClass());
				visualizer.SetFps(fps_tracker.GetFPS());

				// detect key presses
				char character_press = visualizer.ShowObservation();

				// quit processing the current sequence (useful when in Webcam mode)
				if (stop == 'q' || character_press == 'q') {
					stop = 'q';
					INFO_STREAM("break");
					break;
				}

				confidence = face_model.detection_certainty; // 0~1 (1 good)
				//cout << "confidence = " << confidence << endl;

				// Reporting progress
				if (sequence_reader.GetProgress() >= reported_completion / 10.0) {
					std::cout << reported_completion * 10 << "% ";
					if (reported_completion == 10)
						std::cout << std::endl;
					reported_completion = reported_completion + 1;
				}

				// Grabbing the next frame in the sequence
				captured_image = sequence_reader.GetNextFrame();				

				auto awaketime = threadstartTime + threaditer * 40ms;
				while (std::chrono::high_resolution_clock::now() < awaketime) { std::this_thread::yield(); }
				threaditer++;
			}

			INFO_STREAM("open_face_end");
		};

		/// -------------------------------------------------------------------------------------------------------------------------------------------------
		/// -------------------------------------------------------------------------------------------------------------------------------------------------
		/// -------------------------------------------------------------------------------------------------------------------------------------------------
				
		int run_iteration = 1;
		int isStopped = 0;
		double roll_f, pitch_f, yaw_f, mouth_f; // filtered roll,pitch,yaw,mouth values
		VectorXd realXYZ;						// Pitch, Yaw, gamma에 따른 gaze 위치 역계산. 그냥 확인용
		VectorXd RPY_pred;						// 예측된 RPY 값

		// gaze 방향에 따른 로봇 configuration을 모터 위치로 변환해서(TRO 논문 inverse kinematics) 모터 움직이는 thread
		auto dynamixel_control = [&]()
		{
			INFO_STREAM("dynamixel_control_start");
			int iter = 0;
			int delta_timestep = 3;
			bool flag_write = true;				// send control command to DXL
			bool previous_write_flag = true;	// flag_write in previous iteration
			bool flag_start_moving;				// when robot starts to move from stop, then true
			std::vector<double> past_dxl_goal_positon(5); // pitch, roll_r, roll_l, yaw, mouth
			std::vector<double> past_values_pitch(delta_timestep), past_values_roll(delta_timestep), delta(2); // pitch, roll

			double present_val_p, present_val_r, present_val_y;
			double threshold_grad, threshold_val_p, threshold_val_r, threshold_val_y, confidence_limit;

			threshold_grad = 0.40; 
			threshold_val_p = MAX_PITCH; 
			threshold_val_r = MAX_ROLL; 
			threshold_val_y = MAX_YAW; 
			confidence_limit = 0.75;

			bool firstRun = true;
			long long startTime, millisec_since_epoch1;

			int dxl_goal_position_justbefore[5] = { 0,0,0,0,0 }; // just in case if dxl values become zero when 'q' pressed

			std::chrono::high_resolution_clock::time_point threadstartTime;
			int threaditer = 0;

			while (!captured_image.empty()) {

				present_val_p = pitch;
				present_val_r = roll;
				present_val_y = yaw;

				//------------------------------- AVOID SUDDEN MOVEMENT (논문 6페이지 마지막 ~ 7페이지 시작 참조)
				//---------------------

				// calculate gradient
				if (iter >= delta_timestep) {
					delta[0] = present_val_p - past_values_pitch[iter % delta_timestep];
					past_values_pitch[iter % delta_timestep] = present_val_p;
					delta[1] = present_val_r - past_values_roll[iter % delta_timestep];
					past_values_roll[iter % delta_timestep] = present_val_r;

				}
				else {
					past_values_pitch[iter % delta_timestep] = present_val_p;
					past_values_roll[iter % delta_timestep] = present_val_r;
				}

				iter++;
				if (iter >= delta_timestep * 1000000) iter = delta_timestep;

				// check gradient				
				bool flag_grad_all_smaller_than_thr = true;
				for (int mn = 0; mn < delta.size(); mn++) {
					if (abs(delta[mn]) > threshold_grad) {
						flag_grad_all_smaller_than_thr = false;
						break;
					}
				}

				// check absolute angle value
				bool flag_val_all_smaller_than_thr = true;

				// check if the robot starts to move from stop, check if dxl move or not
				flag_start_moving = false;
				if (flag_val_all_smaller_than_thr && flag_grad_all_smaller_than_thr && confidence > confidence_limit) {
					flag_write = true;
					if (flag_write != previous_write_flag)
						flag_start_moving = true;
				}
				else
					flag_write = false;
				previous_write_flag = flag_write;

				//---------------------
				//------------------------------- AVOID SUDDEN MOVEMENT

				if (flag_write || run_iteration == 1) {
					
					vector<double> filteredRPYM = KalmanFiltering_Gaze(roll, pitch, yaw);
					
					roll_f = filteredRPYM[0];
					pitch_f = filteredRPYM[1];
					yaw_f = filteredRPYM[2];
					roll_f = 0;						// gaze에는 roll 불필요
					mouth_f = 0;					// gaze만 할때는 입 0으로 고정

					// output index
					// 0, 1, 2: roll, pitch, yaw 순서로 phi[t] + delta_phi[t] (논문 39번 식 참조)
					// 3, 4, 5: roll, pitch, yaw 순서로 phi[t] + mu[t] x delta_phi[t] -> 이거 사용
					RPY_pred = predictRollPitchYaw_mod(roll_f, pitch_f, yaw_f, flag_start_moving, 4, 4, 4);
					RPY_pred(0) = 0; // roll
										
					realXYZ = getGazePosition(RPY_pred(4), RPY_pred(5), gamma);					
										
					vector<int> DXL = RPY2DXL(RPY_pred(3), RPY_pred(4), RPY_pred(5), mouth_f, 0);		// 로봇 RPY config.를 모터 위치로 변환
					setDXLGoalPosition(dxl_goal_position, DXL[0], DXL[1], DXL[2], DXL[3], DXL[4]);

					if (flag_start_moving && run_iteration != 1) {
						for (int i = 0; i < 5; i++)
							dxl_goal_position[i] = (dxl_goal_position[i] + past_dxl_goal_positon[i]) / 2;
					}

					for (int i = 0; i < 5; i++)
						past_dxl_goal_positon[i] = dxl_goal_position[i];

					// MOVE <-> STOPs (s 누르면 중간에 멈추고 또 누르면 다시 움직임)
					int dxl_goal_position_keyboard[5];
					isStopped = KeyBoardStopMove(dxl_goal_position, dxl_goal_position_keyboard);

					for (int m = 0; m < 5; m++) { // when 'q' pressed, all values becoming zero happened (temporarily)
						if (dxl_goal_position_keyboard[m] < 10 && run_iteration != 1) {
							dxl_goal_position_keyboard[m] = dxl_goal_position_justbefore[m];
						}
					}
					if (run_iteration == 1) {
						if (dxl_goal_position_keyboard[0] < 10) dxl_goal_position_keyboard[0] = default_PITCH;
						if (dxl_goal_position_keyboard[1] < 10) dxl_goal_position_keyboard[1] = default_ROLLR;
						if (dxl_goal_position_keyboard[2] < 10) dxl_goal_position_keyboard[2] = default_ROLLL;
						if (dxl_goal_position_keyboard[3] < 10) dxl_goal_position_keyboard[3] = default_YAW;
						if (dxl_goal_position_keyboard[4] < 10) dxl_goal_position_keyboard[4] = default_MOUTH;
					}

					// 모터를 goal position으로 움직임 -> 천정아 연구원이 새로 짠 함수 가져와서 사용!!!
					//if (!moveDXLtoDesiredPosition_NoVelLimit(packetHandler, groupSyncWritePosition, dxl_goal_position_keyboard)) return 0;
					if (!moveDXLtoDesiredPosition_NoVelLimit_smallrobot(packetHandler, groupSyncWritePosition, dxl_goal_position)) return 0; 

					for (int k = 0; k < 5; k++)
						dxl_goal_position_justbefore[k] = dxl_goal_position_keyboard[k];

					if (run_iteration) run_iteration++;
				}

				if (firstRun) {
					startTime = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
					millisec_since_epoch1 = startTime;
					firstRun = false;

					threadstartTime = std::chrono::high_resolution_clock::now();

				}
				else
					millisec_since_epoch1 = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();

				// CSV 파일 저장
				if (FLAG_SAVE_FACEPOSITION)
				{
					outData_FacePosition << millisec_since_epoch1 - startTime
						<< "," << pH(0) << "," << realXYZ(0)
						<< "," << pH(1) << "," << realXYZ(1)
						<< "," << pH(2) << "," << realXYZ(2)
						<< "," << xyz_f(0) << "," << xyz_f(1) << "," << xyz_f(2)
						<< "," << gamma
						<< endl;
				}

				// CSV 파일 저장
				if (FLAG_SAVE_RPY) {
					outData_RPY << millisec_since_epoch1 - startTime
						<< "," << roll << "," << roll_f
						<< "," << pitch << "," << pitch_f
						<< "," << yaw << "," << yaw_f
						<< "," << RPY_pred(0) << "," << RPY_pred(1) << "," << RPY_pred(2)
						<< "," << RPY_pred(3) << "," << RPY_pred(4) << "," << RPY_pred(5)
						<< endl;
				}				

				// CSV 파일 저장
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

					outData_position << millisec_since_epoch1 - startTime << "," << dxl_goal_position[0] << "," << dxl_present_position[0]
						<< "," << dxl_goal_position[1] << "," << dxl_present_position[1]
						<< "," << dxl_goal_position[2] << "," << dxl_present_position[2]
						<< "," << dxl_goal_position[3] << "," << dxl_present_position[3] << "," << dxl_goal_position[4] << "," << dxl_present_position[4]
						<< endl;
				}

				if (isStopped == 2)
					stop = 'q';

				if (stop == 'q') {
					INFO_STREAM("dynamixel_control break");
					break;
				}

				auto awaketime = threadstartTime + threaditer * 40ms;
				while (std::chrono::high_resolution_clock::now() < awaketime) { std::this_thread::yield(); }
				threaditer++;
			}

		};

		//// open_face thread, dynamixel_control thread 동시 실행
		std::thread t1 = std::thread(open_face);
		std::thread t3 = std::thread(dynamixel_control);
		t1.join();
		t3.join();

		INFO_STREAM("Closing output recorder");	open_face_rec.Close();
		INFO_STREAM("Closing input reader");	sequence_reader.Close();
		INFO_STREAM("Closed successfully");

		if (recording_params.outputAUs()) {
			INFO_STREAM("Postprocessing the Action Unit predictions");
			face_analyser.PostprocessOutputFile(open_face_rec.GetCSVFile());
		}

		// Reset the models for the next video
		face_analyser.Reset();
		face_model.Reset();
	}

	//----- HOMING MODE
	setDXLGoalPosition(dxl_goal_position, END_PITCH, END_ROLLR, END_ROLLL, END_YAW, END_MOUTH);
	//if (!moveDXLtoDesiredPosition(groupSyncWriteVelocity, groupSyncWritePosition, DXL_PROFILE_VELOCITY_HOMING, dxl_goal_position)) { return 0; }
	if (!moveDXLtoDesiredPosition_smallrobot(portHandler, packetHandler, dxl_goal_position, DXL_PROFILE_VELOCITY_HOMING)) { return 0; }

	std::this_thread::sleep_for(3s);

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