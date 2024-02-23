
// FACEROBOT WEBCAM

//#include "MacrosAndFunctions.h"    
#include "MacrosAndFunctions_small.h"

#undef  FLAG_SAVE_DXL_PRESENT_POSITION
#define FLAG_SAVE_DXL_PRESENT_POSITION		false

#undef  FLAG_SAVE_DXL_PRESENT_CURRENT
#define FLAG_SAVE_DXL_PRESENT_CURRENT		false

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

	// Load the modules that are being used for tracking and face analysis
	// Load face landmark detector
	LandmarkDetector::FaceModelParameters det_parameters(arguments);
	// Always track gaze in feature extraction
	LandmarkDetector::CLNF face_model(det_parameters.model_location);

	if (!face_model.loaded_successfully) {
		std::cout << "ERROR: Could not load the landmark detector" << std::endl;
		return 1;
	}

	// Load facial feature extractor and AU analyser
	FaceAnalysis::FaceAnalyserParameters face_analysis_params(arguments);
	//face_analysis_params.OptimizeForVideos();
	FaceAnalysis::FaceAnalyser face_analyser(face_analysis_params);

	if (!face_model.eye_model)
		std::cout << "WARNING: no eye model found" << std::endl;
	
	Utilities::SequenceCapture sequence_reader;

	// A utility for visualizing the results
	Utilities::Visualizer visualizer(arguments);

	// Tracking FPS for visualization
	Utilities::FpsTracker fps_tracker;
	// The sequence reader chooses what to open based on command line arguments provided

	if (face_analyser.GetAUClassNames().size() == 0 && face_analyser.GetAUClassNames().size() == 0)
		std::cout << "WARNING: no Action Unit models found" << std::endl;

	fps_tracker.AddFrame();

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
	ofstream outData_openface, outData_position, outData_current, outData_RPY;
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
	outData_openface.open("_CSV/Webcam/" + year + month + date + ";" + hour + "-" + min + "-" + second + " openface" + ".csv", ios::app);
	if(FLAG_SAVE_DXL_PRESENT_POSITION)
		outData_position.open("_CSV/Webcam/"+year + month + date + ";" + hour + "-" + min + "-" + second + " pos.csv", ios::app);	
	if(FLAG_SAVE_DXL_PRESENT_CURRENT)
		outData_current.open("_CSV/Webcam/" + year + month + date + ";" + hour + "-" + min + "-" + second + " current" + ".csv", ios::app);
	outData_RPY.open("_CSV/Webcam/" + year + month + date + ";" + hour + "-" + min + "-" + second + " rpy.csv", ios::app);

	
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
	//if (!moveDXLtoDesiredPosition(groupSyncWriteVelocity, groupSyncWritePosition, DXL_PROFILE_VELOCITY_HOMING, dxl_goal_position)) { return 0; }
	if (!moveDXLtoDesiredPosition_smallrobot(portHandler, packetHandler, dxl_goal_position, DXL_PROFILE_VELOCITY_HOMING)) { return 0; }

	std::this_thread::sleep_for(5s);

	// ----------------------------------------------------------------------------------------
	// ----------------------------------------------------------------------------------------
	// ----------------------------------------------------------------------------------------
	while (true) // this is not a for loop as we might also be reading from a webcam
	{	
		if (!sequence_reader.Open(arguments))
			break;

		INFO_STREAM("Device or file opened");

		if (sequence_reader.IsWebcam())	{
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

		// For reporting progress
		double reported_completion = 0;

		INFO_STREAM("Starting tracking");

		double roll, pitch, yaw, mouth;
		char stop;		
		float confidence;

		/// -------------------------------------------------------------------------------------------------------------------------------------------------
		/// -------------------------------------------------------------------------------------------------------------------------------------------------
		/// -------------------------------------------------------------------------------------------------------------------------------------------------
		auto open_face = [&]()
		{			
			INFO_STREAM("open_face_start");

			bool flag_initial = true;

			std::chrono::high_resolution_clock::time_point threadstartTime;
			int threaditer = 0;

			while (!captured_image.empty()) {

				//if (!FLAG_SAVE_DXL_PRESENT_POSITION && !FLAG_SAVE_DXL_PRESENT_CURRENT)		// FALSE FALSE
				//	std::this_thread::sleep_for(0ms);
				//else if (FLAG_SAVE_DXL_PRESENT_POSITION && FLAG_SAVE_DXL_PRESENT_CURRENT)	// TRUE TRUE
				//	std::this_thread::sleep_for(20ms);
				//else																		// TRUE FALSE
				//	std::this_thread::sleep_for(0ms);

				//cout << "openface thread---------------" << endl;
							
				//auto millisec_since_epoch_1 = std::chrono::high_resolution_clock::now();
				// Converting to grayscale
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
				cv::Vec6d pose_estimate = LandmarkDetector::GetPose(face_model, sequence_reader.fx, sequence_reader.fy, sequence_reader.cx, sequence_reader.cy);
				
				//// https://github.com/TadasBaltrusaitis/OpenFace/wiki/Output-Format 
				//// https://github.com/TadasBaltrusaitis/OpenFace/wiki/API-calls
				// std::vector <cv::Point2f> eye_landmarks = LandmarkDetector::CalculateAllEyeLandmarks(face_model); // size 56				
				//// 
				//// size 136, [x1;x2;...xn;y1;y2...yn], n=68
				//// face center x: all_landmarks(27), y: all_landmarks(27+68)
				//// x max: sequence_reader.frame_width, y max: sequence_reader.frame_height
				// cv::Mat1f all_landmarks = face_model.detected_landmarks; 

				roll = pose_estimate[5];
				pitch = pose_estimate[3];
				yaw = pose_estimate[4];

				//cout << "roll  = " << roll << endl;
				//cout << "pitch = " << pitch << endl;
				//cout << "yaw   = " << yaw << endl;

				//cout << "X = " << pose_estimate[0] << endl;
				//cout << "Y = " << pose_estimate[1] << endl;
				//cout << "Z = " << pose_estimate[2] << endl;

				if (flag_initial) {
					face_model.detected_landmarks = getInitialFaceLandmarks();
					flag_initial = false;
					threadstartTime = std::chrono::high_resolution_clock::now();
				}

				// get an action unit 25 to control lip movement.
				face_analyser.PredictStaticAUsAndComputeFeatures(captured_image, face_model.detected_landmarks);

				auto aus_intensity = face_analyser.GetCurrentAUsReg();
				// 0과 1로 출력하려면 : auto aus_presence = face_analyser.GetCurrentAUsClass();
				// first가 이름, second가 값

				mouth = aus_intensity[14].second;
				//cout << aus_intensity[14].first << endl;
				//cout << mouth << endl;
				if (mouth > 2.5) { mouth = 2.5; }
				else if (mouth < 0.7) { mouth = 0; }

				//mouth *= 1.5;

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

				confidence = face_model.detection_certainty;

				// Reporting progress
				if (sequence_reader.GetProgress() >= reported_completion / 10.0) {
					std::cout << reported_completion * 10 << "% ";
					if (reported_completion == 10)
						std::cout << std::endl;					
					reported_completion = reported_completion + 1;
				}

				//auto awake_time1 = millisec_since_epoch_1 + 150ms; //loop_time control //revision: 31.6, 40
				
				// Grabbing the next frame in the sequence
				captured_image = sequence_reader.GetNextFrame();
				//while (std::chrono::high_resolution_clock::now() < awake_time1) { std::this_thread::yield(); }

				auto awaketime = threadstartTime + threaditer * 40ms;
				while (std::chrono::high_resolution_clock::now() < awaketime) { std::this_thread::yield(); }
				threaditer++;

			}

			//// Perform AU detection and HOG feature extraction, as this can be expensive only compute it if needed by output or visualization
			//if (recording_params.outputAlignedFaces() || recording_params.outputHOG() || recording_params.outputAUs() || visualizer.vis_align || visualizer.vis_hog || visualizer.vis_aus)
			//{
			//	face_analyser.AddNextFrame(captured_image, face_model.detected_landmarks, face_model.detection_success, sequence_reader.time_stamp, sequence_reader.IsWebcam());
			//	face_analyser.GetLatestAlignedFace(sim_warped_img);
			//	face_analyser.GetLatestHOG(hog_descriptor, num_hog_rows, num_hog_cols);
			//}
			INFO_STREAM("open_face_end");
		};
		
		/// -------------------------------------------------------------------------------------------------------------------------------------------------
		/// -------------------------------------------------------------------------------------------------------------------------------------------------
		/// -------------------------------------------------------------------------------------------------------------------------------------------------
		 
		//bool first_Run = true;
		int run_iteration = 1;
		int isStopped = 0;
		double roll_f, pitch_f, yaw_f, mouth_f; // filtered roll,pitch,yaw,mouth values
		VectorXd RPY_pred;

		auto dynamixel_control = [&]()
		{
			INFO_STREAM("dynamixel_control_start");
			int iter = 0;
			int delta_timestep = 3;
			bool flag_write = true;				// send control command to DXL
			bool previous_write_flag = true;	// flag_write in previous iteration
			bool flag_start_moving;				// when robot starts to move from stop, then true
			std::vector<double> past_dxl_goal_positon(5); // pitch,rollr,rolll,yaw,mouth
			std::vector<double> past_values_pitch(delta_timestep), past_values_roll(delta_timestep), delta(2); // pitch, roll
	
			double present_val_p, present_val_r, present_val_y;
			double threshold_grad, threshold_val_p, threshold_val_r, threshold_val_y, confidence_limit;

			threshold_grad = 0.40; // 0.45
			threshold_val_p = 0.7; // pitch
			threshold_val_r = 0.8; // roll
			threshold_val_y = 1.0; // yaw, mouth 1.1
			confidence_limit = 0.75;

			//double rpym_whenStopped[4] = { 0,0,0,0 };
			

			bool firstRun = true;
			long long startTime, millisec_since_epoch1;

			int dxl_goal_position_justbefore[5] = { 0,0,0,0,0 }; // just in case if dxl values become zero when 'q' pressed

			std::chrono::high_resolution_clock::time_point threadstartTime;
			int threaditer = 0;

			while (!captured_image.empty()) {	

				//if (!FLAG_SAVE_DXL_PRESENT_POSITION && !FLAG_SAVE_DXL_PRESENT_CURRENT)		// FALSE FALSE
				//	std::this_thread::sleep_for(25ms);
				//else if (FLAG_SAVE_DXL_PRESENT_POSITION && FLAG_SAVE_DXL_PRESENT_CURRENT)	// TRUE TRUE
				//	std::this_thread::sleep_for(0ms);
				//else																		// TRUE FALSE
				//	std::this_thread::sleep_for(0ms);
			
				//cout << "dynamixel thread running" << endl;
				
				present_val_p = pitch;
				present_val_r = roll;
				present_val_y = yaw;

				//------------------------------- AVOID SUDDEN MOVEMENT
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
				if (abs(present_val_p) > threshold_val_p || abs(present_val_r) > threshold_val_r || abs(present_val_y) > threshold_val_y)
					flag_val_all_smaller_than_thr = false;
				
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

					vector<double> filteredRPYM = KalmanFiltering(roll, pitch, yaw, mouth);
					//vector<double> filteredRPYM = MeanFiltering_RPYM(roll, pitch, yaw, mouth);
					roll_f = filteredRPYM[0];
					pitch_f = filteredRPYM[1];
					yaw_f = filteredRPYM[2];
					mouth_f = filteredRPYM[3];


					//RPY_pred = predictRollPitchYaw(roll_f, pitch_f, yaw_f, flag_start_moving, 3);
					RPY_pred = predictRollPitchYaw_mod(roll_f, pitch_f, yaw_f, flag_start_moving, 3, 3, 3);

					// change roll pitch yaw mouth values to DXL positions
					int mode = 0; // mirroring
					//int mode = 1; // cloning
					//vector<int> DXL = RPY2DXL(roll_f, pitch_f, yaw_f, mouth_f, mode);
					//vector<int> DXL = RPY2DXL(RPY_pred(0), RPY_pred(1), RPY_pred(2), mouth_f, mode);
					vector<int> DXL = RPY2DXL(RPY_pred(3), RPY_pred(4), RPY_pred(5), mouth_f, mode);
					//vector<int> DXL = RPY2DXL(RPY_pred_mod(3), RPY_pred_mod(4), RPY_pred_mod(5), mouth_f, mode);

					//vector<int> DXL_new = RPY2DXL_new(roll_f, pitch_f, yaw_f, mouth_f, mode);
					//for (int df = 0; df < 5; df++)
					//	cout << df << " difference = " << DXL[df] - DXL_new[df] << endl;

					setDXLGoalPosition(dxl_goal_position, DXL[0], DXL[1], DXL[2], DXL[3], DXL[4]);

					if (flag_start_moving && run_iteration != 1) {
						for (int i = 0; i < 5; i++)
							dxl_goal_position[i] = (dxl_goal_position[i] + past_dxl_goal_positon[i]) / 2;						
					}

					for (int i = 0; i < 5; i++)
						past_dxl_goal_positon[i] = dxl_goal_position[i];					

					// MOVE <-> STOPs
					int dxl_goal_position_keyboard[5];
					isStopped = KeyBoardStopMove(dxl_goal_position, dxl_goal_position_keyboard);

					//double rpym_present[4] = { roll_f, pitch_f, yaw_f, mouth_f };					
					//isStopped = KeyBoardStopMove(dxl_goal_position, dxl_goal_position_keyboard, rpym_present, rpym_whenStopped);
					//if (!moveDXLtoDesiredPosition_NoVelLimit(packetHandler, groupSyncWritePosition, dxl_goal_position_keyboard)) return 0;
					
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

					//if (!moveDXLtoDesiredPosition_NoVelLimit(packetHandler, groupSyncWritePosition, dxl_goal_position_keyboard)) return 0;
					if (!moveDXLtoDesiredPosition_NoVelLimit_smallrobot(packetHandler, groupSyncWritePosition, dxl_goal_position)) return 0;

					for (int k = 0; k < 5; k++)
						dxl_goal_position_justbefore[k] = dxl_goal_position_keyboard[k];

					if(run_iteration) run_iteration++;
				}

				if (firstRun) {
					startTime = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
					millisec_since_epoch1 = startTime;
					firstRun = false;

					threadstartTime = std::chrono::high_resolution_clock::now();

				}
				else
					millisec_since_epoch1 = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();




				outData_RPY << millisec_since_epoch1 - startTime
					<< "," << roll << "," << roll_f
					<< "," << pitch << "," << pitch_f
					<< "," << yaw << "," << yaw_f
					<< "," << RPY_pred(0) << "," << RPY_pred(1) << "," << RPY_pred(2)
					<< "," << RPY_pred(3) << "," << RPY_pred(4) << "," << RPY_pred(5)
					//<< "," << RPY_pred_mod(0) << "," << RPY_pred_mod(1) << "," << RPY_pred_mod(2)
					//<< "," << RPY_pred_mod(3) << "," << RPY_pred_mod(4) << "," << RPY_pred_mod(5)
					<< endl;



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

					MeanFiltering_Current(dxl_present_current, dxl_present_current_filtered, false);

					/*outData_current << millisec_since_epoch1 - startTime << "," << dxl_present_current[0] << "," << dxl_present_current[1]
						<< "," << dxl_present_current[2] << "," << dxl_present_current[3] << "," << dxl_present_current[4] << endl;*/
					outData_current << millisec_since_epoch1 - startTime << "," << "0" << ","
						<< dxl_present_current[0] << "," << dxl_present_current[1] << "," << dxl_present_current[2] << "," << dxl_present_current[3] << "," << dxl_present_current[4] << ","
						<< dxl_present_current_filtered[0] << "," << dxl_present_current_filtered[1] << "," << dxl_present_current_filtered[2] << "," << dxl_present_current_filtered[3] << "," << dxl_present_current_filtered[4] << ","
						<< endl;
				}
				
				outData_openface << millisec_since_epoch1 - startTime << "," << roll << "," << roll_f << "," << pitch << "," << pitch_f << ","
					<< yaw << "," << yaw_f << "," << mouth << "," << mouth_f << endl;

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