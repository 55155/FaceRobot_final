#include "MacrosAndFunctions_v0.h"
#include "Eye_module.h"
#include "includePython.h"
#include <conio.h>
#include <deque>
#include <tuple>


// 대충 자로 잰 값 + 실제 값 보면서 calibration 
#define Poc_x 200   + 200
#define Poc_y -150	+20
#define Poc_z 20	
#define EYE_LAYER_HEIGHT	200 



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

	/////////// OpenFace
	///////////
	LandmarkDetector::FaceModelParameters det_parameters(arguments);
	LandmarkDetector::CLNF face_model(det_parameters.model_location);
	if (!face_model.loaded_successfully) { std::cout << "ERROR: Could not load the landmark detector" << std::endl; return 1; }
	FaceAnalysis::FaceAnalyserParameters face_analysis_params(arguments);
	FaceAnalysis::FaceAnalyser face_analyser(face_analysis_params);
	if (!face_model.eye_model) { std::cout << "WARNING: no eye model found" << std::endl; }
	Utilities::SequenceCapture sequence_reader;
	Utilities::Visualizer visualizer(arguments);
	Utilities::FpsTracker fps_tracker;
	if (face_analyser.GetAUClassNames().size() == 0 && face_analyser.GetAUClassNames().size() == 0) { std::cout << "WARNING: no Action Unit models found" << std::endl; }
	fps_tracker.AddFrame();

	/////////// Dynamixel
	///////////
	dynamixel::PortHandler* portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
	dynamixel::PacketHandler* packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
	dynamixel::GroupSyncWrite groupSyncWritePosition(portHandler, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION);		// POSITION
	dynamixel::GroupSyncWrite groupSyncWriteVelocity(portHandler, packetHandler, ADDR_PRO_PROFILE_VELOCITY, LEN_PRO_PROFILE_VELOCITY);	// VELOCITY
	dynamixel::GroupSyncRead groupSyncReadPosition(portHandler, packetHandler, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);	// POSITION
	dynamixel::GroupSyncRead groupSyncReadCurrent(portHandler, packetHandler, ADDR_PRO_PRESENT_CURRENT, LEN_PRO_PRESENT_CURRENT);		// CURRENT
	//--- MOTOR PARAMETWRS
	int dxl_comm_result = COMM_TX_FAIL;
	int dxl_goal_position[5] = { 0,0,0,0,0 };
	int32_t dxl_present_position[5] = { 0,0,0,0,0 };
	uint8_t dxl_error = 0;
	int DXL_ID[5] = { DXL1_ID, DXL2_ID, DXL3_ID, DXL4_ID, DXL5_ID };
	for (int i = 0; i < 5; i++) {
		if (!groupSyncReadPosition.addParam(DXL_ID[i])) { fprintf(stderr, "[ID:%03d] groupSyncReadPosition addparam failed", DXL_ID[i]); return 0; }
		if (!groupSyncReadCurrent.addParam(DXL_ID[i])) { fprintf(stderr, "[ID:%03d] groupSyncReadCurrent addparam failed", DXL_ID[i]); return 0; }
	}
	if (portHandler->openPort()) { printf("Succeeded to open the port!\n"); }
	else { printf("Failed to open the port!\n"); return 0; }
	if (portHandler->setBaudRate(BAUDRATE)) { printf("Succeeded to change the baudrate!\n"); }
	else { printf("Failed to change the baudrate!\n"); return 0; }
	for (int i = 0; i < 5; i++) { // Enable Dynamixel Torque
		dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID[i], ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
		if (dxl_comm_result != COMM_SUCCESS) { printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result)); }
		else if (dxl_error != 0) { printf("%s\n", packetHandler->getRxPacketError(dxl_error)); }
		else { printf("Dynamixel#%d has been successfully connected \n", DXL_ID[i]); }
	}

	/////////// Motion -> 파이썬으로 음성 기반 동작 만들어서 여기에 넣기
	///////////
	string folderpath = "D:\\OpenFace - mouth\\x64\\Release\\AIspeaker_JA\\";

	string audioname = "AIspeaker_answer";

	string audiofilepath = folderpath + "Audio\\" + audioname + ".wav";
	string Headcsvfilepath = folderpath + "Headmotion\\" + audioname + ".csv";
	string Mouthcsvfilepath = folderpath + "Mouthmotion\\" + audioname + "-delta-big.csv";

	//string segmentfolderpath = folderpath + "segments\\";
	string originalMusicPath = audiofilepath;
	// RobotStatus
	std::ifstream Head_in, Mouth_in;


	if (Head_in.fail())
		return(cout << "Head motion file not found" << endl) && 0;

	if (Mouth_in.fail())
		return(cout << "Mouth motion file not found" << endl) && 0;

	auto initiateTime = std::chrono::high_resolution_clock::now();
	auto awaketime = initiateTime + 5000ms;

	setDXLGoalPosition(dxl_goal_position, default_PITCH, default_ROLLR, default_ROLLL, default_YAW, default_MOUTH);
	//if (!moveDXLtoDesiredPosition(groupSyncWriteVelocity, groupSyncWritePosition, DXL_PROFILE_VELOCITY_HOMING, dxl_goal_position)) { return 0; }
	if (!moveDXLtoDesiredPosition(portHandler, packetHandler, dxl_goal_position, DXL_PROFILE_VELOCITY_HOMING)) { return 0; }

	while (std::chrono::high_resolution_clock::now() < awaketime) { std::this_thread::yield(); } // 5초 동안 Thread 양보 다른 thread에


	// 분산 계산
	deque<double> X; // 로봇 시점 X data 50개 slide window
	deque<double> Y; // 로봇 시점 Z data 50개 slide window
	// 평균 계산
	deque<double> X_point;
	deque<double> Y_point;
	deque<double> Z_point;

	VectorXd Glansing_avg_point_robot(3);
	double avg_X = 0;
	double avg_Y = 0;
	double sum_X2 = 0;
	double sum_Y2 = 0;
	double var_X = 0;
	double var_Y = 0;
	double var = 0;
	double L = 0;

	double N = 50; // data set의 개수 -> 25 samples -> 1s
	bool flag_first_cal = true;

	auto execute_python = [&]() {
		int python_output = execute_AIspeaker_py(); // execute_AIspeaker_py()
		// AIspeaker 실행
		if (!python_output) write_robot_status("PYTHON_ERROR"); // output이 이상할 때, -2 write
	};

	while (true)
	{
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

		// For reporting progress
		double reported_completion = 0;

		INFO_STREAM("Starting tracking");

		double roll_OF = 0, pitch_OF = 0, yaw_OF = 0, mouth_OF = 0;	// gaze 로봇 pose
		VectorXd pH(3), xyz_f(3), RPY(4);		// gaze robot pose
		double gamma;							// 논문 참조
		char stop;
		float confidence;

		// 자연스러운 움직임을 위한
		int velocity;
		bool is_recovery = false; // 포지션 회복
		// 함수 주기에 따른 루프 내에서의 람다 선언


		auto open_face = [&]()
		{
			INFO_STREAM("open_face_start");

			bool flag_initial = true;

			std::chrono::high_resolution_clock::time_point threadstartTime;
			int threaditer = 0;
			VectorXd Fixed_Glansing_point_robot; // 카메라에 의해 위치가 갱신되기 때문에, Glasing 지점 고정.
			bool start_slope = false; // true = Glansing 의 시작, false = Glansing 의 끝
			cv::Point3f gazeDirection0(0, 0, 0); cv::Point3f gazeDirection1(0, 0, 0); cv::Vec2d gazeAngle(0, 0);
			double Glansing_time = 2000; // Glansing_time
			double T; // slope 함수 주기
			while (!captured_image.empty()) {
				// default velocity 
				velocity = 40;

				cv::Mat_<uchar> grayscale_image = sequence_reader.GetGrayFrame();
				bool detection_success = LandmarkDetector::DetectLandmarksInVideo(captured_image, face_model, det_parameters, grayscale_image);
				
				if (detection_success && face_model.eye_model) {
					GazeAnalysis::EstimateGaze(face_model, gazeDirection0, sequence_reader.fx, sequence_reader.fy, sequence_reader.cx, sequence_reader.cy, true);
					GazeAnalysis::EstimateGaze(face_model, gazeDirection1, sequence_reader.fx, sequence_reader.fy, sequence_reader.cx, sequence_reader.cy, false);
					gazeAngle = GazeAnalysis::GetGazeAngle(gazeDirection0, gazeDirection1);
				}
				cv::Mat sim_warped_img;
				cv::Mat_<double> hog_descriptor; int num_hog_rows = 0, num_hog_cols = 0;
				cv::Vec6d pose_estimate = LandmarkDetector::GetPose(face_model, sequence_reader.fx, sequence_reader.fy, sequence_reader.cx, sequence_reader.cy);
				// cout << pose_estimate[3] << "\t" << pose_estimate[4] << "\t" << pose_estimate[5] << "\t" << endl;
				cout << radian_to_degree(gazeAngle[0]) << "\t" << radian_to_degree(gazeAngle[1]) << endl;
				// rows: 3, cols: 68
				cv::Mat1f eyescenter = face_model.GetShape(sequence_reader.fx, sequence_reader.fy, sequence_reader.cx, sequence_reader.cy);
				cv::Point3f P_c;
				cv::Point3f P_s;
				cv::Point3f P_sc = { Poc_x, Poc_y, Poc_z };


				float camx = eyescenter(0, 27);
				float camy = eyescenter(1, 27);
				float camz = eyescenter(2, 27);

				P_c.x = camx;
				P_c.y = camy;
				P_c.z = camz;

				// 로봇 중심
				P_s.x = P_sc.x + camz;
				P_s.y = P_sc.y - camx;
				P_s.z = P_sc.z - camy;

				// left eye
				cv::Mat lefteyeLdmks3d = face_model.hierarchical_models[0].GetShape(sequence_reader.fx, sequence_reader.fy, sequence_reader.cx, sequence_reader.cy);

				// right eye
				cv::Mat righteyeLdmks3d = face_model.hierarchical_models[1].GetShape(sequence_reader.fx, sequence_reader.fy, sequence_reader.cx, sequence_reader.cy);

				// get pupil position
				cv::Point3f P_e1 = GazeAnalysis::GetPupilPosition(lefteyeLdmks3d);
				cv::Point3f P_e2 = GazeAnalysis::GetPupilPosition(righteyeLdmks3d);

				// gaze point define
				cv::Point3f Gaze_point_cam;
				VectorXd Gaze_point_robot;

				// EyeGaze point define
				cv::Point3f Glansing_point_cam;
				VectorXd Glansing_point_robot;

				// get gaze direction
				cv::Point3f e1 = gazeDirection0;
				cv::Point3f e2 = gazeDirection1;
				cv::Point3f ec = (e1 + e2) / 2;
				ec = ec / norm(ec);

				// Gaze point, Glansing point 
				std::tie(Gaze_point_cam, Gaze_point_robot) = Gaze(eyescenter);
				std::tie(Glansing_point_cam, Glansing_point_robot) = EyeGaze(eyescenter, ec, gazeAngle, DISTANCE2PLANE);

				// 10cm 정도 떨어진 지점. 
				if (BACKWARD_LIMIT < Glansing_point_robot(0) && Glansing_point_robot(0) < cam_to_robot(0, 0, P_c.z)(0)) { // 시선공유 해야하는 경우  rotation matrix
					// 50개 이전에는 분산을 구할 수 없음.
					if (X.size() < N && Y.size() < N && flag_first_cal) {
						X.push_back(radian_to_degree(gazeAngle(0)));
						Y.push_back(radian_to_degree(gazeAngle(1)));
						
						// X, Y, Z 의 평균위치에 Glansing 하기 위해 X point, Y point, Z point 저장
						X_point.push_back(Glansing_point_robot(0));
						Y_point.push_back(Glansing_point_robot(1));
						Z_point.push_back(Glansing_point_robot(2));
					}
					// 처음 50개의 데이터가 들어왔을 때
					else if (X.size() == N && Y.size() == N && flag_first_cal) {
						Glansing_avg_point_robot(0) = deque_mean(X_point);
						Glansing_avg_point_robot(1) = deque_mean(Y_point);
						Glansing_avg_point_robot(2) = deque_mean(Z_point);

						flag_first_cal = false;
						// 초기 mean, sum^2 계산
						avg_X = deque_mean(X);
						sum_X2 = deque_sum2(X);
						avg_Y = deque_mean(Y);
						sum_Y2 = deque_sum2(Y);
					}
					// 50개 데이터 이후에는 재귀적으로 평균, 분산 계산
					else {
						// Glansing_avg_point_robot update
						Glansing_avg_point_robot(0) = Moving_Avg_Filter(X_point.front(), Glansing_avg_point_robot(0), X_point.back(), N);
						X_point.pop_front();
						X_point.push_back(Glansing_point_robot(0));

						Glansing_avg_point_robot(1) = Moving_Avg_Filter(Y_point.front(), Glansing_avg_point_robot(1), Y_point.back(), N);
						Y_point.pop_front();
						Y_point.push_back(Glansing_point_robot(1));

						Glansing_avg_point_robot(2) = Moving_Avg_Filter(Z_point.front(), Glansing_avg_point_robot(2), Z_point.back(), N);
						Z_point.pop_front();
						Z_point.push_back(Glansing_point_robot(2));

						// X - Z ANGLE 평균, 분산 계산
						double X_kn = X.front();
						X.pop_front();
						double X_k = radian_to_degree(gazeAngle(0));
						X.push_back(X_k);

						double pre_avg_X = avg_X;
						double pre_sum_X2 = sum_X2;
						double pre_val_X = var_X;

						avg_X = Moving_Avg_Filter(X_k, pre_avg_X, X_kn, N);
						sum_X2 = recursive_sum2(X_k, pre_sum_X2, X_kn);
						var_X = variance(sum_X2, avg_X, N);

						// Y-Z ANGLE 평균, 분산 계산
						double Y_kn = Y.front();
						Y.pop_front();
						double Y_k = radian_to_degree(gazeAngle(1));
						Y.push_back(Y_k);

						double pre_avg_Y = avg_Y;
						double pre_sum_Y2 = sum_Y2;
						double pre_val_Y = var_Y;

						avg_Y = Moving_Avg_Filter(Y_k, pre_avg_Y, Y_kn, N);
						sum_Y2 = recursive_sum2(Y_k, pre_sum_Y2, Y_kn);
						var_Y = variance(sum_Y2, avg_Y, N);

						// 표준편차 계산
						var = sqrt(var_Y + var_X);
					}
				}
				else { // Gaze 해야하는 경우 rotation matrix
					// 다시 초기값으로 돌아가기.
					flag_first_cal = true;
					while (!Y.empty()) {
						// 이전 좌표값들 모두 비우기
						X.pop_back();
						Y.pop_back();
						X_point.pop_back();
						Y_point.pop_back();
						Z_point.pop_back();
					}
					L = 0;
					var = 1e9;
				}

				double threshold_var = 1; // degree
				double t;
				// cout << Gaze_point_robot(0) << "\t" << Gaze_point_robot(1) << "\t" << Gaze_point_robot(2) << "\t" << endl;
				// 2초동안 어딘가를 응시하고 있으면서, 그 분산이 크지 않다면, 
				if (var < threshold_var && Y.size() == N && !start_slope) {
					// cout << Glansing_avg_point_robot(0) << "\t" << Glansing_avg_point_robot(1) << "\t" << Glansing_avg_point_robot(2) << "\t" << endl;
					cout << avg_X << "\t"  << avg_Y << endl;
					Fixed_Glansing_point_robot = Glansing_point_robot;			// Glansing point 고정
					t = 0;
					start_slope = true;											// Glansing point로 서서히 이동
					T = Period(Fixed_Glansing_point_robot, Gaze_point_robot);	// T / 2 는 이동하는데 소요되는 시간, 
				}


				if (start_slope) {
					L = x(t, T, Glansing_time);											// 시간 t에 따른 L값 저장
					pH = Lag_Interp(Fixed_Glansing_point_robot, Gaze_point_robot, L);	// L값에 따른 보간 수행
					if (t >= T + Glansing_time && var > threshold_var) {				// 만약 Transition이 모두 수행되었을떄, 
						t = 0;
						start_slope = false;
						while (X.size() && Y.size()) {
							// 이전 좌표값들 모두 비우기
							X.pop_back();
							Y.pop_back();
							X_point.pop_back();
							Y_point.pop_back();
							Z_point.pop_back();
						}
					}
					if(t >= T/2 + Glansing_time && var <= threshold_var){
						// Gaze로 복귀해야하는 시점에도 해당위치를 보고 있으면, 
						t = T / 2;		// 복귀하지 않고 Glansing 
					}
					t += FREQUENCY.count(); // t update
				}
				else {
					// Transition이 되지 않을 때에는 L = 0, 즉 Gaze
					L = 0;
					pH = Lag_Interp(Glansing_point_robot, Gaze_point_robot, L);
				}
				// pH = Glansing_Test(degree_to_radian(20));
				RPY = findGaze_RPY_Gamma(pH);
				roll_OF = RPY(0);
				pitch_OF = RPY(1);
				yaw_OF = RPY(2);

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

				// Reporting progress
				if (sequence_reader.GetProgress() >= reported_completion / 10.0) {
					std::cout << reported_completion * 10 << "% ";
					if (reported_completion == 10)
						std::cout << std::endl;
					reported_completion = reported_completion + 1;
				}

				// Grabbing the next frame in the sequence
				captured_image = sequence_reader.GetNextFrame();

				auto awaketime = threadstartTime + threaditer * FREQUENCY;
				while (std::chrono::high_resolution_clock::now() < awaketime) { std::this_thread::yield(); }
				threaditer++;
			}
			INFO_STREAM("open_face_end");
		};

		double roll_f, pitch_f, yaw_f, mouth_f; // filtered roll,pitch,yaw,mouth values		
		double roll_csv, pitch_csv, yaw_csv, mouth_csv;
		double roll_gaze = 0, pitch_gaze = 0, yaw_gaze = 0;
		VectorXd RPY_pred;
		RobotStatus output = WAITING_FOR_USER_INPUT;
		RobotStatus* status = &output;

		auto dynamixel_control = [&]()
		{
			int iter = 0;
			int delta_timestep = 3;
			bool flag_write = true;				// send control command to DXL
			bool previous_write_flag = true;	// flag_write in previous iteration
			bool flag_start_moving;				// when robot starts to move from stop, then true
			std::vector<double> past_dxl_goal_positon(5); // pitch,rollr,rolll,yaw,mouth
			std::vector<double> past_values_pitch(delta_timestep), past_values_roll(delta_timestep), delta(2); // pitch, roll
			
			double present_val_p = 0, present_val_r = 0, present_val_y = 0;

			double threshold_grad = 0.40; // 0.45
			double threshold_val_p = MAX_PITCH; // pitch
			double threshold_val_r = MAX_ROLL; // roll
			double threshold_val_y = MAX_YAW; // yaw, mouth 1.1
			double confidence_limit = 0.75;

			// status 정리
			// 1 : play
			// 2 : wait
			// -1 : 정상 종료 
			// -2 : python thread error에 의한 종료

			// 대답해야하는 경우 : 1, -1
			// 동작만 생성하는 경우 : 2
			// 대답도 동작도 하지 않는 경우 : -2


			// 기본적으로 계산 status = 2 인 상태에서 상태변화가 감지되면 바로 BREAK

			while (true) {
				if (output == READY_TO_SPEAK || output == QUIT_PROGRAM) {
					INFO_STREAM("dynamixel_control_start");
					INFO_STREAM("Ready for speaking");

					bool firstRun = true; // 

					std::chrono::high_resolution_clock::time_point threadstartTime;
					int threaditer = 0;
					std::ifstream Head_in, Mouth_in;

					Head_in.open(Headcsvfilepath);
					Mouth_in.open(Mouthcsvfilepath);

					while (!captured_image.empty() && Head_in.good() && Mouth_in.good()) {

						//------------------------------- AVOID SUDDEN MOVEMENT
						//---------------------

						present_val_p = pitch_OF;
						present_val_r = roll_OF;
						present_val_y = yaw_OF;
						
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

						if (flag_write) {
							vector<double> filteredRPYM = KalmanFiltering_Gaze(roll_OF, pitch_OF, yaw_OF);
							roll_f = 0;
							pitch_f = filteredRPYM[1];
							yaw_f = filteredRPYM[2];

							RPY_pred = predictRollPitchYaw_mod(roll_f, pitch_f, yaw_f, flag_start_moving, 4, 4, 4);

							roll_gaze = 0;
							pitch_gaze = RPY_pred(4);
							yaw_gaze = RPY_pred(5);
						}

						//// 동작 파일에서 읽어오는 머리 동작(RPY), 입 동작
						std::vector<std::string> row_head = csv_read_row(Head_in, ',');
						std::vector<std::string> row_mouth = csv_read_row(Mouth_in, ',');

						roll_csv = stof(row_head[0]);
						pitch_csv = stof(row_head[1]);
						yaw_csv = stof(row_head[2]);
						mouth_csv = stof(row_mouth[0]) / ROBOT_MOUTH_TUNE; // python에서 이미 곱해져서 나옴

						//// 읽어온 동작 파일에 gaze에 따른 head configuration을 더해줌
						double roll_final = roll_csv + roll_gaze;
						double pitch_final = pitch_csv + pitch_gaze;
						double yaw_final = yaw_csv + yaw_gaze;

						//// 범위 초과하면 줄여주기
						rangeCheckOne(roll_final, MAX_ROLL);
						rangeCheckOne(pitch_final, MAX_PITCH);
						// rangeCheckOne(yaw_final, MAX_YAW);

						vector<int> DXL = RPY2DXL(roll_final, pitch_final, yaw_final, mouth_csv, 0);
						//vector<int> DXL = RPY2DXL(roll_csv, pitch_csv, yaw_csv, mouth_csv, 0);

						setDXLGoalPosition(dxl_goal_position, DXL[0], DXL[1], DXL[2], DXL[3], DXL[4]);

						if (flag_start_moving && !firstRun) { // 처음 움직이는데, 
							for (int i = 0; i < 5; i++)
								dxl_goal_position[i] = (dxl_goal_position[i] + past_dxl_goal_positon[i]) / 2;
						}
						for (int i = 0; i < 5; i++)
							past_dxl_goal_positon[i] = dxl_goal_position[i];

						if (firstRun) {

							moveDXLtoDesiredPosition(portHandler, packetHandler, dxl_goal_position, DXL_PROFILE_VELOCITY_HOMING);
							std::this_thread::sleep_for(500ms);

							if (originalMusicPath == "NONE")
								playwav(audiofilepath);
							else
								playwav(originalMusicPath);
							firstRun = false;
							threadstartTime = std::chrono::high_resolution_clock::now();
						}
						else {
							if (!moveDXLtoDesiredPosition(portHandler, packetHandler, dxl_goal_position, FREQUENCY.count())) return QUIT_PROGRAM;
						}

						auto awaketime = threadstartTime + threaditer * FREQUENCY;
						while (std::chrono::high_resolution_clock::now() < awaketime) { std::this_thread::yield(); }
						threaditer++;

						if (output == WAITING_FOR_USER_INPUT) {
							//----- HOMING MODE
							std::chrono::high_resolution_clock::time_point initiateTime_end = std::chrono::high_resolution_clock::now();
							auto awaketime_end = initiateTime_end + 500ms;
							setDXLGoalPosition(dxl_goal_position, default_PITCH, default_ROLLR, default_ROLLL, default_YAW, default_MOUTH);
							if (!moveDXLtoDesiredPosition(portHandler, packetHandler, dxl_goal_position, DXL_PROFILE_VELOCITY_HOMING / 2)) { return QUIT_PROGRAM; }
							while (std::chrono::high_resolution_clock::now() < awaketime_end) { std::this_thread::yield(); }
						}


						if (stop == 'q') {
							INFO_STREAM("dynamixel_control break");
							output = QUIT_PROGRAM;
						}
					}
					if (output == QUIT_PROGRAM) {
						write_robot_status("QUIT_PROGRAM");
						return QUIT_PROGRAM;
					}
					else {
						*status = WAITING_FOR_USER_INPUT;
						write_robot_status("WAITING_FOR_USER_INPUT");
					}
				}

				if (output == WAITING_FOR_USER_INPUT) {
					INFO_STREAM("dynamixel_control_start");
					INFO_STREAM("Waiting for input");


					bool firstRun = true;

					std::chrono::high_resolution_clock::time_point threadstartTime;
					int threaditer = 0;
					string robot_status = "WAITING_FOR_USER_INPUT";

					RobotStatus output = WAITING_FOR_USER_INPUT;
					double roll_f, pitch_f, yaw_f, delta_mouth; // filtered roll,pitch,yaw,mouth values	
					int dxl_goal_position[DXL_NUM] = { default_PITCH,default_ROLLR,default_ROLLL,default_YAW, default_MOUTH };



					static int veryfirst = 0;
					if (veryfirst == 0) {
						//----- HOMING MODE
						std::chrono::high_resolution_clock::time_point initiateTime_veryfirst = std::chrono::high_resolution_clock::now();
						auto awaketime_veryfirst = initiateTime_veryfirst + 2000ms;
						setDXLGoalPosition(dxl_goal_position, default_PITCH, default_ROLLR, default_ROLLL, default_YAW, default_MOUTH);
						if (!moveDXLtoDesiredPosition(portHandler, packetHandler, dxl_goal_position, DXL_PROFILE_VELOCITY_HOMING / 2)) { return QUIT_PROGRAM; }
						while (std::chrono::high_resolution_clock::now() < awaketime_veryfirst) { std::this_thread::yield(); }
						veryfirst = 1;
					}

					while (!captured_image.empty()) {
						robot_status = read_robot_status();

						if (robot_status == "PYTHON_ERROR") { // 비정상 종료
							output = QUIT_PROGRAM;
							write_robot_status("QUIT_PROGRAM");
							cout << "PYTHON_ERROR" << endl;
							break;
						}
						else if (robot_status == "READY_TO_SPEAK") {
							output = READY_TO_SPEAK;
							cout << "robot_status : " << robot_status << endl;
							break;
						}
						else if (robot_status == "SING_A_SONG") {
							output = SING_A_SONG;
							break;
						}
						else if (robot_status == "QUIT_PROGRAM") {
							output = QUIT_PROGRAM;
							write_robot_status("QUIT_PROGRAM");
							break;
						}

						//------------------------------- AVOID SUDDEN MOVEMENT
						//---------------------

						present_val_p = pitch_OF;
						present_val_r = roll_OF;
						present_val_y = yaw_OF;

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
						if (flag_val_all_smaller_than_thr && flag_grad_all_smaller_than_thr && confidence > confidence_limit) { // 
							flag_write = true;
							if (flag_write != previous_write_flag)
								flag_start_moving = true;
						}
						else
							flag_write = false;
						previous_write_flag = flag_write;

						//---------------------
						//------------------------------- AVOID SUDDEN MOVEMENT

						if (flag_write) {
							vector<double> filteredRPYM = KalmanFiltering_Gaze(roll_OF, pitch_OF, yaw_OF);
							roll_f = 0;
							pitch_f = filteredRPYM[1];
							yaw_f = filteredRPYM[2];

							RPY_pred = predictRollPitchYaw_mod(roll_f, pitch_f, yaw_f, flag_start_moving, 4, 4, 4);

							roll_gaze = 0;
							pitch_gaze = RPY_pred(4);
							yaw_gaze = RPY_pred(5);
						}


						//// 읽어온 동작 파일에 gaze에 따른 head configuration을 더해줌
						double roll_final = roll_gaze;
						double pitch_final = pitch_gaze;
						double yaw_final = yaw_gaze;

						//// 범위 초과하면 줄여주기
						rangeCheckOne(roll_final, MAX_ROLL);
						rangeCheckOne(pitch_final, MAX_PITCH);
						// rangeCheckOne(yaw_final, MAX_YAW);

						// vector<int> DXL = RPY2DXL4singing(roll_final, pitch_final, yaw_final, 0);
						vector<int> DXL = RPY2DXL(roll_final, pitch_final, yaw_final, 0, 0);

						setDXLGoalPosition(dxl_goal_position, DXL[0], DXL[1], DXL[2], DXL[3], DXL[4]);

						if (flag_start_moving && !firstRun) {
							for (int i = 0; i < 5; i++)
								dxl_goal_position[i] = (dxl_goal_position[i] + past_dxl_goal_positon[i]) / 2;
						}
						for (int i = 0; i < 5; i++)
							past_dxl_goal_positon[i] = dxl_goal_position[i];

						if (firstRun) {

							moveDXLtoDesiredPosition(portHandler, packetHandler, dxl_goal_position, DXL_PROFILE_VELOCITY_HOMING);
							std::this_thread::sleep_for(500ms);


							firstRun = false;
							threadstartTime = std::chrono::high_resolution_clock::now();
						}
						else {
							if (!moveDXLtoDesiredPosition(portHandler, packetHandler, dxl_goal_position, FREQUENCY.count())) return QUIT_PROGRAM;
						}

						auto awaketime = threadstartTime + threaditer * FREQUENCY;
						while (std::chrono::high_resolution_clock::now() < awaketime) { std::this_thread::yield(); }
						threaditer++;
						if (stop == 'q') {
							INFO_STREAM("dynamixel_control break");
							write_robot_status("QUIT_PROGRAM");
							return QUIT_PROGRAM;
						}

					}
					*status = output;
				}
					
				// std::this_thread::sleep_for(500ms);
				// convertion 발생시 천천히 delays
			}
		};


		std::thread t1 = std::thread(open_face);
		std::thread t3 = std::thread(dynamixel_control);
		std::thread t2 = std::thread(execute_python);

		t1.join();
		t2.join();
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
	if (!moveDXLtoDesiredPosition(portHandler, packetHandler, dxl_goal_position, DXL_PROFILE_VELOCITY_HOMING)) { return 0; }

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