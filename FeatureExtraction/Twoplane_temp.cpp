
// FACEROBOT WEBCAM GAZE

//#include "MacrosAndFunctions.h"    
#include "MacrosAndFunctions_v0.h"
#include <deque>

// 대충 자로 잰 값 + 실제 값 보면서 calibration 
#define Poc_x 200   + 200
#define Poc_y -150	+20
#define Poc_z 20	
#define EYE_LAYER_HEIGHT 200

#define FLAG_SAVE_FACEPOSITION				true		// 얼굴 위치 좌표값 저장할지 말지
#define FLAG_SAVE_RPY						true		// 로봇 pose RPY 저장할지 말지

#undef  FLAG_SAVE_DXL_PRESENT_POSITION
#define FLAG_SAVE_DXL_PRESENT_POSITION		false		// 모터 위치 저장할지 말지

using namespace std;

// 이동평균선에서의 표준편차를 구해야함. 
// 데이터셋은 2s를 기준으로 표준편차를 구해서 일정수준 이하로 판정될 때, gaze 하는 것으로 설정

// 칼만필터는 어렵지 않아 22p
double Moving_Avg_Filter(double X_k, double pre_avg, double X_kn, double N = 25) {
	double avg;
	avg = pre_avg + (X_k - X_kn) / N;
	return avg;
}

// 직접 계산한 점화식
double recursive_Variance(double X_k, double pre_avg, double pre_val, double X_kn, double N = 25) {
	double avg = Moving_Avg_Filter(X_k, pre_avg, X_kn);
	double val = pre_val + (pow(X_k, 2) - pow(X_kn, 2) - pow(avg, 2) * N + pow(pre_avg, 2) * N) / (N - 1);
	return val;
}

// queue 에서 평균과 분산을 지원하는 메서드 없음. 
#include <numeric>
#include <deque>
double deque_mean(std::deque<double> q) {
	double initial_value = 0.0;
	// std::accumulate 는 intial 값의 자료형을 따라감. 
	double sum = std::accumulate(q.begin(), q.end(), initial_value);
	double mean = sum / q.size();

	return mean;
}

double deque_variance(std::deque<double> q) {
	double mean = deque_mean(q);
	double sumSquaredDifferences = 0.0;
	for (const auto& value : q) {
		double difference = value - mean;
		sumSquaredDifferences += difference * difference;
	}
	double variance = sumSquaredDifferences / (q.size() - 1);

	return variance;
}

double radian_to_degree(double radian) {
	double degree = (180 / PI) * radian;
	return degree;
}

VectorXd Gaze(cv::Point3f eyecenter) {
	cv::Point3f gaze_point_robot;
	VectorXd pH(3);

	// rotation matrix
	gaze_point_robot.x = eyecenter.z;
	gaze_point_robot.y = -eyecenter.x;
	gaze_point_robot.z = -eyecenter.y;

	pH(0) = Poc_x + gaze_point_robot.x;
	pH(1) = (Poc_y + gaze_point_robot.y);		// 카메라 or OpenFace가 조금 부정확한 거 같아서 임의로 튜닝

	if (Poc_z + gaze_point_robot.z > 0)
		pH(2) = (Poc_z + gaze_point_robot.z);	// 카메라 or OpenFace가 조금 부정확한 거 같아서 임의로 튜닝
	else
		pH(2) = (Poc_z + gaze_point_robot.z);	// 카메라 or OpenFace가 조금 부정확한 거 같아서 임의로 튜닝

	return pH;
}

VectorXd EyeGaze(cv::Point3f eyecenter, cv::Point3f ec) {

}


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
	if (!moveDXLtoDesiredPosition(portHandler, packetHandler, dxl_goal_position, DXL_PROFILE_VELOCITY_HOMING)) { return 0; }

	std::this_thread::sleep_for(3s);

	deque<double> X; // 로봇 시점 X data 50개 slide window
	deque<double> Y; // 로봇 시점 Z data 50개 slide window
	double avg_X = 0;
	double avg_Y = 0;
	double var_X = 0;
	double var_Y;
	double std_X;
	double std_Y;

	double N = 50; // data set의 개수
	bool flag_first_cal = true;


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

		// 자연스러운 움직임을 위한
		int velocity;
		bool is_recovery = false; // 포지션 회복

		// OpenFace 돌리면서 gaze 방향에 따른 로봇 configuration 계산하는 thread
		auto open_face = [&]()
			{
				INFO_STREAM("open_face_start");

				bool flag_initial = true;

				std::chrono::high_resolution_clock::time_point threadstartTime;
				int threaditer = 0;

				while (!captured_image.empty()) {
					// default velocity 
					velocity = is_recovery ? 1000 : 40;
					is_recovery = false;

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
					cv::Point3f gaze_point_cam;
					cv::Point3f gaze_point_robot;

					// solution vector
					std::vector<double> x(2);
					std::vector<double> y(2);
					std::vector<double> z(2);

					// get gaze direction
					cv::Point3f e1 = gazeDirection0;
					cv::Point3f e2 = gazeDirection1;
					cv::Point3f ec = (e1 + e2) / 2;



					double d = 300;
					// 로봇 눈위치에서 계산하기 위해 EYE LEYER의 높이를 SUM
					double upperplane = robot_to_cam(0.0, 0.0, EYE_LAYER_HEIGHT + d)(1);
					double lowerplane = robot_to_cam(0.0, 0.0, EYE_LAYER_HEIGHT - d)(1);
					double rightplane = robot_to_cam(0.0, d, 0.0)(0);
					double leftplane = robot_to_cam(0.0, -d, 0.0)(0);

					double _gamma = 0;

					// upper, lower or left, right

						// right plane or left plane
					if (gazeAngle(0) < 0) {
						_gamma = (-P_c.x + rightplane) / ec.x;
					}
					else if (gazeAngle(0) > 0) {
						_gamma = (-P_c.x + leftplane) / ec.x;
					}
					else {
						cout << "gamma is undefined" << endl;
					}

					cv::Point3f temp = P_c + _gamma * ec;
					if (lowerplane < temp.y || temp.y < upperplane)
						gamma = 0;

					gaze_point_cam = P_c + _gamma * ec;

					// 10cm 정도 떨어진 지점. 
					if (100 < gaze_point_cam.z && gaze_point_cam.z < P_c.z - 100) { // 시선공유 해야하는 경우  rotation matrix
						gaze_point_robot.x = gaze_point_cam.z;
						gaze_point_robot.y = -gaze_point_cam.x;
						gaze_point_robot.z = -gaze_point_cam.y;
						// 50개 이전에는 분산을 구할 수 없음.
						if (X.size() < N && Y.size() < N && flag_first_cal) {
							X.push_back(radian_to_degree(gazeAngle(0)));
							Y.push_back(radian_to_degree(gazeAngle(1)));

							cout << radian_to_degree(gazeAngle(1)) << endl;

							// 시선공유 무시
							gaze_point_robot.x = P_c.z;
							gaze_point_robot.y = -P_c.x;
							gaze_point_robot.z = -P_c.y;
						}
						// 처음 50개의 데이터가 들어왔을 때
						else if (X.size() == N && Y.size() == N && flag_first_cal) {
							velocity = 500;
							flag_first_cal = false;
							// initial mean

							avg_X = deque_mean(X);
							avg_Y = deque_mean(Y);

							var_X = deque_variance(X);
							var_Y = deque_variance(Y);
						}
						else {
							// gaze 하는 과정에서 조금 스무스한 움직임을 줘야함. 
							velocity = 500;
							double X_kn = Y.front();
							Y.pop_front();
							double X_k = gazeAngle(1);
							Y.push_back(X_k);

							double pre_avg = avg_Y;
							double pre_val = var_Y;

							avg_Y = Moving_Avg_Filter(X_k, pre_avg, X_kn, N);
							var_Y = recursive_Variance(X_k, pre_avg, pre_val, X_kn, N);
						}

					}
					else { // Gaze 해야하는 경우 rotation matrix
						gaze_point_robot.x = P_c.z;
						gaze_point_robot.y = -P_c.x;
						gaze_point_robot.z = -P_c.y;
						// 다시 초기값으로 돌아가기.
						flag_first_cal = true;
						while (!Y.empty()) {
							X.pop_back();
							Y.pop_back();
						}
					}

					pH(0) = Poc_x + gaze_point_robot.x;
					pH(1) = (Poc_y + gaze_point_robot.y);		// 카메라 or OpenFace가 조금 부정확한 거 같아서 임의로 튜닝

					if (Poc_z + gaze_point_robot.z > 0)
						pH(2) = (Poc_z + gaze_point_robot.z);	// 카메라 or OpenFace가 조금 부정확한 거 같아서 임의로 튜닝
					else
						pH(2) = (Poc_z + gaze_point_robot.z);	// 카메라 or OpenFace가 조금 부정확한 거 같아서 임의로 튜닝
					
					xyz_f = pH;
					RPY = findGaze_RPY_Gamma(xyz_f);	// 논문 Algorithm 1 (gaze 위치 -> 로봇 configuration 변환)
					//RPY = findGaze_RPY_Gamma(pH);
					roll = RPY(0);
					pitch = RPY(1);
					yaw = RPY(2);
					gamma = RPY(3);

					double threshold_var = PI / 6;
					
					// 2초동안 어딘가를 응시하고 있으면서, 그 분산이 크지 않다면, 
					if (var_Y < threshold_var && Y.size() == N && !flag_first_cal) {
						
						cout << "var_X : " << var_Y << endl;
						flag_first_cal = true;
						is_recovery = true;
						std::chrono::seconds stop_time = std::chrono::seconds(threaditer) % 3; // 정지 시간이 랜덤성있게. 
						std::this_thread::sleep_for(2s);
					}

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
					visualizer.SetObservationGaze(e1, e2, LandmarkDetector::CalculateAllEyeLandmarks(face_model), LandmarkDetector::Calculate3DEyeLandmarks(face_model, sequence_reader.fx, sequence_reader.fy, sequence_reader.cx, sequence_reader.cy), face_model.detection_certainty);
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
					while (std::chrono::high_resolution_clock::now() < awaketime) { std::this_thread::yield(); } // 현재 시간이 awaketime 보다 작으면 yield() -> thread 양보.
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

				double present_val_p, present_val_r, present_var_X;
				double threshold_grad, threshold_val_p, threshold_val_r, threshold_var_X, confidence_limit;

				threshold_grad = 0.40;
				threshold_val_p = MAX_PITCH;
				threshold_val_r = MAX_ROLL;
				threshold_var_X = MAX_YAW;
				confidence_limit = 0.75;

				bool firstRun = true;
				long long startTime, millisec_since_epoch1;

				int dxl_goal_position_justbefore[5] = { 0,0,0,0,0 }; // just in case if dxl values become zero when 'q' pressed

				std::chrono::high_resolution_clock::time_point threadstartTime;
				int threaditer = 0;

				while (!captured_image.empty()) {

					present_val_p = pitch;
					present_val_r = roll;
					present_var_X = yaw;

					//------------------------------- AVOID SUDDEN MOVEMENT (논문 6페이지 마지막 ~ 7페이지 시작 참조)
					//---------------------

					// calculate gradient
					if (iter >= delta_timestep) {
						// 0 ~ 2
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
						if (!moveDXLtoDesiredPosition(portHandler, packetHandler, dxl_goal_position, velocity)) return 0;

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