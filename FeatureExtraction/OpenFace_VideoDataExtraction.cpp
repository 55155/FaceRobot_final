
// OPENFACE ROLL PITCH YAW MOUTH FACE EXTRACTION

#include "MacrosAndFunctions_small.h"    

int extractFeatures(vector<string> arguments, string videofolderpath);

int main(int argc, char** argv)
{
	// video containing directory
	string videofolderpath = "D:/ex/test_cpp";
	//string videofolderpath = "D:/_data/facedata/FIV2/train-5/training80_51";

	std::vector<std::string> arguments;
	for (int i = 0; i < argc; ++i)
		arguments.push_back(std::string(argv[i]));

	// no arguments: output usage
	if (arguments.size() == 1) {
		std::cout << "For command line arguments see:" << std::endl;
		std::cout << " https://github.com/TadasBaltrusaitis/OpenFace/wiki/Command-line-arguments";
		return 0;
	}

	arguments.push_back("-f"); arguments.push_back(" ");

	vector<string> filepaths;
	for (const auto& entry : std::filesystem::directory_iterator(videofolderpath))
		filepaths.push_back(entry.path().u8string());

	for (int i = 0; i < filepaths.size(); i++) {
		cout << "-------------------------------------------------------------------------------------" << filepaths[i] << endl;
		arguments[arguments.size() - 1] = filepaths[i];

		int state = extractFeatures(arguments, videofolderpath);
		if (state == 1)
			break;
		//else if (state == 2)
		//	continue;
		Sleep(100);

		//if (extractFeatures(arguments, videofolderpath))
		//	break;
		//Sleep(100);
	}

	return 0;
}

int extractFeatures(vector<string> arguments, string videofolderpath)
{
	static int func_iter = 1;
		
	LandmarkDetector::FaceModelParameters det_parameters(arguments);
	LandmarkDetector::CLNF face_model(det_parameters.model_location);

	if (!face_model.loaded_successfully) {
		std::cout << "ERROR: Could not load the landmark detector" << std::endl;
		return 3;
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

	
	string videofilepath = arguments[arguments.size() - 1];
	videofilepath.erase(videofilepath.end() - 4, videofilepath.end()); // erase .mp4

	ofstream outData_openface;
	//outData_openface.open(videofilepath + "_features.csv", ios::app);
	outData_openface.open(videofilepath + ".csv", ios::app);
	//outData_openface.open(videofolderpath + "/../features/headfeature" + to_string(func_iter) + ".csv", ios::app);

	func_iter++;

	//bool toNext = false;
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

		INFO_STREAM("Starting extracting");

		// openface
		double roll, pitch, yaw, mouth;
		double roll_f, pitch_f, yaw_f, mouth_f;
		float confidence;
		float fc_x, fc_y, x_max, y_max; // face center in pixels
		float fc_mm_x, fc_mm_y, fc_mm_z; // face center in mm

		// filtering
		double present_val_p, present_val_r, present_val_y;
		double threshold_grad, threshold_val_p, threshold_val_r, threshold_val_y, confidence_limit;

		threshold_grad = 0.40; // 0.45
		threshold_val_p = 0.7; // pitch
		threshold_val_r = 0.8; // roll
		threshold_val_y = 1.0; // yaw, mouth 1.1
		confidence_limit = 0.75;

		int iter = 0;
		int delta_timestep = 3;
		bool flag_write = true;				// write CSV file
		bool previous_write_flag = true;	// flag_write in previous iteration
		bool flag_start_moving;				// when face starts to move from stop, then true
		std::vector<double> past_dxl_goal_positon(5); // pitch,rollr,rolll,yaw,mouth
		std::vector<double> past_values_pitch(delta_timestep), past_values_roll(delta_timestep), delta(2); // pitch, roll

		long long startTime, millisec_since_epoch1;
		bool firstRun = true;

		INFO_STREAM("open_face_start");

		bool flag_initial = true;
		while (!captured_image.empty()) {

			if (PauseQuitNext() == 1)
				return 1;
			//else if (PauseQuitNext() == 2)
			//	toNext = true;

			// Converting to grayscale
			cv::Mat_<uchar> grayscale_image = sequence_reader.GetGrayFrame();

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

			// https://github.com/TadasBaltrusaitis/OpenFace/wiki/Output-Format 
			// https://github.com/TadasBaltrusaitis/OpenFace/wiki/API-calls
			// size 136, [x1;x2;...xn;y1;y2...yn], n=68
			// face center x: all_landmarks(27), y: all_landmarks(27+68)
			// x max: sequence_reader.frame_width, y max: sequence_reader.frame_height
			cv::Mat1f all_landmarks = face_model.detected_landmarks;

			fc_x = all_landmarks(27);
			fc_y = all_landmarks(27 + 68);
			x_max = sequence_reader.frame_width;
			y_max = sequence_reader.frame_height;

			cv::Mat1f all_landmarks_mm = face_model.GetShape(sequence_reader.fx, sequence_reader.fy, sequence_reader.cx, sequence_reader.cy);

			fc_mm_x = all_landmarks_mm(27);
			fc_mm_y = all_landmarks_mm(27 + 68);
			fc_mm_z = all_landmarks_mm(27 + 68 + 68);

			roll = pose_estimate[5];
			pitch = pose_estimate[3];
			yaw = pose_estimate[4];

			// get an action unit 25 to control lip movement.
			face_analyser.PredictStaticAUsAndComputeFeatures(captured_image, face_model.detected_landmarks);

			auto aus_intensity = face_analyser.GetCurrentAUsReg();
			// 0과 1로 출력하려면 : auto aus_presence = face_analyser.GetCurrentAUsClass();
			// first가 이름, second가 값

			mouth = aus_intensity[14].second;
			if (mouth > 2.5) { mouth = 2.5; }
			else if (mouth < 0.7) { mouth = 0; }
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

			//if (toNext)
			//	character_press = 'q';

			// quit processing the current sequence (useful when in Webcam mode)
			if (character_press == 'q') {
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

			// Grabbing the next frame in the sequence
			captured_image = sequence_reader.GetNextFrame();

			/////////////////////////////////////////////////////////
			//// FILTERING
			/////////////////////////////////////////////////////////

			present_val_p = pitch;
			present_val_r = roll;
			present_val_y = yaw;

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
			if (iter >= delta_timestep * 10000) iter = delta_timestep;

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

			if (flag_write || firstRun) {
				vector<double> filteredRPYM = KalmanFiltering(roll, pitch, yaw, mouth);
				roll_f = filteredRPYM[0];
				pitch_f = filteredRPYM[1];
				yaw_f = filteredRPYM[2];
				mouth_f = filteredRPYM[3];
			}

			if (firstRun) {
				startTime = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
				millisec_since_epoch1 = startTime;
				firstRun = false;
			}
			else
				millisec_since_epoch1 = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();

			outData_openface << millisec_since_epoch1 - startTime << ","
				<< roll << "," << roll_f << "," << pitch << "," << pitch_f << "," << yaw << "," << yaw_f << "," << mouth << "," << mouth_f << ","
				<< fc_x << "," << fc_y << "," << x_max << "," << y_max << ","
				<< fc_mm_x << "," << fc_mm_y << "," << fc_mm_z
				<< endl;
		}

		INFO_STREAM("open_face_end");

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

	//if (toNext)
	//	return 2;

	return 0;
}