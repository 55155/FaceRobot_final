///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2017, Carnegie Mellon University and University of Cambridge,
// all rights reserved.
//
// ACADEMIC OR NON-PROFIT ORGANIZATION NONCOMMERCIAL RESEARCH USE ONLY
//
// BY USING OR DOWNLOADING THE SOFTWARE, YOU ARE AGREEING TO THE TERMS OF THIS LICENSE AGREEMENT.  
// IF YOU DO NOT AGREE WITH THESE TERMS, YOU MAY NOT USE OR DOWNLOAD THE SOFTWARE.
//
// License can be found in OpenFace-license.txt

//     * Any publications arising from the use of this software, including but
//       not limited to academic journal and conference publications, technical
//       reports and manuals, must cite at least one of the following works:
//
//       OpenFace 2.0: Facial Behavior Analysis Toolkit
//       Tadas Baltru큄aitis, Amir Zadeh, Yao Chong Lim, and Louis-Philippe Morency
//       in IEEE International Conference on Automatic Face and Gesture Recognition, 2018  
//
//       Convolutional experts constrained local model for facial landmark detection.
//       A. Zadeh, T. Baltru큄aitis, and Louis-Philippe Morency,
//       in Computer Vision and Pattern Recognition Workshops, 2017.    
//
//       Rendering of Eyes for Eye-Shape Registration and Gaze Estimation
//       Erroll Wood, Tadas Baltru큄aitis, Xucong Zhang, Yusuke Sugano, Peter Robinson, and Andreas Bulling 
//       in IEEE International. Conference on Computer Vision (ICCV),  2015 
//
//       Cross-dataset learning and person-specific normalisation for automatic Action Unit detection
//       Tadas Baltru큄aitis, Marwa Mahmoud, and Peter Robinson 
//       in Facial Expression Recognition and Analysis Challenge, 
//       IEEE International Conference on Automatic Face and Gesture Recognition, 2015 
//
///////////////////////////////////////////////////////////////////////////////


// FeatureExtraction.cpp : Defines the entry point for the feature extraction console application.

// Local includes
#include "LandmarkCoreIncludes.h"

#include <Face_utils.h>
#include <FaceAnalyser.h>
#include <GazeEstimation.h>
#include <RecorderOpenFace.h>
#include <RecorderOpenFaceParameters.h>
#include <SequenceCapture.h>
#include <Visualizer.h>
#include <VisualizationUtils.h>

#include "dynamixel_sdk.h"
#include <dense>
#include <string>
#include <fstream>
#include <iostream>
#include <thread>
#include <mutex>
#include <chrono>
#include <conio.h>
//#include <Eigen/Core>;

using namespace Eigen;
using namespace std;
using std::cout;
using std::endl;
using std::chrono::duration_cast;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::chrono::system_clock;
using namespace std::this_thread;     // sleep_for, sleep_until
using namespace std::chrono_literals; // ns, us, ms, s, h, etc.

// ========================================== DXL SETTINGS

// Control table address
#define ADDR_PRO_TORQUE_ENABLE          64                 // Control table address is different in Dynamixel model
#define ADDR_PRO_GOAL_POSITION          116
#define ADDR_PRO_PRESENT_POSITION       132
#define ADDR_PRO_PROFILE_VELOCITY		112
#define ADDR_PRO_PRESENT_CURRENT		126

// Data Byte Length
#define LEN_PRO_GOAL_POSITION           4
#define LEN_PRO_PRESENT_POSITION        4
#define LEN_PRO_PROFILE_VELOCITY		4
#define LEN_PRO_PRESENT_CURRENT			2

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

#define TORQUE_ENABLE                   1					// Value for enabling the torque
#define TORQUE_DISABLE                  0					// Value for disabling the torque

#define DXL_PROFILE_VELOCITY_HOMING		50
#define DXL_PROFILE_VELOCITY			0					// NO LIMIT

// ========================================== ROBOT SETTINGS

// Default setting
#define DXL1_ID                         1					// pitch
#define DXL2_ID                         2					// right(from observer)
#define DXL3_ID                         3					// left(from observer)
#define DXL4_ID                         4					// yaw
#define DXL5_ID                         5					// mouth

// starting positions of motors
#define default_PITCH					700//800
#define default_ROLLR					1300//1200
#define default_ROLLL					1250//1150
#define default_YAW						1550
#define default_MOUTH					2100//2300

// ending positions of motors; position increase -> string tension down
#define END_PITCH						default_PITCH + 500
#define END_ROLLR						default_ROLLR + 1000
#define END_ROLLL						default_ROLLL + 1000
#define END_YAW							default_YAW
#define END_MOUTH						default_MOUTH - 100

#define BAUDRATE                        57600				// bps?
#define DEVICENAME                      "COM6"				// Check which port is being used on your controller, ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

#define p_d								50					// pulley_diameter 40,50
#define ROBOT_HEIGHT					180					// small: 100, large: 180 (mm)
#define ROBOT_LAYERS					13					// number of layers
#define ROBOT_STRING_HOLE_RADIUS		50					// small: 25, large: 50 (mm) radius of string hole
#define ROBOT_YAW_GEAR_RATIO			2					// yaw gear ratio
#define ROBOT_MOUTH_TUNE				200					// mouth movement size
#define ROBOT_MOUTH_BACK_COMPENSATION	1.5					// 입에 대한 뒷쪽 보상(small: 1.2, large: 1.5)

// ========================================== ETC SETTINGS
#define PI								3.141592
#define FLAG_SAVE_DXL_PRESENT_POSITION	false
#define FLAG_SAVE_DXL_PRESENT_CURRENT	false

#ifndef CONFIG_DIR
#define CONFIG_DIR "~"
#endif

#define INFO_STREAM( stream ) \
std::cout << stream << std::endl

#define WARN_STREAM( stream ) \
std::cout << "Warning: " << stream << std::endl

#define ERROR_STREAM( stream ) \
std::cout << "Error: " << stream << std::endl

static void printErrorAndAbort(const std::string& error){ std::cout << error << std::endl; }
#define FATAL_STREAM( stream ) \
printErrorAndAbort( std::string( "Fatal error: " ) + stream )

// FUNCTIONS
void setDXLGoalPosition(int dxl_goal_position[], int pitch, int rollr, int rolll, int yaw, int mouth);
cv::Mat getInitialFaceLandmarks();


vector<double> KalmanFiltering(double roll, double pitch, double yaw, double mouth)
{
	static bool first = true;
	static MatrixXd A(6, 6), H(3, 6), Q(6, 6), R(3, 3), x(6, 1), P(6, 6), xp(6, 1), Pp(6, 6), K(6, 3), z(3, 1);				// roll pitch yaw
	static MatrixXd Am(2, 2), Hm(1, 2), Qm(2, 2), Rm(1, 1), xm(2, 1), Pm(2, 2), xpm(2, 1), Ppm(2, 2), Km(2, 1), zm(1, 1);	// mouth
	double dt = 0.032;

	if (first) {
		// A,H,Q,R: constant
		A << 1, dt, 0, 0, 0, 0,
			0, 1, 0, 0, 0, 0,
			0, 0, 1, dt, 0, 0,
			0, 0, 0, 1, 0, 0,
			0, 0, 0, 0, 1, dt,
			0, 0, 0, 0, 0, 1;
		H << 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0;
		Q = 5 * MatrixXd::Identity(6, 6);
		R = 100 * MatrixXd::Identity(3, 3);  //50

		Am << 1, dt, 0, 1;
		Hm << 1, 0;
		Qm = 5 * MatrixXd::Identity(2, 2);
		Rm = 300 * MatrixXd::Identity(1, 1);

		// x,P,xp,Pp,K,z: change
		x.setZero();
		P = 200 * MatrixXd::Identity(6, 6);

		xm.setZero();
		Pm = 200 * MatrixXd::Identity(2, 2);

		first = false;
	}

	//filtering pitch,yaw,roll values
	xp = A * x;
	Pp = A * P * A.transpose() + Q;
	K = Pp * H.transpose() * (H * Pp * H.transpose() + R).inverse();
	z << roll, pitch, yaw;
	x = xp + K * (z - H * xp);
	P = Pp - K * H * Pp;

	double roll_f = x(0, 0);	// -0.8 ~ +0.8
	double pitch_f = x(2, 0);	// -0.7 ~ +0.7
	double yaw_f = x(4, 0);	// -1.0 ~ +1.0

	// filtering mouth values
	xpm = Am * xm;
	Ppm = Am * Pm * Am.transpose() + Qm;
	Km = Ppm * Hm.transpose() * (Hm * Ppm * Hm.transpose() + Rm).inverse();
	zm << mouth;
	xm = xpm + Km * (zm - Hm * xpm);
	Pm = Ppm - Km * Hm * Ppm;

	double mouth_f = xm(0, 0);	// 0 ~ 2.5

	vector<double> filtered_rpym(4);
	filtered_rpym[0] = roll_f;
	filtered_rpym[1] = pitch_f;
	filtered_rpym[2] = yaw_f;
	filtered_rpym[3] = mouth_f;

	return filtered_rpym;
}



void to_DXL_param(uint8_t output[], int input);
bool moveDXLtoDesiredPosition(dynamixel::GroupSyncWrite& groupSyncWriteVelocity, dynamixel::GroupSyncWrite& groupSyncWritePosition, int velocity, int dxl_goal_position[]);
bool moveDXLtoDesiredPosition_NoVelLimit(dynamixel::PacketHandler* packetHandler, dynamixel::GroupSyncWrite& groupSyncWritePosition, int dxl_goal_position[]);
vector<int> RPY2DXL(double roll_f, double pitch_f, double yaw_f , double mouth_f, int mode);
int KeyBoardStopMove(int calculatedDXL[], int afterKeyboard[]);
//int KeyBoardStopMove(int calculatedDXL[], int afterKeyboard[], double rpym_present[], double rpym_whenStopped[]);
//vector<bool> KeyBoardControl(double& roll, double& pitch, double& yaw, double& mouth, bool isStop);

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
	outData_openface.open("_CSV/" + year + month + date + ";" + hour + "-" + min + "-" + second + " openface" + ".csv", ios::app);
	if(FLAG_SAVE_DXL_PRESENT_POSITION)
		outData_position.open("_CSV/"+year + month + date + ";" + hour + "-" + min + "-" + second + " pos.csv", ios::app);	
	if(FLAG_SAVE_DXL_PRESENT_CURRENT)
		outData_current.open("_CSV/" + year + month + date + ";" + hour + "-" + min + "-" + second + " current" + ".csv", ios::app);


	
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
			while (!captured_image.empty()) {

				if (!FLAG_SAVE_DXL_PRESENT_POSITION && !FLAG_SAVE_DXL_PRESENT_CURRENT)		// FALSE FALSE
					std::this_thread::sleep_for(0ms);
				else if (FLAG_SAVE_DXL_PRESENT_POSITION && FLAG_SAVE_DXL_PRESENT_CURRENT)	// TRUE TRUE
					std::this_thread::sleep_for(20ms);
				else																		// TRUE FALSE
					std::this_thread::sleep_for(0ms);

				cout << "openface thread---------------" << endl;
							
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

				if (flag_initial) {
					face_model.detected_landmarks = getInitialFaceLandmarks();
					flag_initial = false;
				}

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

		// -------------------- KALMAN FILTERING VARIABLES
		// ---------------

		double roll_f, pitch_f, yaw_f, mouth_f; // filtered roll,pitch,yaw,mouth values
		//MatrixXd A(6, 6), H(3, 6), Q(6, 6), R(3, 3), x(6, 1), P(6, 6), xp(6, 1), Pp(6, 6), K(6, 3), z(3, 1);			// roll pitch yaw
		//MatrixXd Am(2, 2), Hm(1, 2), Qm(2, 2), Rm(1, 1), xm(2, 1), Pm(2, 2), xpm(2, 1), Ppm(2, 2), Km(2, 1), zm(1, 1);	// mouth
		//
		//double dt = 0.032;
		//// A,H,Q,R: constant
		//A << 1, dt, 0, 0, 0, 0,
		//	0, 1, 0, 0, 0, 0,
		//	0, 0, 1, dt, 0, 0,
		//	0, 0, 0, 1, 0, 0,
		//	0, 0, 0, 0, 1, dt,
		//	0, 0, 0, 0, 0, 1;
		//H << 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0;
		//Q = 5 * MatrixXd::Identity(6, 6);
		//R = 100 * MatrixXd::Identity(3, 3);  //50

		//Am << 1, dt, 0, 1;
		//Hm << 1, 0;
		//Qm = 5 * MatrixXd::Identity(2, 2);
		//Rm = 300 * MatrixXd::Identity(1, 1);

		//// x,P,xp,Pp,K,z: change
		//x.setZero();
		//P = 200 * MatrixXd::Identity(6, 6);

		//xm.setZero();
		//Pm = 200 * MatrixXd::Identity(2, 2);

		// ---------------
		// -------------------- KALMAN FILTERING VARIABLES

		/// -------------------------------------------------------------------------------------------------------------------------------------------------
		/// -------------------------------------------------------------------------------------------------------------------------------------------------
		/// -------------------------------------------------------------------------------------------------------------------------------------------------
		 
		//bool first_Run = true;
		int run_iteration = 1;
		int isStopped = 0;
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
						
			while (!captured_image.empty()) {	

				if (!FLAG_SAVE_DXL_PRESENT_POSITION && !FLAG_SAVE_DXL_PRESENT_CURRENT)		// FALSE FALSE
					std::this_thread::sleep_for(22ms);
				else if (FLAG_SAVE_DXL_PRESENT_POSITION && FLAG_SAVE_DXL_PRESENT_CURRENT)	// TRUE TRUE
					std::this_thread::sleep_for(0ms);
				else																		// TRUE FALSE
					std::this_thread::sleep_for(0ms);
			
				cout << "dynamixel thread running" << endl;

				// Work out the pose of the head from the tracked model
				auto millisec_since_epoch1 = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
				
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

				//---------------------
				//------------------------------- AVOID SUDDEN MOVEMENT

				if (flag_write || run_iteration == 1) {

					////filtering mouth values
					//xpm = Am * xm;
					//Ppm = Am * Pm * Am.transpose() + Qm;
					//Km = Ppm * Hm.transpose() * (Hm * Ppm * Hm.transpose() + Rm).inverse();
					//zm << mouth;
					//xm = xpm + Km * (zm - Hm * xpm);
					//Pm = Ppm - Km * Hm * Ppm;

					//mouth_f = xm(0, 0);	// 0 ~ 2.5

					////filtering pitch,yaw,roll values
					//xp = A * x;
					//Pp = A * P * A.transpose() + Q;
					//K = Pp * H.transpose() * (H * Pp * H.transpose() + R).inverse();
					//z << roll, pitch, yaw;
					//x = xp + K * (z - H * xp);
					//P = Pp - K * H * Pp;

					//roll_f = x(0, 0);	// -0.8 ~ +0.8
					//pitch_f = x(2, 0);	// -0.7 ~ +0.7
					//yaw_f = x(4, 0);	// -1.0 ~ +1.0



					vector<double> filteredRPYM = KalmanFiltering(roll, pitch, yaw, mouth);
					roll_f = filteredRPYM[0];
					pitch_f = filteredRPYM[1];
					yaw_f = filteredRPYM[2];
					mouth_f = filteredRPYM[3];


					// change roll pitch yaw mouth values to DXL positions
					int mode = 0; // mirroring
					//int mode = 1; // cloning
					vector<int> DXL = RPY2DXL(roll_f, pitch_f, yaw_f, mouth_f, mode);
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
					for (int k = 0; k < 5; k++)
						dxl_goal_position[k] = dxl_goal_position_keyboard[k];

					//double rpym_present[4] = { roll_f, pitch_f, yaw_f, mouth_f };					
					//isStopped = KeyBoardStopMove(dxl_goal_position, dxl_goal_position_keyboard, rpym_present, rpym_whenStopped);
					//if (!moveDXLtoDesiredPosition_NoVelLimit(packetHandler, groupSyncWritePosition, dxl_goal_position_keyboard)) return 0;
					
					if (!moveDXLtoDesiredPosition_NoVelLimit(packetHandler, groupSyncWritePosition, dxl_goal_position)) return 0;

					//if (isStopped) {	
					//	Sleep(30);
					//	vector<bool> output = KeyBoardControl(rpym_whenStopped[0], rpym_whenStopped[1], rpym_whenStopped[2], rpym_whenStopped[3], isStopped);
					//	if (output[2]) {
					//		vector<int> DXL_keyboard = RPY2DXL(rpym_whenStopped[0], rpym_whenStopped[1], rpym_whenStopped[2], rpym_whenStopped[3], mode);
					//		//for (int mm = 0; mm < 5; mm++)
					//		//	cout << DXL_keyboard[mm] << " ";
					//		//cout << endl;
					//		setDXLGoalPosition(dxl_goal_position_keyboard, DXL_keyboard[0], DXL_keyboard[1], DXL_keyboard[2], DXL_keyboard[3], DXL_keyboard[4]);
					//		if (!moveDXLtoDesiredPosition_NoVelLimit(packetHandler, groupSyncWritePosition, dxl_goal_position_keyboard)) return 0;
					//	}
					//}
					//else {
					//	if (!moveDXLtoDesiredPosition_NoVelLimit(packetHandler, groupSyncWritePosition, dxl_goal_position_keyboard)) return 0;
					//}

					if(run_iteration) run_iteration++;
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

					outData_position << millisec_since_epoch1 << "," << dxl_goal_position[0] << "," << dxl_present_position[0]
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

					outData_current << millisec_since_epoch1 << "," << dxl_present_current[0] << "," << dxl_present_current[1]
						<< "," << dxl_present_current[2] << "," << dxl_present_current[3] << "," << dxl_present_current[4] << endl;
				}

				// Clear syncwrite parameter storage
				groupSyncWritePosition.clearParam();
				
				outData_openface << millisec_since_epoch1 << "," << roll << "," << roll_f << "," << pitch << "," << pitch_f << ","
					<< yaw << "," << yaw_f << "," << mouth << "," << mouth_f << endl;

				if (isStopped == 2)
					stop = 'q';

				if (stop == 'q') {
					INFO_STREAM("dynamixel_control break");
					break;				
				}
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

	groupSyncWriteVelocity.clearParam();

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

cv::Mat getInitialFaceLandmarks() {
	float data[136] = { 271.69623, 271.82077, 273.89084, 277.28607, 281.68842, 288.05957, 295.75983, 305.14264, 316.90778, 329.39532,
	340.55182, 350.41235, 357.4267, 361.77673, 364.37564, 365.84381, 366.27979, 277.07129, 282.77597, 290.06531, 297.46561, 304.18289,
	325.82047, 333.80997, 341.86813, 349.25903, 354.44379, 315.28378, 314.66376, 314.09268, 313.52332, 305.48163, 309.8074, 314.3866,
	319.35226, 323.89697, 285.38028, 290.07968, 296.92252, 302.23816, 296.5549, 289.72995, 328.73456, 334.20651, 341.23352, 346.50391,
	341.97198, 335.08917, 298.36707, 303.95691, 309.7356, 314.16751, 318.98099, 325.9827, 332.49347, 326.05673, 319.34296, 314.05801,
	309.16208, 303.41394, 301.01834, 309.86325, 314.4035, 319.32504, 329.43152, 319.12863, 314.1265, 309.52582, 208.08917, 221.4265,
	234.70872, 247.66771, 259.24414, 269.22748, 276.93927, 282.35025, 283.43497, 281.82846, 275.47977, 267.10376, 256.85941, 245.26056,
	232.8992, 219.93524, 207.02707, 187.50021, 181.7841, 179.66052, 180.42511, 183.31708, 182.63174, 179.20944, 178.41438, 180.49646,
	186.0107, 197.87358, 208.03188, 218.02512, 228.08081, 234.92499, 236.61705, 238.02171, 236.70427, 235.03503, 201.43588, 196.92465,
	197.25784, 202.27583, 204.07004, 204.09869, 201.90338, 196.7272, 196.52003, 200.67773, 203.37827, 203.59331, 253.88109, 249.6516,
	247.41498, 248.73662, 247.49908, 249.85741, 253.50937, 258.86298, 260.81927, 261.22537, 260.68729, 258.59988, 253.9399, 252.87222,
	253.2009, 252.78207, 253.90419, 253.84804, 254.30095, 253.81267 };
	return cv::Mat(1, 136, CV_32F, data);
}

void setDXLGoalPosition(int out[], int in1, int in2, int in3, int in4, int in5)
{
	out[0] = in1;
	out[1] = in2;
	out[2] = in3;
	out[3] = in4;
	out[4] = in5;
}

void to_DXL_param(uint8_t output[], int input) 
{
	// Allocate motor control value into byte array for GroupSyncWrite (ex. goal position)	
	output[0] = DXL_LOBYTE(DXL_LOWORD(input));
	output[1] = DXL_HIBYTE(DXL_LOWORD(input));
	output[2] = DXL_LOBYTE(DXL_HIWORD(input));
	output[3] = DXL_LOBYTE(DXL_HIWORD(input));
}
	
bool moveDXLtoDesiredPosition(dynamixel::GroupSyncWrite& groupSyncWriteVelocity, dynamixel::GroupSyncWrite& groupSyncWritePosition, int velocity, int position[])
{
	uint8_t param_profile_velocity[4];
	int DXL_ID[5] = { DXL1_ID, DXL2_ID, DXL3_ID, DXL4_ID, DXL5_ID };

	to_DXL_param(param_profile_velocity, velocity);

	// Add Dynamixel profile velocity value to the Syncwrite storage
	for (int i = 0; i < 5; i++) {
		if (!groupSyncWriteVelocity.addParam(DXL_ID[i], param_profile_velocity)) {
			fprintf(stderr, "[ID:%03d] groupSyncWriteVelocity addparam failed", DXL_ID[i]);
			return false;
		}
	}

	// Syncwrite profile velocity
	int dxl_comm_result = groupSyncWriteVelocity.txPacket();

	//roll, pitch, yaw parameter goal position
	uint8_t param_zero_position_pitch[4], param_zero_position_rollr[4], param_zero_position_rolll[4], param_zero_position_yaw[4], param_zero_position_mouth[4];

	//Allocate goal position value into byte array for pitch, rollr, rolll, yaw, mouth
	to_DXL_param(param_zero_position_pitch, position[0]);
	to_DXL_param(param_zero_position_rollr, position[1]);
	to_DXL_param(param_zero_position_rolll, position[2]);
	to_DXL_param(param_zero_position_yaw, position[3]);
	to_DXL_param(param_zero_position_mouth, position[4]);

	// Add Dynamixel #1(pitch),#2(roll_right),#3(roll_left),#4(yaw),#5(mouth) goal position values to the Syncwrite parameter storage	
	if (!groupSyncWritePosition.addParam(DXL_ID[0], param_zero_position_pitch)) { fprintf(stderr, "[ID:%03d] groupSyncWritePosition addparam failed", DXL_ID[0]); return 0; }
	if (!groupSyncWritePosition.addParam(DXL_ID[1], param_zero_position_rollr)) { fprintf(stderr, "[ID:%03d] groupSyncWritePosition addparam failed", DXL_ID[1]); return 0; }
	if (!groupSyncWritePosition.addParam(DXL_ID[2], param_zero_position_rolll)) { fprintf(stderr, "[ID:%03d] groupSyncWritePosition addparam failed", DXL_ID[2]); return 0; }
	if (!groupSyncWritePosition.addParam(DXL_ID[3], param_zero_position_yaw)) { fprintf(stderr, "[ID:%03d] groupSyncWritePosition addparam failed", DXL_ID[3]); return 0; }
	if (!groupSyncWritePosition.addParam(DXL_ID[4], param_zero_position_mouth)) { fprintf(stderr, "[ID:%03d] groupSyncWritePosition addparam failed", DXL_ID[4]); return 0; }

	// Syncwrite goal position
	dxl_comm_result = groupSyncWritePosition.txPacket();

	// Clear syncwrite parameter storage
	groupSyncWriteVelocity.clearParam();
	groupSyncWritePosition.clearParam();

	to_DXL_param(param_profile_velocity, DXL_PROFILE_VELOCITY); // unlimit motor velocity

	// Add Dynamixel profile velocity value to the Syncwrite storage
	for (int i = 0; i < 5; i++) {
		if (!groupSyncWriteVelocity.addParam(DXL_ID[i], param_profile_velocity)) {
			fprintf(stderr, "[ID:%03d] groupSyncWriteVelocity addparam failed", DXL_ID[i]);
			return false;
		}
	}

	// Syncwrite goal velocity
	dxl_comm_result = groupSyncWriteVelocity.txPacket();

	return true;
}

bool moveDXLtoDesiredPosition_NoVelLimit(dynamixel::PacketHandler* packetHandler, dynamixel::GroupSyncWrite& groupSyncWritePosition, int dxl_goal_position[])
{
	int DXL_ID[5] = { DXL1_ID, DXL2_ID, DXL3_ID, DXL4_ID, DXL5_ID };

	//roll, pitch, yaw parameter goal position
	uint8_t param_goal_position_pitch[4], param_goal_position_rollr[4], param_goal_position_rolll[4], param_goal_position_yaw[4], param_goal_position_mouth[4];

	// Allocate goal position value into byte array for pitch, rollr, rolll, yaw, mouth
	to_DXL_param(param_goal_position_pitch, dxl_goal_position[0]);
	to_DXL_param(param_goal_position_rollr, dxl_goal_position[1]);
	to_DXL_param(param_goal_position_rolll, dxl_goal_position[2]);
	to_DXL_param(param_goal_position_yaw, dxl_goal_position[3]);
	to_DXL_param(param_goal_position_mouth, dxl_goal_position[4]);

	// Add Dynamixel #1~#5 goal position values to the Syncwrite parameter storage (pith,rollright,rollleft,yaw,mouth)
	if (!groupSyncWritePosition.addParam(DXL_ID[0], param_goal_position_pitch)) { fprintf(stderr, "[ID:%03d] groupSyncWritePosition addparam failed", DXL_ID[0]); return 0; }
	if (!groupSyncWritePosition.addParam(DXL_ID[1], param_goal_position_rollr)) { fprintf(stderr, "[ID:%03d] groupSyncWritePosition addparam failed", DXL_ID[1]); return 0; }
	if (!groupSyncWritePosition.addParam(DXL_ID[2], param_goal_position_rolll)) { fprintf(stderr, "[ID:%03d] groupSyncWritePosition addparam failed", DXL_ID[2]); return 0; }
	if (!groupSyncWritePosition.addParam(DXL_ID[3], param_goal_position_yaw)) { fprintf(stderr, "[ID:%03d] groupSyncWritePosition addparam failed", DXL_ID[3]); return 0; }
	if (!groupSyncWritePosition.addParam(DXL_ID[4], param_goal_position_mouth)) { fprintf(stderr, "[ID:%03d] groupSyncWritePosition addparam failed", DXL_ID[4]); return 0; }

	// Syncwrite goal position
	int dxl_comm_result = groupSyncWritePosition.txPacket();
	if (dxl_comm_result != COMM_SUCCESS) { printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result)); return false; }

	return true;
}

vector<int> RPY2DXL(double roll_f, double pitch_f, double yaw_f, double mouth_f, int mode)
{
	// CHANGE ROLL PITCH YAW MOUTH VALUES TO DXL POSITIONS

	//-------- change roll pitch yaw angle to string length L1 L2 L3
	//--------

	double yaw_degree = yaw_f * 180 / PI + 0.05;	// yaw
	float gamma = (float)(roll_f);					// roll
	float beta = (float)(pitch_f);					// pitch 

	MatrixXd T_X_gamma(4, 4), T_Y_beta(4, 4), T(4, 4);
	T_X_gamma << 1, 0, 0, 0, 0, cos(gamma), -sin(gamma), 0, 0, sin(gamma), cos(gamma), 0, 0, 0, 0, 1;	// x축을 기준으로 gamma 회전, roll
	T_Y_beta << cos(beta), 0, sin(beta), 0, 0, 1, 0, 0, -sin(beta), 0, cos(beta), 0, 0, 0, 0, 1;		// y축을 기준으로 beta 회전, pitch
	T = T_Y_beta * T_X_gamma;																			// 평행이동, roll, pitch 변환하는 행렬 

	MatrixXd Z_p(3, 1), Z_p_ext(4, 1), Z_n(4, 1), vec(4, 1);
	Z_p << 0, 0, 1;											//바닥면의 normal vector, z방향 유닛백터 
	Z_p_ext << 0, 0, 1, 1;
	vec << 0, 0, 0, 1;
	Z_n = T.inverse() * (Z_p_ext - vec);					//변환면 roll-pitch 후에 normal vector 
	double n1 = Z_n(0, 0), n2 = Z_n(1, 0), n3 = Z_n(2, 0);	//n1x + n2y + n3z + d = 0; 의 평면방정식을 가정 

	MatrixXd normalvec(1, 3);
	normalvec << n1, n2, n3;
	double theta = acos( (normalvec * Z_p).value() / (normalvec.norm() * Z_p.norm()) );
	double alpha = atan2(n2, n1);

	if (theta <= 0.00001) // theta 가 0이 되는 상황을 방지, 계산상으로 아주 작은 값을 넣어줌 
		theta = 0.001;

	double r = ROBOT_HEIGHT / theta;		// 문제, theta 가 0에 가까울수록, r이 커짐....
	double r_xy = r * cos(theta);

	double rcm_x = r * cos(alpha);			//  원격회전을 위한 중심
	double rcm_y = r * sin(alpha);

	double rx = rcm_x - r_xy * cos(alpha);	// 이동한 윗면의 중심 
	double ry = rcm_y - r_xy * sin(alpha);
	double rz = r * sin(theta);

	// (2) roll-pitch 가 될 때, 3개 실구멍의 위치 구하기
	double a = rx;	// roll-pitch에 의해 이동한 위치 x
	double b = ry;	// roll-pitch에 의해 이동한 위치 y
	double c = rz;	// roll-pitch에 의해 이동한 위치 z

	// 바닥면의 구멍과 방향백터 정의
	MatrixXd P1(3, 1), P2(3, 1), P3(3, 1);
	//, Q1(4, 1), Q2(4, 1), Q3(4, 1), E1(3, 1), E2(3, 1), E3(3, 1), abc(3, 1), P11(4, 1), P21(4, 1), P31(4, 1);
	P1 << ROBOT_STRING_HOLE_RADIUS * cos(0),		  ROBOT_STRING_HOLE_RADIUS* sin(0),			 0; // 1번 구멍의 바닥위치
	P2 << ROBOT_STRING_HOLE_RADIUS * cos(2 * PI / 3), ROBOT_STRING_HOLE_RADIUS* sin(2 * PI / 3), 0; // 2번 구멍의 바닥위치
	P3 << ROBOT_STRING_HOLE_RADIUS * cos(4 * PI / 3), ROBOT_STRING_HOLE_RADIUS* sin(4 * PI / 3), 0; // 3번 구멍의 바닥위치
	//P11 << P1, 1; //padding
	//P21 << P2, 1;
	//P31 << P3, 1;
	//abc << a, b, c;
	//Q1 << (T.inverse() * P11);
	//E1 << (Q1.block(0, 0, 3, 1) + abc);
	//Q2 << (T.inverse() * P21);
	//E2 << (Q2.block(0, 0, 3, 1) + abc);
	//Q3 << (T.inverse() * P31);
	//E3 << (Q3.block(0, 0, 3, 1) + abc);

	//실의 길이
	MatrixXd R(1, 2), u_rcm(1, 2), P1R(1, 2), P2R(1, 2), P3R(1, 2), P1R_1(1, 2), P2R_1(1, 2), P3R_1(1, 2);
	R << rcm_x, rcm_y;

	u_rcm << cos(alpha), sin(alpha);
	P1R_1 << P1(0), P1(1);
	P2R_1 << P2(0), P2(1);
	P3R_1 << P3(0), P3(1);

	P1R << (P1R_1 - R);
	P2R << (P2R_1 - R);
	P3R << (P3R_1 - R);

	double r1 = (P1R * u_rcm.transpose()).value();
	double r2 = (P2R * u_rcm.transpose()).value();
	double r3 = (P3R * u_rcm.transpose()).value();

	double L1 = (ROBOT_LAYERS * abs(r1) * (theta / ROBOT_LAYERS)); //앞쪽(DXL#1)
	double L2 = (ROBOT_LAYERS * abs(r2) * (theta / ROBOT_LAYERS)); //오른쪽(관찰자 시점//DXL#2)
	double L3 = (ROBOT_LAYERS * abs(r3) * (theta / ROBOT_LAYERS)); //왼쪽(관찰자 시점//DXL#3)

	//--------- string length to DXL position
	//---------

	double dxl_goal_position_pitch_double, dxl_goal_position_rollr_double, dxl_goal_position_rolll_double, dxl_goal_position_yaw_double, dxl_goal_position_mouth_double;
		
	if (mode == 0) // mirroring
	{
		dxl_goal_position_pitch_double = default_PITCH - (L1 - ROBOT_HEIGHT) * (4096 / (p_d * PI));

		double delta_mouth = (float)mouth_f * ROBOT_MOUTH_TUNE; // 입 크기180

		dxl_goal_position_rollr_double = default_ROLLR - (L2 - ROBOT_HEIGHT) * (4096 / (p_d * PI)) - (delta_mouth / ROBOT_MOUTH_BACK_COMPENSATION);			
		dxl_goal_position_rolll_double = default_ROLLL - (L3 - ROBOT_HEIGHT) * (4096 / (p_d * PI)) - (delta_mouth / ROBOT_MOUTH_BACK_COMPENSATION);
		
		//// R, L이 너무 기울어졌을 때 입 보상 끄는거
		//if (dxl_goal_position_rollr_double < 200)
		//	dxl_goal_position_rollr_double += (delta_mouth / 1.5);
		//if (dxl_goal_position_rolll_double < 200)
		//	dxl_goal_position_rolll_double += (delta_mouth / 1.5);
		
		dxl_goal_position_mouth_double = default_MOUTH - delta_mouth - (default_PITCH - dxl_goal_position_pitch_double) / 2;
		dxl_goal_position_yaw_double = (-1) * static_cast<double> (yaw_degree) * ROBOT_YAW_GEAR_RATIO * 4096.0 / 360.0 + default_YAW;
	}
	else if (mode == 1) // cloning
	{
		dxl_goal_position_pitch_double = default_PITCH - (L1 - ROBOT_HEIGHT) * (4096 / (p_d * PI));
		double delta_mouth = (float)mouth_f * ROBOT_MOUTH_TUNE;

		// R L change
		dxl_goal_position_rolll_double = default_ROLLR - (L2 - ROBOT_HEIGHT) * (4096 / (p_d * PI)) - (delta_mouth / ROBOT_MOUTH_BACK_COMPENSATION);
		dxl_goal_position_rollr_double = default_ROLLL - (L3 - ROBOT_HEIGHT) * (4096 / (p_d * PI)) - (delta_mouth / ROBOT_MOUTH_BACK_COMPENSATION);

		dxl_goal_position_mouth_double = default_MOUTH - delta_mouth - (default_PITCH - dxl_goal_position_pitch_double) / 2;

		// yaw_degree reverse
		dxl_goal_position_yaw_double = static_cast<double> (yaw_degree) * ROBOT_YAW_GEAR_RATIO * 4096.0 / 360.0 + default_YAW;
	}

	vector<int> DXL(5);
	DXL[0] = (int)dxl_goal_position_pitch_double;
	DXL[1] = (int)dxl_goal_position_rollr_double;
	DXL[2] = (int)dxl_goal_position_rolll_double;
	DXL[3] = (int)dxl_goal_position_yaw_double;
	DXL[4] = (int)dxl_goal_position_mouth_double;

	return DXL;
}

//int KeyBoardStopMove(int calculatedDXL[], int afterKeyboard[], double rpym_present[], double rpym_whenStopped[])
int KeyBoardStopMove(int calculatedDXL[], int afterKeyboard[])
{
	static int mode_webcamTrackingStop = 0; // 0: tracking, 1: stop			
	static bool changedFromStop2Move = false;
	static int dxl_goal_position_WHENSTOP[5] = { 0,0,0,0,0 };

	if (_kbhit()) {
		char c = _getch();
		if (c == 's' || c == 'S') {
			if (mode_webcamTrackingStop == 0) {
				mode_webcamTrackingStop = 1;

				for (int k = 0; k < 5; k++)
					dxl_goal_position_WHENSTOP[k] = calculatedDXL[k];
				//for (int k = 0; k < 4; k++)
				//	rpym_whenStopped[k] = rpym_present[k];

				INFO_STREAM("ROBOT MODE CHANGED TO STOP");
			}
			else {
				mode_webcamTrackingStop = 0;
				changedFromStop2Move = true;
				INFO_STREAM("ROBOT MODE CHANGED TO MOVE");
			}
		}
		if (c == 'q' || c == 'Q') {
			return 2;
		}
	}

	if (mode_webcamTrackingStop == 0) // webcam tracking
	{
		if (changedFromStop2Move) // change from stop to move
		{
			for (int i = 0; i < 5; i++)
				calculatedDXL[i] = (calculatedDXL[i] + dxl_goal_position_WHENSTOP[i]) / 2;
			changedFromStop2Move = false;
			for (int i = 0; i < 5; i++)
				afterKeyboard[i] = calculatedDXL[i];
		}
		else // originally tracking
		{
			for (int i = 0; i < 5; i++)
				afterKeyboard[i] = calculatedDXL[i];
		}
	}
	else // stop
	{
		for (int i = 0; i < 5; i++)
			afterKeyboard[i] = dxl_goal_position_WHENSTOP[i];
	}

	return mode_webcamTrackingStop; // 0: tracking, 1: stop	
}

//vector<bool> KeyBoardControl(double& roll, double& pitch, double& yaw, double& mouth, bool isStop)
//{
//	bool hitKeyboard = false;
//	bool goHome = false;
//	bool exit = false;
//	static double yaw_step = 0.03;
//	static double pitch_step = 0.03;
//	static double roll_step = 0.03;
//	static double mouth_step = 0.08;
//
//	if (!isStop) {
//		vector<bool> out(3);
//		out[0] = false;
//		out[1] = false;
//		out[2] = false;
//		return out;
//	}
//
//	if (_kbhit()) {
//		hitKeyboard = true;
//
//		char c = _getch();
//
//		//if (c == 'q' || c == 'Q')
//		//	exit = true;
//		//if (c == 'h' || c == 'H') {
//		//	roll = 0; pitch = 0; yaw = 0;
//		//	goHome = true;
//		//	cout << "Going to home position..." << endl;
//		//}
//
//		// YAW
//		if (c == 'j' || c == 'J') {
//			if (yaw > -1.0) // left
//				yaw -= yaw_step;
//		}
//		if (c == 'l' || c == 'L') {
//			if (yaw < 1.0) // right
//				yaw += yaw_step;
//		}
//
//		// PITCH
//		if (c == 'i' || c == 'I') {
//			if (pitch > -0.7) // up
//				pitch -= pitch_step;
//		}
//		if (c == 'k' || c == 'K') {
//			if (pitch < 0.7) // down
//				pitch += pitch_step;
//		}
//
//		// ROLL
//		if (c == 'u' || c == 'U') {
//			if (roll < 0.8) // left
//				roll += roll_step;
//		}
//		if (c == 'o' || c == 'O') {
//			if (roll > -0.8) // right
//				roll -= roll_step;
//		}
//
//		//// MOUTH
//		//if (c == 's' || c == 'S') {
//		//	if (mouth < 2.2) // up
//		//		mouth += mouth_step;
//		//}
//		//if (c == 'a' || c == 'A') {
//		//	if (mouth > 0.0) // down
//		//		mouth -= mouth_step;
//		//}
//		//// SPEED
//		//if (c == 'x' || c == 'X') {
//		//	cout << "SPEED UP" << endl;
//		//	if (yaw_step <= 0.055)		yaw_step += 0.005;
//		//	if (pitch_step <= 0.055)	pitch_step += 0.005;
//		//	if (roll_step <= 0.055)		roll_step += 0.005;
//		//	if (mouth_step <= 0.13)		mouth_step += 0.01;
//		//	cout << "yaw_step   = " << yaw_step << endl;
//		//	cout << "pitch_step = " << pitch_step << endl;
//		//	cout << "roll_step  = " << roll_step << endl;
//		//	cout << "mouth_step = " << mouth_step << endl;
//		//}
//		//if (c == 'z' || c == 'Z') {
//		//	cout << "SPEED DOWN" << endl;
//		//	if (yaw_step >= 0.005)		yaw_step -= 0.005;
//		//	if (pitch_step >= 0.005)	pitch_step -= 0.005;
//		//	if (roll_step >= 0.005)		roll_step -= 0.005;
//		//	if (mouth_step >= 0.01)		mouth_step -= 0.01;
//		//	cout << "yaw_step   = " << yaw_step << endl;
//		//	cout << "pitch_step = " << pitch_step << endl;
//		//	cout << "roll_step  = " << roll_step << endl;
//		//	cout << "mouth_step = " << mouth_step << endl;
//		//}
//	}
//
//	vector<bool> output(3);
//	output[0] = goHome;
//	output[1] = exit;
//	output[2] = hitKeyboard;
//
//	return output;
//}