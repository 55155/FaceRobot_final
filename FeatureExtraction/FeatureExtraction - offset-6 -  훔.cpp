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
//#include <Eigen/Core>;
using namespace Eigen;


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

// Default setting
#define DXL1_ID                         1                  // 앞쪽
#define DXL2_ID                         2                  // 오른쪽(관찰자 시점)
#define DXL3_ID                         3                  // 왼쪽(관찰자 시점)
#define DXL4_ID                         4                 // yaw
#define DXL5_ID                         5               // mouth

// MOTOR POSITION DECREASE -> TENSION UP
#define default_PITCH					800 // 2300
#define default_ROLLR					1250 // 1500
#define default_ROLLL					1200 // 1650
#define default_YAW						1550 // 1400
#define default_MOUTH					2500 // 1300

#define BAUDRATE                        57600
#define DEVICENAME                      "COM6"				 // Check which port is being used on your controller, ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
#define TORQUE_ENABLE                   1					 // Value for enabling the torque
#define TORQUE_DISABLE                  0					 // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE      0					 // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      4095				 // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MINIMUM_PROFILE_VELOCITY	0
#define DXL_MAXIMUM_PROFILE_VELOCITY	32767
#define DXL_MOVING_STATUS_THRESHOLD     1					 // Dynamixel moving status threshold 

#define ESC_ASCII_VALUE                 0x1b
#define PROFILE_VELOCITY_DEFAULT		0
#define p_d								50					// pulley_diameter 30, 40, 55
using std::cout;
using std::endl;

#ifndef CONFIG_DIR
#define CONFIG_DIR "~"
#endif

#define INFO_STREAM( stream ) \
std::cout << stream << std::endl

#define WARN_STREAM( stream ) \
std::cout << "Warning: " << stream << std::endl

#define ERROR_STREAM( stream ) \
std::cout << "Error: " << stream << std::endl
using std::chrono::duration_cast;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::chrono::system_clock;
using namespace std;
static void printErrorAndAbort(const std::string& error)
{
	std::cout << error << std::endl;
}

#define FATAL_STREAM( stream ) \
printErrorAndAbort( std::string( "Fatal error: " ) + stream )

std::vector<std::string> get_arguments(int argc, char** argv)
{

	std::vector<std::string> arguments;

	// First argument is reserved for the name of the executable
	for (int i = 0; i < argc; ++i)
	{
		arguments.push_back(std::string(argv[i]));
	}
	return arguments;
}

int main(int argc, char** argv)
{


	std::vector<std::string> arguments = get_arguments(argc, argv);

	// no arguments: output usage
	if (arguments.size() == 1)
	{
		std::cout << "For command line arguments see:" << std::endl;
		std::cout << " https://github.com/TadasBaltrusaitis/OpenFace/wiki/Command-line-arguments";
		return 0;
	}

	// Load the modules that are being used for tracking and face analysis
	// Load face landmark detector
	LandmarkDetector::FaceModelParameters det_parameters(arguments);
	// Always track gaze in feature extraction
	LandmarkDetector::CLNF face_model(det_parameters.model_location);

	if (!face_model.loaded_successfully)
	{
		std::cout << "ERROR: Could not load the landmark detector" << std::endl;
		return 1;
	}

	// Load facial feature extractor and AU analyser
	FaceAnalysis::FaceAnalyserParameters face_analysis_params(arguments);
	//face_analysis_params.OptimizeForVideos();
	FaceAnalysis::FaceAnalyser face_analyser(face_analysis_params);

	if (!face_model.eye_model)
	{
		std::cout << "WARNING: no eye model found" << std::endl;
	}
	Utilities::SequenceCapture sequence_reader;

	// A utility for visualizing the results
	Utilities::Visualizer visualizer(arguments);

	// Tracking FPS for visualization
	Utilities::FpsTracker fps_tracker;
	// The sequence reader chooses what to open based on command line arguments provided

	if (face_analyser.GetAUClassNames().size() == 0 && face_analyser.GetAUClassNames().size() == 0)
	{
		std::cout << "WARNING: no Action Unit models found" << std::endl;
	}


	fps_tracker.AddFrame();
	// (1) Dynamixel setting (start)
	// Initialize PortHandler instance
	// Set the port path
	// Get methods and members of PortHandlerLinux or PortHandlerWindows
	dynamixel::PortHandler* portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

	// Initialize PacketHandler instance
	// Set the protocol version
	// Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
	dynamixel::PacketHandler* packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

	// Initialize GroupSyncWrite instance
	dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION);
	dynamixel::GroupSyncWrite groupSyncWriteVelocity(portHandler, packetHandler, ADDR_PRO_PROFILE_VELOCITY, LEN_PRO_PROFILE_VELOCITY);

	// Initialize Groupsyncread instance for Present Position
	dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
	dynamixel::GroupSyncRead groupSyncReadCurrent(portHandler, packetHandler, ADDR_PRO_PRESENT_CURRENT, LEN_PRO_PRESENT_CURRENT);



	int dxl_comm_result = COMM_TX_FAIL;               // Communication result
	bool dxl_addparam_result = false;                 // addParam result
	bool dxl_getdata_result = false;                  // GetParam result
	int dxl_goal_position[5] = { 0,0,0,0,0 };
	int32_t dxl_present_position[5] = { 0,0,0,0,0 };
	int16_t dxl_present_current[5] = { 0,0,0,0,0 };
	int dxl_profile_velocity_homing = 50;
	int dxl_profile_velocity = 0;
	uint8_t param_profile_velocity[4];
	uint8_t dxl_error = 0;        // Dynamixel error
	using namespace std::this_thread;     // sleep_for, sleep_until
	using namespace std::chrono_literals; // ns, us, ms, s, h, etc.
	using std::chrono::system_clock;
	time_t curr;
	struct tm* d;
	curr = time(NULL);
	d = localtime(&curr);
	ofstream outData;
	ofstream outData2;
	ofstream outData_current;
	string year = std::to_string(d->tm_year + 1900);
	string month = "0" + std::to_string(d->tm_mon + 1);
	string date = std::to_string(d->tm_mday);
	string hour = std::to_string(d->tm_hour);
	string min = std::to_string(d->tm_min);
	string second = std::to_string(d->tm_sec);
	outData.open(year + month + date + ";" + hour + "-" + min + "-" + second + " pos.csv", ios::app);
	outData2.open(year + month + date + ";" + hour + "-" + min + "-" + second + " openface" + ".csv", ios::app);
	outData_current.open(year + month + date + ";" + hour + "-" + min + "-" + second + " current" + ".csv", ios::app);
	int DXL_ID[5] = { DXL1_ID, DXL2_ID, DXL3_ID, DXL4_ID, DXL5_ID };


	
	// Add parameter storage for present position value
	for(int i=0; i<5; i++)
	{ 
		dxl_addparam_result = groupSyncRead.addParam(DXL_ID[i]);
		if (dxl_addparam_result != true)
		{
		fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", DXL_ID[i]);
		return 0;
		}
	}


	// Add parameter storage for present current value
	for (int i = 0; i < 5; i++)
	{
		dxl_addparam_result = groupSyncReadCurrent.addParam(DXL_ID[i]);
		if (dxl_addparam_result != true)
		{
			fprintf(stderr, "[ID:%03d] groupSyncReadCurrent addparam failed", DXL_ID[i]);
			return 0;
		}
	}
	

	// Open port
	if (portHandler->openPort())
	{
		printf("Succeeded to open the port!\n");
	}
	else
	{
		printf("Failed to open the port!\n");
		printf("Press any key to terminate...\n");
		//getch();
		return 0;
	}

	// Set port baudrate
	if (portHandler->setBaudRate(BAUDRATE))
	{
		printf("Succeeded to change the baudrate!\n");
	}
	else
	{
		printf("Failed to change the baudrate!\n");
		printf("Press any key to terminate...\n");
		//getch();
		return 0;
	}

	// Enable Dynamixel Torque
	for (int i = 0; i < 5; i++)
	{
		dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID[i], ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
		if (dxl_comm_result != COMM_SUCCESS)
		{
			printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
		}
		else if (dxl_error != 0)
		{
			printf("%s\n", packetHandler->getRxPacketError(dxl_error));
		}
		else
		{
			printf("Dynamixel#%d has been successfully connected \n", DXL_ID[i]);
		}
	}


	// Syncread present position
	dxl_comm_result = groupSyncRead.txRxPacket();
	if (dxl_comm_result != COMM_SUCCESS)
	{
		printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
	}
	for (int i = 0; i < 5; i++)
	{
		if (groupSyncRead.getError(DXL_ID[i], &dxl_error))
		{
			printf("[ID:%03d] %s\n", DXL_ID[i], packetHandler->getRxPacketError(dxl_error));
		}
	}


	// Check if groupsyncread data of Dynamixel is available
	for (int i = 0; i < 5; i++)
	{
		dxl_getdata_result = groupSyncRead.isAvailable(DXL_ID[i], ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
		if (dxl_getdata_result != true)
		{
			fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", DXL_ID[i]);
			return 0;
		}
	}
	
	
	// Get Dynamixel present position value
	for (int i = 0; i < 5; i++)
	{
		dxl_present_position[i] = groupSyncRead.getData(DXL_ID[i], ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
		printf("[ID:%03d] PresPos:%03d \n", DXL_ID[i], dxl_present_position[i]);
	}
	

	// Syncread present current
	dxl_comm_result = groupSyncReadCurrent.txRxPacket();
	if (dxl_comm_result != COMM_SUCCESS)
	{
		printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
	}
	for (int i = 0; i < 5; i++)
	{
		if (groupSyncReadCurrent.getError(DXL_ID[i], &dxl_error))
		{
			printf("[ID:%03d] %s\n", DXL_ID[i], packetHandler->getRxPacketError(dxl_error));
		}
	}
	

	// Check if groupsyncread current data of Dynamixel is available
	for (int i = 0; i < 5; i++)
	{
		dxl_getdata_result = groupSyncReadCurrent.isAvailable(DXL_ID[i], ADDR_PRO_PRESENT_CURRENT, LEN_PRO_PRESENT_CURRENT);
		if (dxl_getdata_result != true)
		{
			fprintf(stderr, "[ID:%03d] groupSyncReadCurrent getdata failed", DXL_ID[i]);
			return 0;
		}
	}


	// Get Dynamixel#1 present current value
	for (int i = 0; i < 5; i++)
	{
		dxl_present_current[i] = groupSyncReadCurrent.getData(DXL_ID[i], ADDR_PRO_PRESENT_CURRENT, LEN_PRO_PRESENT_CURRENT);
		printf("[ID:%03d] PresCurrent:%03d \n", DXL_ID[i], dxl_present_current[i]);
	}
	

	// Allocate profile velocity value into byte array
	param_profile_velocity[0] = DXL_LOBYTE(DXL_LOWORD(dxl_profile_velocity_homing));
	param_profile_velocity[1] = DXL_HIBYTE(DXL_LOWORD(dxl_profile_velocity_homing));
	param_profile_velocity[2] = DXL_LOBYTE(DXL_HIWORD(dxl_profile_velocity_homing));
	param_profile_velocity[3] = DXL_HIBYTE(DXL_HIWORD(dxl_profile_velocity_homing));

	// Add Dynamixel profile velocity value to the Syncwrite storage
	for (int i = 0; i < 5; i++)
	{
		dxl_addparam_result = groupSyncWriteVelocity.addParam(DXL_ID[i], param_profile_velocity);
		if (dxl_addparam_result != true)
		{
			fprintf(stderr, "[ID:%03d] groupSyncWriteVelocity addparam failed", DXL_ID[i]);
			return 0;
		}
	}

	// Syncwrite profile velocity
	dxl_comm_result = groupSyncWriteVelocity.txPacket();

	//roll, pitch, yaw parameter goal position
	uint8_t param_zero_position_pitch[4];
	uint8_t param_zero_position_rollr[4];
	uint8_t param_zero_position_rolll[4];
	uint8_t param_zero_position_yaw[4];
	uint8_t param_zero_position_mouth[4];

	//Allocate goal position value into byte array for pitch
	param_zero_position_pitch[0] = DXL_LOBYTE(DXL_LOWORD(default_PITCH));
	param_zero_position_pitch[1] = DXL_HIBYTE(DXL_LOWORD(default_PITCH));
	param_zero_position_pitch[2] = DXL_LOBYTE(DXL_HIWORD(default_PITCH));
	param_zero_position_pitch[3] = DXL_HIBYTE(DXL_HIWORD(default_PITCH));

	//Allocate goal position value into byte array for rollr
	param_zero_position_rollr[0] = DXL_LOBYTE(DXL_LOWORD(default_ROLLR));
	param_zero_position_rollr[1] = DXL_HIBYTE(DXL_LOWORD(default_ROLLR));
	param_zero_position_rollr[2] = DXL_LOBYTE(DXL_HIWORD(default_ROLLR));
	param_zero_position_rollr[3] = DXL_HIBYTE(DXL_HIWORD(default_ROLLR));

	//Allocate goal position value into byte array for rolll
	param_zero_position_rolll[0] = DXL_LOBYTE(DXL_LOWORD(default_ROLLL));
	param_zero_position_rolll[1] = DXL_HIBYTE(DXL_LOWORD(default_ROLLL));
	param_zero_position_rolll[2] = DXL_LOBYTE(DXL_HIWORD(default_ROLLL));
	param_zero_position_rolll[3] = DXL_HIBYTE(DXL_HIWORD(default_ROLLL));


	//Allocate goal position value into byte array for yaw
	param_zero_position_yaw[0] = DXL_LOBYTE(DXL_LOWORD(default_YAW));
	param_zero_position_yaw[1] = DXL_HIBYTE(DXL_LOWORD(default_YAW));
	param_zero_position_yaw[2] = DXL_LOBYTE(DXL_HIWORD(default_YAW));
	param_zero_position_yaw[3] = DXL_HIBYTE(DXL_HIWORD(default_YAW));

	//Allocate goal position value into byte array for mouth
	param_zero_position_mouth[0] = DXL_LOBYTE(DXL_LOWORD(default_MOUTH));
	param_zero_position_mouth[1] = DXL_HIBYTE(DXL_LOWORD(default_MOUTH));
	param_zero_position_mouth[2] = DXL_LOBYTE(DXL_HIWORD(default_MOUTH));
	param_zero_position_mouth[3] = DXL_HIBYTE(DXL_HIWORD(default_MOUTH));

	// Add Dynamixel#1 goal position value to the Syncwrite parameter storage
	dxl_addparam_result = groupSyncWrite.addParam(DXL1_ID, param_zero_position_pitch); //DXL_1 = pitch
	if (dxl_addparam_result != true)
	{
		fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL1_ID);
		return 0;
	}
	// Add Dynamixel#2 goal position value to the Syncwrite parameter storage
	dxl_addparam_result = groupSyncWrite.addParam(DXL2_ID, param_zero_position_rollr); //DXL_2 = roll_right
	if (dxl_addparam_result != true)
	{
		fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL2_ID);
		return 0;
	}
	// Add Dynamixel#3 goal position value to the Syncwrite parameter storage
	dxl_addparam_result = groupSyncWrite.addParam(DXL3_ID, param_zero_position_rolll); //DXL_3 = roll_left
	if (dxl_addparam_result != true)
	{
		fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL3_ID);
		return 0;
	}
	// Add Dynamixel#4 goal position value to the Syncwrite parameter storage
	dxl_addparam_result = groupSyncWrite.addParam(DXL4_ID, param_zero_position_yaw); //DXL_4 = yaw
	if (dxl_addparam_result != true)
	{
		fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL4_ID);
		return 0;
	}
	// Add Dynamixel#5 goal position value to the Syncwrite parameter storage
	dxl_addparam_result = groupSyncWrite.addParam(DXL5_ID, param_zero_position_mouth); //DXL_5 = mouth
	if (dxl_addparam_result != true)
	{
		fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL5_ID);
		return 0;
	}

	// Syncwrite goal position
	dxl_comm_result = groupSyncWrite.txPacket();

	// Clear syncwrite parameter storage
	groupSyncWriteVelocity.clearParam();
	groupSyncWrite.clearParam();


	// Allocate profile velocity value into byte array
	param_profile_velocity[0] = DXL_LOBYTE(DXL_LOWORD(dxl_profile_velocity));
	param_profile_velocity[1] = DXL_HIBYTE(DXL_LOWORD(dxl_profile_velocity));
	param_profile_velocity[2] = DXL_LOBYTE(DXL_HIWORD(dxl_profile_velocity));
	param_profile_velocity[3] = DXL_HIBYTE(DXL_HIWORD(dxl_profile_velocity));

	// Add Dynamixel profile velocity value to the Syncwrite storage
	for (int i = 0; i < 5; i++)
	{
		dxl_addparam_result = groupSyncWriteVelocity.addParam(DXL_ID[i], param_profile_velocity);
		if (dxl_addparam_result != true)
		{
			fprintf(stderr, "[ID:%03d] groupSyncWriteVelocity addparam failed", DXL_ID[i]);
			return 0;
		}
	}

	// Syncwrite goal position
	dxl_comm_result = groupSyncWriteVelocity.txPacket();


	while (true) // this is not a for loop as we might also be reading from a webcam
	{
		mutex mtx;


		if (!sequence_reader.Open(arguments))
			break;

		INFO_STREAM("Device or file opened");

		if (sequence_reader.IsWebcam())
		{
			INFO_STREAM("WARNING: using a webcam in feature extraction, Action Unit predictions will not be as accurate in real-time webcam mode");
			INFO_STREAM("WARNING: using a webcam in feature extraction, forcing visualization of tracking to allow quitting the application (press q)");
			visualizer.vis_track = true;
		}

		cv::Mat captured_image;

		Utilities::RecorderOpenFaceParameters recording_params(arguments, true, sequence_reader.IsWebcam(),
			sequence_reader.fx, sequence_reader.fy, sequence_reader.cx, sequence_reader.cy, sequence_reader.fps);
		if (!face_model.eye_model)
		{
			recording_params.setOutputGaze(false);
		}
		Utilities::RecorderOpenFace open_face_rec(sequence_reader.name, recording_params, arguments);

		if (recording_params.outputGaze() && !face_model.eye_model)
			std::cout << "WARNING: no eye model defined, but outputting gaze" << std::endl;

		captured_image = sequence_reader.GetNextFrame();

		// For reporting progress
		double reported_completion = 0;

		INFO_STREAM("Starting tracking");


		float roll, pitch, roll_f, pitch_f, mouth_f;
		double yaw, yaw_f;
		char stop;
		int first_Run = 0;
		float mouth = 0;
		MatrixXd A(6, 6), H(3, 6), Q(6, 6), R(3, 3), IMat6(6, 6), IMat3(3, 3), x(6, 1), P(6, 6), xp(6, 1), Pp(6, 6), K(6, 3), z(3, 1);
		MatrixXd Am(2, 2), Hm(1, 2), Qm(2, 2), Rm(1, 1), IMat2(2, 2), IMat1(1, 1), xm(2, 1), Pm(2, 2), xpm(2, 1), Ppm(2, 2), Km(2, 1), zm(1, 1);
		//double DXL_MOUTH_MAX = default_MOUTH;
		int count = 0, r = 0, first_Run_m = 0;
		float confidence_limit = 0.75;
		float confidence = confidence_limit; // added 220504 see LandmarkDetectorFunc.h and LandmarkDetectorFunc.cpp

		auto open_face = [&]()
		{
			
			INFO_STREAM("open_face_start");

			bool flag_initial = true;
			while (!captured_image.empty()) {
				//auto millisec_since_epoch_1 = std::chrono::high_resolution_clock::now();
				// Converting to grayscale
				/*auto millisec_since_epoch2 = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();*/

				cv::Mat_<uchar> grayscale_image = sequence_reader.GetGrayFrame();


				// The actual facial landmark detection / tracking				
				bool detection_success = LandmarkDetector::DetectLandmarksInVideo(captured_image, face_model, det_parameters, grayscale_image);
				//bool detection_success = LandmarkDetector::DetectLandmarksInVideo_withConfidence(captured_image, face_model, det_parameters, grayscale_image, confidence);

				
				// Gaze tracking, absolute gaze direction
				cv::Point3f gazeDirection0(0, 0, 0); cv::Point3f gazeDirection1(0, 0, 0); cv::Vec2d gazeAngle(0, 0);

				if (detection_success && face_model.eye_model)
				{
					GazeAnalysis::EstimateGaze(face_model, gazeDirection0, sequence_reader.fx, sequence_reader.fy, sequence_reader.cx, sequence_reader.cy, true);
					GazeAnalysis::EstimateGaze(face_model, gazeDirection1, sequence_reader.fx, sequence_reader.fy, sequence_reader.cx, sequence_reader.cy, false);
					gazeAngle = GazeAnalysis::GetGazeAngle(gazeDirection0, gazeDirection1);
				}

				// Do face alignment
				cv::Mat sim_warped_img;
				cv::Mat_<double> hog_descriptor; int num_hog_rows = 0, num_hog_cols = 0;
				/*mtx.lock();*/
				cv::Vec6d pose_estimate = LandmarkDetector::GetPose(face_model, sequence_reader.fx, sequence_reader.fy, sequence_reader.cx, sequence_reader.cy);
				/*mtx.unlock();*/
				roll = pose_estimate[5];
				pitch = pose_estimate[3];
				yaw = pose_estimate[4];

				if (flag_initial)
				{
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
					face_model.detected_landmarks = cv::Mat(1, 136, CV_32F, data);

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
				stop = character_press;
				// quit processing the current sequence (useful when in Webcam mode)
				if (character_press == 'q')
				{
					INFO_STREAM("break");
					break;
				}

				confidence = face_model.detection_certainty;

				// Reporting progress
				if (sequence_reader.GetProgress() >= reported_completion / 10.0)
				{
					std::cout << reported_completion * 10 << "% ";
					if (reported_completion == 10)
					{
						std::cout << std::endl;
					}
					reported_completion = reported_completion + 1;
				}


				//auto awake_time1 = millisec_since_epoch_1 + 150ms; //loop_time control //revision: 31.6, 40
				
				// Grabbing the next frame in the sequence
				captured_image = sequence_reader.GetNextFrame();
				//while (std::chrono::high_resolution_clock::now() < awake_time1) { std::this_thread::yield(); }
				/*outData << roll << "," << pitch << "," << yaw << endl;*/

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


		auto dynamixel_control = [&]()
		{
			INFO_STREAM("dynamixel_control_start");
			int iter = 0;
			int delta_timestep = 3;
			bool flag_write = true;
			std::vector<double> past_dxl_goal_positon(5); // pitch,rollr,rolll,yaw,mouth
			std::vector<double> past_values1(delta_timestep); // pitch
			std::vector<double> past_values2(delta_timestep); // yaw
			std::vector<double> past_values3(delta_timestep); // roll
			std::vector<double> delta(2);

			double r1, r2, r3, L1, L2, L3; // 이게 무슨 변수인지 설명

			bool flag_save_motor_present_current = true;
			bool flag_save_motor_present_position = false;
						
			while (!captured_image.empty()) {			
				//auto millisec_since_epoch_2 = std::chrono::high_resolution_clock::now();
				// Work out the pose of the head from the tracked model
				auto millisec_since_epoch1 = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();

				// mouth 
				if (first_Run_m == 0) {
					double dt = 0.032;
					Am << 1, dt, 0, 1;
					Hm << 1, 0;
					IMat2 << 1, 0, 0, 1;
					IMat1 << 1;
					Qm = IMat2 * 5;
					Rm = IMat1 * 300;
					xm << 0, 0;
					Pm = IMat2 * 200;
					first_Run_m = 1;
				}
				
				//roll, pitch, yaw dxl
				if (first_Run == 0)
				{
					double dt = 0.032;


					A << 1, dt, 0, 0, 0, 0,
						0, 1, 0, 0, 0, 0,
						0, 0, 1, dt, 0, 0,
						0, 0, 0, 1, 0, 0,
						0, 0, 0, 0, 1, dt,
						0, 0, 0, 0, 0, 1;
					H << 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0;
					IMat6 << 1, 0, 0, 0, 0, 0,
						0, 1, 0, 0, 0, 0,
						0, 0, 1, 0, 0, 0,
						0, 0, 0, 1, 0, 0,
						0, 0, 0, 0, 1, 0,
						0, 0, 0, 0, 0, 1;
					IMat3 << 1, 0, 0,
						0, 1, 0,
						0, 0, 1;
					Q = IMat6 * 5;
					R = IMat3 * 100;  //50
					x << 0, 0, 0, 0, 0, 0;
					P = IMat6 * 200;

					first_Run = 1;

				}


				double current_val1 = pitch;
				double current_val2 = roll;
				double current_val3 = yaw;

				if (iter >= delta_timestep)
				{
					delta[0] = current_val1 - past_values1[iter % delta_timestep];
					past_values1[iter % delta_timestep] = current_val1;
					delta[1] = current_val2 - past_values2[iter % delta_timestep];
					past_values2[iter % delta_timestep] = current_val2;
				
				}
				else {
					past_values1[iter % delta_timestep] = current_val1;
					past_values2[iter % delta_timestep] = current_val2;
					/*past_values3[iter % delta_timestep] = current_val3;*/
				}

				iter++;
				if (iter >= delta_timestep * 1000) {
					iter = 0;
				}

				double threshold_val = 0.45;
				bool flag_grad_all_smaller_than_thr = true;
				for (int mn = 0; mn < delta.size(); mn++) {
					if (abs(delta[mn]) > threshold_val) {
						flag_grad_all_smaller_than_thr = false;
						//if (flag_grad_all_smaller_than_thr == false) { cout << " flag_grad_all_smaller " << endl; }
						break;
					}
				}
				//cout << " confidence is " << confidence << endl;

				double threshold_val1 = 0.7; //pitch
				double threshold_val2 = 0.8; //roll
				double threshold_val3 = 1.1;	//yaw, mouth
				bool flag_val_all_smaller_than_thr = true;
				if (abs(current_val1) > threshold_val1) {
					flag_val_all_smaller_than_thr = false;
					//cout << " pitch flag " << endl;
				}
				if (abs(current_val2) > threshold_val2) {
					flag_val_all_smaller_than_thr = false;
					//cout << " roll flag " << endl;
				}
				if (abs(current_val3) > threshold_val3) {
					flag_val_all_smaller_than_thr = false;
					//cout << " yaw flag "<< yaw << endl;

				}

				//if (confidence < confidence_limit) { cout << " confidence flag "<< confidence << endl; }
				
				bool flag_start_moving = false;
				if (flag_val_all_smaller_than_thr && flag_grad_all_smaller_than_thr && confidence > confidence_limit) {
					bool pre_flag = flag_write;
					flag_write = true;
					if (flag_write != pre_flag) { flag_start_moving = true;  
					//cout << " startmoving " << endl;
					}
				}
				else {
					flag_write = false;
				}

				if (flag_write || first_Run == 1) {
					
					//std::chrono::steady_clock::time_point begin1 = std::chrono::steady_clock::now();

					//filtering mouth values
					xpm = Am * xm;
					Ppm = Am * Pm * (Am.transpose()) + Qm;
					Km = Ppm * (Hm.transpose()) * ((Hm * Ppm * (Hm.transpose()) + Rm).inverse());
					zm << mouth;
					xm = xpm + Km * (zm - Hm * xpm);
					Pm = Ppm - Km * Hm * Ppm;
					mouth_f = xm(0, 0);

					//filtering pitch,yaw,roll values
					xp = A * x;
					Pp = A * P * (A.transpose()) + Q;
					K = Pp * (H.transpose()) * ((H * Pp * (H.transpose()) + R).inverse());
					//std::cout << "((H * Pp * (H.transpose()) + R).size() = (" << (H * Pp * H.transpose() + R).rows() << "," << (H * Pp * H.transpose() + R).cols() <<")" << std::endl;
					z << roll, pitch, yaw;
					x = xp + K * (z - H * xp);
					P = Pp - K * H * Pp;
					roll_f = x(0, 0);
					pitch_f = x(2, 0);
					yaw_f = x(4, 0);

					//std::chrono::steady_clock::time_point end1 = std::chrono::steady_clock::now();
					//std::cout << "Filtering time = " << std::chrono::duration_cast<std::chrono::microseconds>(end1 - begin1).count() << "[mus]" << std::endl;

					double dry = (180 * yaw_f) / 3.141592 + 0.05;   //yaw
					// PART I: openface에서 넘어오는 roll-pitch 각도
					float gamma = (float)(roll_f); //roll
					float beta = (float)(pitch_f); //pitch 
					int h = 180; //작은 애: 100;// 큰 애: 180; // 로봇구조의 높이 mm
					int N = 13; //layer의 갯수
					/*std::cout << "roll: " << gamma << " pitch: " << beta;*/

					MatrixXd T_X_gamma(4, 4), T_Y_beta(4, 4), T(4, 4), Z_p(3, 1), Z_p_ext(4, 1), Z_n(4, 1), vec(4, 1);
					T_X_gamma << 1, 0, 0, 0, 0, cos(gamma), -sin(gamma), 0, 0, sin(gamma), cos(gamma), 0, 0, 0, 0, 1; //x축을 기준으로 gamma 회전, roll
					T_Y_beta << cos(beta), 0, sin(beta), 0, 0, 1, 0, 0, -sin(beta), 0, cos(beta), 0, 0, 0, 0, 1; // y축을 기준으로 beta 회전, pitch
					T = T_Y_beta * T_X_gamma; //평행이동, roll, pitch 변환하는 행렬 

					Z_p << 0, 0, 1; //바닥면의 normal vector, z방향 유닛백터 
					Z_p_ext << 0, 0, 1, 1;
					vec << 0, 0, 0, 1;
					Z_n = T.inverse() * Z_p_ext - T.inverse() * vec; //변환면 roll-pitch 후에 normal vector 
					double n1 = Z_n(0, 0), n2 = Z_n(1, 0), n3 = Z_n(2, 0); //n1x + n2y + n3z + d = 0; 의 평면방정식을 가정 

					MatrixXd normalvec(1, 3);
					normalvec << n1, n2, n3;
					double theta = acos((normalvec * Z_p).value() / (normalvec.norm() * Z_p.norm()));
					double alpha = atan2(n2, n1);

					if (theta <= 0.00001) // theta 가 0이 되는 상황을 방지, 계산상으로 아주 작은 값을 넣어줌 
						theta = 0.001;


					double r = h / theta; // 문제, theta 가 0에 가까울수록, r이 커짐....
					double r_xy = r * cos(theta);

					double rcm_x = r * cos(alpha); //  원격회전을 위한 중심
					double rcm_y = r * sin(alpha);

					double rx = rcm_x - r_xy * cos(alpha); // 이동한 윗면의 중심 
					double ry = rcm_y - r_xy * sin(alpha);
					double rz = r * sin(theta);

					// (2) roll-pitch 가 될 때, 3개 실구멍의 위치 구하기
					double a = rx; // roll-pitch에 의해 이동한 위치 x
					double b = ry; //roll-pitch에 의해 이동한 위치 y
					double c = rz; //roll-pitch에 의해 이동한 위치 z

					// 바닥면의 구멍과 방향백터 정의
					int d = 50; // 작은 애 25 큰 애 실구멍의 반경 50mm

					MatrixXd P1(3, 1), P2(3, 1), P3(3, 1), Q1(4, 1), Q2(4, 1), Q3(4, 1), E1(3, 1), E2(3, 1), E3(3, 1), abc(3, 1), P11(4, 1), P21(4, 1), P31(4, 1);
					P1 << d * cos(0), d* sin(0), 0; // 1번 구멍의 바닥위치
					P2 << d * cos(2 * (3.141592) / 3), d* sin(2 * (3.141592) / 3), 0; // 2번 구멍의 바닥위치
					P3 << d * cos(4 * (3.141592) / 3), d* sin(4 * (3.141592) / 3), 0; // 3번 구멍의 바닥위치
					P11 << P1, 1; //padding
					P21 << P2, 1;
					P31 << P3, 1;
					abc << a, b, c;
					Q1 << (T.inverse() * P11);
					E1 << (Q1.block(0, 0, 3, 1) + abc);
					Q2 << (T.inverse() * P21);
					E2 << (Q2.block(0, 0, 3, 1) + abc);
					Q3 << (T.inverse() * P31);
					E3 << (Q3.block(0, 0, 3, 1) + abc);

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

					r1 = (P1R * u_rcm.transpose()).value();
					r2 = (P2R * u_rcm.transpose()).value();
					r3 = (P3R * u_rcm.transpose()).value();

					L1 = (N * abs(r1) * (theta / N)); //앞쪽(DXL#1)
					L2 = (N * abs(r2) * (theta / N)); //오른쪽(관찰자 시점//DXL#2)
					L3 = (N * abs(r3) * (theta / N)); //왼쪽(관찰자 시점//DXL#3)

					// 값 바꿔주는거

					double dxl_goal_position_pitch_double = default_PITCH - (L1 - h) * (4096 / (p_d * 3.141592)); //작은 애: 2048- ~~ 
					double delta_mouth = (float)mouth_f * 300; // 입 크기180
					double dxl_goal_position_rollr_double = default_ROLLR - (L2 - h) * (4096 / (p_d * 3.141592)) - (delta_mouth / 1.5); //작은 애1.2 // 입에 대한 뒷쪽 보상
					if (dxl_goal_position_rollr_double < 200) {
						dxl_goal_position_rollr_double = default_ROLLR - (L2 - h) * (4096 / (p_d * 3.141592));
					}
					double dxl_goal_position_rolll_double = default_ROLLL - (L3 - h) * (4096 / (p_d * 3.141592)) - (delta_mouth / 1.5); //작은 애1.2 // 입에 대한 뒷쪽 보상
					// R L 이 너무 기울어졌을 때 입 보상 끄는거
					//if (dxl_goal_position_rolll_double < 200) {
					//	dxl_goal_position_rolll_double = default_ROLLL - (L3 - h) * (4096 / (p_d * 3.141592));
					//}
					double dxl_goal_position_mouth_double = default_MOUTH + (-1) * delta_mouth + (-1) * (default_PITCH - dxl_goal_position_pitch_double) / 2;
					double dxl_goal_position_yaw_double = (-1) * static_cast<double> (dry) * 2 * 11.377 + default_YAW; //작은 애: 2*11.377


					//roll, pitch, yaw goal position
					dxl_goal_position[0] = dxl_goal_position_pitch_double;
					dxl_goal_position[1] = dxl_goal_position_rollr_double;
					dxl_goal_position[2] = dxl_goal_position_rolll_double;
					dxl_goal_position[3] = dxl_goal_position_yaw_double;
					dxl_goal_position[4] = dxl_goal_position_mouth_double;
					//int dxl_goal_position[5] = { dxl_goal_position_pitch_double, dxl_goal_position_rollr_double, dxl_goal_position_rolll_double,
					//	dxl_goal_position_yaw_double, dxl_goal_position_mouth_double };  //pitch, rollr, rolll, yaw mouth

					if (flag_start_moving) {
						//cout << "reflect past valuse" << endl;
						for (int i = 0; i < 5; i++)
						{
							dxl_goal_position[i] = (dxl_goal_position[i] + past_dxl_goal_positon[i]) / 2;
						}
					}

					for (int i = 0; i < 5; i++)
					{
						past_dxl_goal_positon[i] = dxl_goal_position[i];
					}

					//roll, pitch, yaw parameter goal position
					uint8_t param_goal_position_pitch[4];
					uint8_t param_goal_position_rollr[4];
					uint8_t param_goal_position_rolll[4];
					uint8_t param_goal_position_yaw[4];
					uint8_t param_goal_position_mouth[4];

					//Allocate goal position value into byte array for pitch
					param_goal_position_pitch[0] = DXL_LOBYTE(DXL_LOWORD(dxl_goal_position[0]));
					param_goal_position_pitch[1] = DXL_HIBYTE(DXL_LOWORD(dxl_goal_position[0]));
					param_goal_position_pitch[2] = DXL_LOBYTE(DXL_HIWORD(dxl_goal_position[0]));
					param_goal_position_pitch[3] = DXL_HIBYTE(DXL_HIWORD(dxl_goal_position[0]));

					//Allocate goal position value into byte array for rollr
					param_goal_position_rollr[0] = DXL_LOBYTE(DXL_LOWORD(dxl_goal_position[1]));
					param_goal_position_rollr[1] = DXL_HIBYTE(DXL_LOWORD(dxl_goal_position[1]));
					param_goal_position_rollr[2] = DXL_LOBYTE(DXL_HIWORD(dxl_goal_position[1]));
					param_goal_position_rollr[3] = DXL_HIBYTE(DXL_HIWORD(dxl_goal_position[1]));

					//Allocate goal position value into byte array for rolll
					param_goal_position_rolll[0] = DXL_LOBYTE(DXL_LOWORD(dxl_goal_position[2]));
					param_goal_position_rolll[1] = DXL_HIBYTE(DXL_LOWORD(dxl_goal_position[2]));
					param_goal_position_rolll[2] = DXL_LOBYTE(DXL_HIWORD(dxl_goal_position[2]));
					param_goal_position_rolll[3] = DXL_HIBYTE(DXL_HIWORD(dxl_goal_position[2]));


					//Allocate goal position value into byte array for yaw
					param_goal_position_yaw[0] = DXL_LOBYTE(DXL_LOWORD(dxl_goal_position[3]));
					param_goal_position_yaw[1] = DXL_HIBYTE(DXL_LOWORD(dxl_goal_position[3]));
					param_goal_position_yaw[2] = DXL_LOBYTE(DXL_HIWORD(dxl_goal_position[3]));
					param_goal_position_yaw[3] = DXL_HIBYTE(DXL_HIWORD(dxl_goal_position[3]));

					//Allocate goal position value into byte array for mouth
					param_goal_position_mouth[0] = DXL_LOBYTE(DXL_LOWORD(dxl_goal_position[4]));
					param_goal_position_mouth[1] = DXL_HIBYTE(DXL_LOWORD(dxl_goal_position[4]));
					param_goal_position_mouth[2] = DXL_LOBYTE(DXL_HIWORD(dxl_goal_position[4]));
					param_goal_position_mouth[3] = DXL_HIBYTE(DXL_HIWORD(dxl_goal_position[4]));

					// Add Dynamixel#1 goal position value to the Syncwrite parameter storage
					dxl_addparam_result = groupSyncWrite.addParam(DXL1_ID, param_goal_position_pitch); //DXL_1 = pitch
					if (dxl_addparam_result != true)
					{
						fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL1_ID);
						return 0;
					}
					// Add Dynamixel#2 goal position value to the Syncwrite parameter storage
					dxl_addparam_result = groupSyncWrite.addParam(DXL2_ID, param_goal_position_rollr); //DXL_2 = roll_right
					if (dxl_addparam_result != true)
					{
						fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL2_ID);
						return 0;
					}
					// Add Dynamixel#3 goal position value to the Syncwrite parameter storage
					dxl_addparam_result = groupSyncWrite.addParam(DXL3_ID, param_goal_position_rolll); //DXL_3 = roll_left
					if (dxl_addparam_result != true)
					{
						fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL3_ID);
						return 0;
					}
					// Add Dynamixel#4 goal position value to the Syncwrite parameter storage
					dxl_addparam_result = groupSyncWrite.addParam(DXL4_ID, param_goal_position_yaw); //DXL_4 = yaw
					if (dxl_addparam_result != true)
					{
						fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL4_ID);
						return 0;
					}
					// Add Dynamixel#5 goal position value to the Syncwrite parameter storage
					dxl_addparam_result = groupSyncWrite.addParam(DXL5_ID, param_goal_position_mouth); //DXL_5 = mouth
					if (dxl_addparam_result != true)
					{
						fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL5_ID);
						return 0;
					}

					// Syncwrite goal position
					dxl_comm_result = groupSyncWrite.txPacket();
					if (dxl_comm_result != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
					
					if (first_Run == 1) { first_Run = 2; }

					//outData << millisec_since_epoch1 << "," << dxl_goal_position[0] << "," << dxl_present_position[0]
					//	<< "," << dxl_goal_position[1] << "," << dxl_present_position[1]
					//	<< "," << dxl_goal_position[2] << "," << dxl_present_position[2]
					//	<< "," << dxl_goal_position[3] << "," << dxl_present_position[3] << "," << dxl_goal_position[4] << "," << dxl_present_position[4]
					//	<< endl;					
				}

				if (flag_save_motor_present_position)
				{
					dxl_comm_result = groupSyncRead.txRxPacket();
					if (dxl_comm_result != COMM_SUCCESS)
					{
						printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
					}
					for (int i = 0; i < 5; i++)
					{
						if (groupSyncRead.getError(DXL_ID[i], &dxl_error))
						{
							printf("[ID:%03d] %s\n", DXL_ID[i], packetHandler->getRxPacketError(dxl_error));
						}
					}

					// Get Dynamixel present current value
					for (int i = 0; i < 5; i++)
					{
						dxl_present_position[i] = groupSyncRead.getData(DXL_ID[i], ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
					}

					outData << millisec_since_epoch1 << "," << dxl_goal_position[0] << "," << dxl_present_position[0]
						<< "," << dxl_goal_position[1] << "," << dxl_present_position[1]
						<< "," << dxl_goal_position[2] << "," << dxl_present_position[2]
						<< "," << dxl_goal_position[3] << "," << dxl_present_position[3] << "," << dxl_goal_position[4] << "," << dxl_present_position[4]
						<< endl; 
				}

				if (flag_save_motor_present_current)
				{
					dxl_comm_result = groupSyncReadCurrent.txRxPacket();
					if (dxl_comm_result != COMM_SUCCESS)
					{
						printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
					}
					for (int i = 0; i < 5; i++)
					{
						if (groupSyncReadCurrent.getError(DXL_ID[i], &dxl_error))
						{
							printf("[ID:%03d] %s\n", DXL_ID[i], packetHandler->getRxPacketError(dxl_error));
						}
					}


					// Get Dynamixel present current value
					for (int i = 0; i < 5; i++)
					{
						dxl_present_current[i] = groupSyncReadCurrent.getData(DXL_ID[i], ADDR_PRO_PRESENT_CURRENT, LEN_PRO_PRESENT_CURRENT);
					}

					outData_current << millisec_since_epoch1 << "," << dxl_present_current[0] << "," << dxl_present_current[1]
						<< "," << dxl_present_current[2] << "," << dxl_present_current[3] << "," << dxl_present_current[4] << endl;
				}

				// Clear syncwrite parameter storage
				groupSyncWrite.clearParam();

				
				outData2 << millisec_since_epoch1 << "," << roll << "," << roll_f << "," << pitch << "," << pitch_f << ","
					<< yaw << "," << yaw_f << "," << mouth << "," << mouth_f << endl;
				
			

				if (stop == 'q')
				{
					INFO_STREAM("dynamixel_control break");
					break;
				
				}
				//auto awake_time = millisec_since_epoch_2 + 150ms; //loop_time control //revision: 31.6, 40
				//while (std::chrono::high_resolution_clock::now() < awake_time) { std::this_thread::yield(); }

			}
			
		};

		std::thread t1 = std::thread(open_face);
		//std::thread t2 = std::thread(mouth_control);
		std::thread t3 = std::thread(dynamixel_control);
		t1.join();
		//t2.join();
		t3.join();
		INFO_STREAM("Closing output recorder");
		open_face_rec.Close();
		INFO_STREAM("Closing input reader");
		sequence_reader.Close();
		INFO_STREAM("Closed successfully");


		if (recording_params.outputAUs())
		{
			INFO_STREAM("Postprocessing the Action Unit predictions");
			face_analyser.PostprocessOutputFile(open_face_rec.GetCSVFile());
		}

		// Reset the models for the next video
		face_analyser.Reset();
		face_model.Reset();
	}

	groupSyncWriteVelocity.clearParam();
	// Allocate profile velocity value into byte array
	param_profile_velocity[0] = DXL_LOBYTE(DXL_LOWORD(dxl_profile_velocity_homing));
	param_profile_velocity[1] = DXL_HIBYTE(DXL_LOWORD(dxl_profile_velocity_homing));
	param_profile_velocity[2] = DXL_LOBYTE(DXL_HIWORD(dxl_profile_velocity_homing));
	param_profile_velocity[3] = DXL_HIBYTE(DXL_HIWORD(dxl_profile_velocity_homing));

	// Add Dynamixel profile velocity value to the Syncwrite storage
	for (int i = 0; i < 5; i++)
	{
		dxl_addparam_result = groupSyncWriteVelocity.addParam(DXL_ID[i], param_profile_velocity);
		if (dxl_addparam_result != true)
		{
			fprintf(stderr, "[ID:%03d] groupSyncWriteVelocity addparam failed", DXL_ID[i]);
			return 0;
		}
	}

	// Syncwrite profile velocity
	dxl_comm_result = groupSyncWriteVelocity.txPacket();


	//Allocate goal position value into byte array for pitch
	param_zero_position_pitch[0] = DXL_LOBYTE(DXL_LOWORD(default_PITCH + 100));
	param_zero_position_pitch[1] = DXL_HIBYTE(DXL_LOWORD(default_PITCH + 100));
	param_zero_position_pitch[2] = DXL_LOBYTE(DXL_HIWORD(default_PITCH + 100));
	param_zero_position_pitch[3] = DXL_HIBYTE(DXL_HIWORD(default_PITCH + 100));

	//Allocate goal position value into byte array for rollr
	param_zero_position_rollr[0] = DXL_LOBYTE(DXL_LOWORD(default_ROLLR + 100));
	param_zero_position_rollr[1] = DXL_HIBYTE(DXL_LOWORD(default_ROLLR + 100));
	param_zero_position_rollr[2] = DXL_LOBYTE(DXL_HIWORD(default_ROLLR + 100));
	param_zero_position_rollr[3] = DXL_HIBYTE(DXL_HIWORD(default_ROLLR + 100));

	//Allocate goal position value into byte array for rolll
	param_zero_position_rolll[0] = DXL_LOBYTE(DXL_LOWORD(default_ROLLL + 100));
	param_zero_position_rolll[1] = DXL_HIBYTE(DXL_LOWORD(default_ROLLL + 100));
	param_zero_position_rolll[2] = DXL_LOBYTE(DXL_HIWORD(default_ROLLL + 100));
	param_zero_position_rolll[3] = DXL_HIBYTE(DXL_HIWORD(default_ROLLL + 100));


	//Allocate goal position value into byte array for yaw
	param_zero_position_yaw[0] = DXL_LOBYTE(DXL_LOWORD(default_YAW));
	param_zero_position_yaw[1] = DXL_HIBYTE(DXL_LOWORD(default_YAW));
	param_zero_position_yaw[2] = DXL_LOBYTE(DXL_HIWORD(default_YAW));
	param_zero_position_yaw[3] = DXL_HIBYTE(DXL_HIWORD(default_YAW));

	//Allocate goal position value into byte array for mouth
	param_zero_position_mouth[0] = DXL_LOBYTE(DXL_LOWORD(default_MOUTH-100));
	param_zero_position_mouth[1] = DXL_HIBYTE(DXL_LOWORD(default_MOUTH-100));
	param_zero_position_mouth[2] = DXL_LOBYTE(DXL_HIWORD(default_MOUTH-100));
	param_zero_position_mouth[3] = DXL_HIBYTE(DXL_HIWORD(default_MOUTH-100));

	// Add Dynamixel#1 goal position value to the Syncwrite parameter storage
	dxl_addparam_result = groupSyncWrite.addParam(DXL1_ID, param_zero_position_pitch); //DXL_1 = pitch
	if (dxl_addparam_result != true)
	{
		fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL1_ID);
		return 0;
	}
	// Add Dynamixel#2 goal position value to the Syncwrite parameter storage
	dxl_addparam_result = groupSyncWrite.addParam(DXL2_ID, param_zero_position_rollr); //DXL_2 = roll_right
	if (dxl_addparam_result != true)
	{
		fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL2_ID);
		return 0;
	}
	// Add Dynamixel#3 goal position value to the Syncwrite parameter storage
	dxl_addparam_result = groupSyncWrite.addParam(DXL3_ID, param_zero_position_rolll); //DXL_3 = roll_left
	if (dxl_addparam_result != true)
	{
		fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL3_ID);
		return 0;
	}
	// Add Dynamixel#4 goal position value to the Syncwrite parameter storage
	dxl_addparam_result = groupSyncWrite.addParam(DXL4_ID, param_zero_position_yaw); //DXL_4 = yaw
	if (dxl_addparam_result != true)
	{
		fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL4_ID);
		return 0;
	}
	// Add Dynamixel#5 goal position value to the Syncwrite parameter storage
	dxl_addparam_result = groupSyncWrite.addParam(DXL5_ID, param_zero_position_mouth); //DXL_5 = mouth
	if (dxl_addparam_result != true)
	{
		fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL5_ID);
		return 0;
	}

	// Syncwrite goal position
	dxl_comm_result = groupSyncWrite.txPacket();


	// Clear syncwrite parameter storage
	groupSyncWriteVelocity.clearParam();
	groupSyncWrite.clearParam();

	// Allocate profile velocity value into byte array
	param_profile_velocity[0] = DXL_LOBYTE(DXL_LOWORD(dxl_profile_velocity));
	param_profile_velocity[1] = DXL_HIBYTE(DXL_LOWORD(dxl_profile_velocity));
	param_profile_velocity[2] = DXL_LOBYTE(DXL_HIWORD(dxl_profile_velocity));
	param_profile_velocity[3] = DXL_HIBYTE(DXL_HIWORD(dxl_profile_velocity));

	// Add Dynamixel profile velocity value to the Syncwrite storage
	for (int i = 0; i < 5; i++)
	{
		dxl_addparam_result = groupSyncWriteVelocity.addParam(DXL_ID[i], param_profile_velocity);
		if (dxl_addparam_result != true)
		{
			fprintf(stderr, "[ID:%03d] groupSyncWriteVelocity addparam failed", DXL_ID[i]);
			return 0;
		}
	}


	// Syncwrite goal position
	dxl_comm_result = groupSyncWriteVelocity.txPacket();

	sleep_for(4s);

	// Disable Dynamixel Torque
	for (int i = 0; i < 5; i++)
	{
		dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID[i], ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
		if (dxl_comm_result != COMM_SUCCESS)
		{
			printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
		}
		else if (dxl_error != 0)
		{
			printf("%s\n", packetHandler->getRxPacketError(dxl_error));
		} //here
	}
	
	
	// Close port
	portHandler->closePort();
	return 0;

}
	
