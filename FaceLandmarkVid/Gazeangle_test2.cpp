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
//       Tadas Baltru쉆itis, Amir Zadeh, Yao Chong Lim, and Louis-Philippe Morency
//       in IEEE International Conference on Automatic Face and Gesture Recognition, 2018  
//
//       Convolutional experts constrained local model for facial landmark detection.
//       A. Zadeh, T. Baltru쉆itis, and Louis-Philippe Morency,
//       in Computer Vision and Pattern Recognition Workshops, 2017.    
//
//       Rendering of Eyes for Eye-Shape Registration and Gaze Estimation
//       Erroll Wood, Tadas Baltru쉆itis, Xucong Zhang, Yusuke Sugano, Peter Robinson, and Andreas Bulling 
//       in IEEE International. Conference on Computer Vision (ICCV),  2015 
//
//       Cross-dataset learning and person-specific normalisation for automatic Action Unit detection
//       Tadas Baltru쉆itis, Marwa Mahmoud, and Peter Robinson 
//       in Facial Expression Recognition and Analysis Challenge, 
//       IEEE International Conference on Automatic Face and Gesture Recognition, 2015 
//
///////////////////////////////////////////////////////////////////////////////
// FaceTrackingVid.cpp : Defines the entry point for the console application for tracking faces in videos.

// Libraries for landmark detection (includes CLNF and CLM modules)
#include "LandmarkCoreIncludes.h"
#include "GazeEstimation.h"

#include <SequenceCapture.h>
#include <Visualizer.h>
#include <VisualizationUtils.h>
#include <cmath>
#include <vector>
#include <dense>
#include "matplotlibcpp.h"
#include <vector>
#include <iostream>
#include <windows.h>



#define Poc_x 200
#define Poc_y -150	-20
#define Poc_z 20	-100
#define EYE_LAYER_HEIGHT 200
#define INFO_STREAM( stream ) \
std::cout << stream << std::endl

#define WARN_STREAM( stream ) \
std::cout << "Warning: " << stream << std::endl

#define ERROR_STREAM( stream ) \
std::cout << "Error: " << stream << std::endl



static void printErrorAndAbort(const std::string& error)
{
	std::cout << error << std::endl;
	abort();
}


#define FATAL_STREAM( stream ) \
printErrorAndAbort( std::string( "Fatal error: " ) + stream )

std::vector<std::string> get_arguments(int argc, char** argv)
{

	std::vector<std::string> arguments;

	for (int i = 0; i < argc; ++i)
	{
		arguments.push_back(std::string(argv[i]));
	}
	return arguments;
}

using namespace std;
using namespace Eigen;
namespace plt = matplotlibcpp;

VectorXd robot_to_cam(double x, double y, double z) {
	// define P_cs (cam to robot)
	VectorXd P_cs(3);
	P_cs(0) = double(Poc_y);
	P_cs(1) = double(Poc_z);
	P_cs(2) = -double(Poc_x);

	// define P_s (robot frame)
	VectorXd P_s(3);
	P_s(0) = x;
	P_s(1) = y;
	P_s(2) = z;

	// rotation (x R_cs)
	VectorXd rotate_P_s(3);
	rotate_P_s(0) = -P_s(1);
	rotate_P_s(1) = -P_s(2);
	rotate_P_s(2) = P_s(0);

	// define P_c (cam frame)
	VectorXd P_c(3);
	P_c = P_cs + rotate_P_s;

	return P_c;
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

	LandmarkDetector::FaceModelParameters det_parameters(arguments);

	// The modules that are being used for tracking
	LandmarkDetector::CLNF face_model(det_parameters.model_location);
	if (!face_model.loaded_successfully)
	{
		std::cout << "ERROR: Could not load the landmark detector" << std::endl;
		return 1;
	}

	if (!face_model.eye_model)
	{
		std::cout << "WARNING: no eye model found" << std::endl;
	}

	// Open a sequence
	Utilities::SequenceCapture sequence_reader;

	// A utility for visualizing the results (show just the tracks)
	Utilities::Visualizer visualizer(true, false, false, false);

	// Tracking FPS for visualization
	Utilities::FpsTracker fps_tracker;
	fps_tracker.AddFrame();
	int sequence_number = 0;

	std::vector<double> x(2);
	std::vector<double> y(2);
	std::vector<double> z(2);

	plt::ion();
	double iter = 0;

	while (true) // this is not a for loop as we might also be reading from a webcam
	{
		iter += 1;
		// The sequence reader chooses what to open based on command line arguments provided
		if (!sequence_reader.Open(arguments))
			break;

		INFO_STREAM("Device or file opened");

		cv::Mat rgb_image = sequence_reader.GetNextFrame();

		INFO_STREAM("Starting tracking");

		int max_limit = -1000;
		int min_limit = 1000;
		std::map<std::string, std::string> graphmap;
		graphmap["color"] = "lightgray";
		graphmap["linestyle"] = "--";
		graphmap["linewidth"] = "2";

		plt::xlim(max_limit, min_limit);
		plt::ylim(max_limit, min_limit);

		plt::axhline(0, 0, 1, graphmap);
		plt::axvline(0, 0, 1, graphmap);

		while (!rgb_image.empty()) // this is not a for loop as we might also be reading from a webcam
		{

			// Reading the images
			cv::Mat_<uchar> grayscale_image = sequence_reader.GetGrayFrame();

			// The actual facial landmark detection / tracking
			bool detection_success = LandmarkDetector::DetectLandmarksInVideo(rgb_image, face_model, det_parameters, grayscale_image);

			// Gaze tracking, absolute gaze direction
			cv::Point3f gazeDirection0(0, 0, -1);
			cv::Point3f gazeDirection1(0, 0, -1);
			cv::Vec2d gazeAngle(0, 0);

			// If tracking succeeded and we have an eye model, estimate gaze
			if (detection_success && face_model.eye_model)
			{
				GazeAnalysis::EstimateGaze(face_model, gazeDirection0, sequence_reader.fx, sequence_reader.fy, sequence_reader.cx, sequence_reader.cy, true);
				GazeAnalysis::EstimateGaze(face_model, gazeDirection1, sequence_reader.fx, sequence_reader.fy, sequence_reader.cx, sequence_reader.cy, false);
				gazeAngle = GazeAnalysis::GetGazeAngle(gazeDirection0, gazeDirection1);
			}

			// 250mm -> 로봇 중심에서 25cm정도 떨어진 곳을 바라볼 때는 gaze


			// rows: 3, cols: 68
			cv::Mat1f eyescenter = face_model.GetShape(sequence_reader.fx, sequence_reader.fy, sequence_reader.cx, sequence_reader.cy);

			cv::Point3f P_c; // cam 중심 좌표
			cv::Point3f P_s; // 로봇 중심 좌표
			cv::Point3f P_sc = { Poc_x, Poc_y, Poc_z };

			// cam 중심
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

			// calibration
			// cout << P_s << endl;
			// left eye
			cv::Mat lefteyeLdmks3d = face_model.hierarchical_models[0].GetShape(sequence_reader.fx, sequence_reader.fy, sequence_reader.cx, sequence_reader.cy);

			// right eye
			cv::Mat righteyeLdmks3d = face_model.hierarchical_models[1].GetShape(sequence_reader.fx, sequence_reader.fy, sequence_reader.cx, sequence_reader.cy);

			// get pupil position
			cv::Point3f P_e1 = GazeAnalysis::GetPupilPosition(lefteyeLdmks3d);
			cv::Point3f P_e2 = GazeAnalysis::GetPupilPosition(righteyeLdmks3d);

			// gaze point 
			cv::Point3f gaze_point_cam;
			cv::Point3f gaze_point_robot;

			// get gaze direction
			cv::Point3f e1 = gazeDirection0;
			cv::Point3f e2 = gazeDirection1;
			cv::Point3f ec = (e1 + e2) / 2;

			// define the plane
			double R = 300; // 적절한 위치에 생성해야함.
			// 눈 위치를 중심으로 원을 그리는 가정
			VectorXd O = robot_to_cam(0.0, 0.0, EYE_LAYER_HEIGHT);
			VectorXd gamma(2);

			double c1 = P_c.x - O(0);
			double c2 = P_c.y - O(1);
			
			// constants
			double a = pow(ec.x, 2) + pow(ec.y, 2);
			// zero division 고려 [0, 0, -1]
			double b = 2 * (ec.x * c1 + ec.y * c2);
			double c = pow(c1, 2) + pow(c2, 2) - pow(R, 2);

			// solution
			if (a != 0) {
				gamma(0) = (-b + sqrt(pow(b, 2) - 4 * a * c)) / (2 * a);
				gamma(1) = (-b - sqrt(pow(b, 2) - 4 * a * c)) / (2 * a);
				
				// solution1
				x[0] = P_c.x + gamma(0) * ec.x;
				y[0] = P_c.y + gamma(0) * ec.y;
				z[0] = P_c.z + gamma(0) * ec.z;

				// solution2
				x[1] = P_c.x + gamma(1) * ec.x;
				y[1] = P_c.y + gamma(1) * ec.y;
				z[1] = P_c.z + gamma(1) * ec.z;
				
				// z 값이 더 작은 gamma 값을 추종해야함
				gaze_point_cam = (z[0] < z[1]) ? P_c + gamma(0) * ec : P_c + gamma(1) * ec;
			}
			else { // zerodivision
				cout << "undefiend gaze angle" << endl;
				gaze_point_cam = P_c;
			}

			if (0 < gaze_point_cam.z && gaze_point_cam.z < P_c.z) { // gaze 해야하는 경우  rotation matrix

				gaze_point_robot.x = gaze_point_cam.z;
				gaze_point_robot.y = -gaze_point_cam.x;
				gaze_point_robot.z = -gaze_point_cam.y;

			}
			else { // eye gaze 해야하는 경우 rotation matrix
				gaze_point_robot.x = P_c.z;
				gaze_point_robot.y = -P_c.x;
				gaze_point_robot.z = -P_c.y;

			}

			gaze_point_robot.x += Poc_x;
			gaze_point_robot.y += Poc_y;
			gaze_point_robot.z += Poc_z;

			cout << sqrt(pow((O(0) - gaze_point_cam.x), 2) + pow((O(1) - gaze_point_cam.y), 2)) << endl;

			x[0] = gaze_point_robot.x;
			y[0] = gaze_point_robot.y;
			z[0] = gaze_point_robot.z;

			plt::scatter(y, z, 3.0);
			plt::pause(0.1);

			// Work out the pose of the head from the tracked model
			cv::Vec6d pose_estimate = LandmarkDetector::GetPose(face_model, sequence_reader.fx, sequence_reader.fy, sequence_reader.cx, sequence_reader.cy);

			// Keeping track of FPS
			fps_tracker.AddFrame();

			// Displaying the tracking visualizations
			visualizer.SetImage(rgb_image, sequence_reader.fx, sequence_reader.fy, sequence_reader.cx, sequence_reader.cy);
			// visualizer.SetObservationLandmarks(face_model.detected_landmarks, face_model.detection_certainty, face_model.GetVisibilities());
			// visualizer.SetObservationPose(pose_estimate, face_model.detection_certainty);
			visualizer.SetObservationGaze(10 * ec, 10 * ec, LandmarkDetector::CalculateAllEyeLandmarks(face_model), LandmarkDetector::Calculate3DEyeLandmarks(face_model, sequence_reader.fx, sequence_reader.fy, sequence_reader.cx, sequence_reader.cy), face_model.detection_certainty);
			visualizer.SetFps(fps_tracker.GetFPS());
			// detect key presses (due to pecularities of OpenCV, you can get it when displaying images)
			char character_press = visualizer.ShowObservation();

			// 1초씩 카메라 쉼

		   // restart the tracker
			if (character_press == 'r')
			{
				face_model.Reset();
			}
			// quit the application
			else if (character_press == 'q')
			{
				return(0);
			}

			// Grabbing the next frame in the sequence
			rgb_image = sequence_reader.GetNextFrame();

		}

		// Reset the model, for the next video
		face_model.Reset();
		sequence_reader.Close();
		// plt kill
		plt::detail::_interpreter::kill();
		sequence_number++;

	}
	return 0;
}

