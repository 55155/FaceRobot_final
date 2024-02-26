#pragma once
#include "MacrosAndFunctions_v0.h"
#include <conio.h>
#include <deque>
#include <tuple>

// queue 에서 평균과 분산을 지원하는 메서드 없음. 
#include <numeric>
#include <math.h>



// 대충 자로 잰 값 + 실제 값 보면서 calibration 
#define Poc_x 200   + 200
#define Poc_y -150	+20
#define Poc_z 20	
#define EYE_LAYER_HEIGHT 200

// 가상의 공간 설정
#define DISTANCE2PLANE	450.0	// 로봇과 시선의 종점이 만나는 plane까지의 거리
#define BACKWARD_LIMIT	-1000	// 로봇의 뒷방향 한계


using namespace std;

// 이동평균선에서의 표준편차를 구해야함. 
// 데이터셋은 2s를 기준으로 표준편차를 구해서 일정수준 이하로 판정될 때, gaze 하는 것으로 설정

// 칼만필터는 어렵지 않아 22p
// 이동평균필터의 재귀분산 계산
double Moving_Avg_Filter(double X_k, double pre_avg, double X_kn, double N = 25) {
	double avg;
	avg = pre_avg + (X_k - X_kn) / N;
	return avg;
}
// 분산계산시에 들어가는 sum^2을 재귀적으로 구한 값
double recursive_sum2(double X_k, double pre_sum2, double X_kn) {
	double sum2 = pre_sum2 + (X_k * X_k - X_kn * X_kn);
	return sum2;
}

// 직접 계산한 점화식
double recursive_Variance(double X_k, double pre_avg, double pre_val, double X_kn, double N) {
	double avg = Moving_Avg_Filter(X_k, pre_avg, X_kn);
	double var = pre_val + (pow(X_k, 2) - pow(X_kn, 2) - pow(avg, 2) * N + pow(pre_avg, 2) * N) / (N);
	return var;
}

// 초기 평균 계산
double deque_mean(std::deque<double> q) {
	double initial_value = 0.0;
	// std::accumulate 는 intial 값의 자료형을 따라감. 
	double sum = std::accumulate(q.begin(), q.end(), initial_value);
	double mean = sum / q.size();

	return mean;
}

// 초기 sum^2 계산
double deque_sum2(std::deque<double> q) {
	double initial_value = 0.0;
	while (q.size()) {
		initial_value += q.back() * q.back();
		q.pop_back();
	}

	return initial_value;
}

// sum^2에 의해 재귀적으로 구해진 분산 계산
double variance(double sum2, double avg, double N) {
	double var = (sum2 / N) - avg * avg;
	return var;
}

// 초기 분산 계산
double deque_variance(std::deque<double> q) {
	double mean = deque_mean(q);
	double sumSquaredDifferences = 0.0;
	for (const auto& value : q) {
		double difference = value - mean;
		sumSquaredDifferences += difference * difference;
	}
	double variance = sumSquaredDifferences / (q.size());
	return variance;
}

// cam 좌표에서 robot 좌표계로 변환
VectorXd cam_to_robot(double x, double y, double z) {
	// 실제 값 * 보정값까지 고려
	VectorXd pH(3);

	VectorXd P_sc(3);
	P_sc(0) = double(Poc_x);
	P_sc(1) = double(Poc_y);
	P_sc(2) = double(Poc_z);

	// define P_c(cam frame)
	VectorXd P_c(3);
	P_c(0) = x;
	P_c(1) = y;
	P_c(2) = z;

	// rotation matrix ( x R_sc)
	VectorXd rotate_P_c(3);
	rotate_P_c(0) = P_c(2);
	rotate_P_c(1) = -P_c(0);
	rotate_P_c(2) = -P_c(1);

	VectorXd P_s(3);
	P_s = rotate_P_c;

	pH(0) = Poc_x + P_s(0);
	pH(1) = (Poc_y + P_s(1));		// 카메라 or OpenFace가 조금 부정확한 거 같아서 임의로 튜닝

	if (Poc_z + P_s(2) > 0)
		pH(2) = (Poc_z + P_s(2));	// 카메라 or OpenFace가 조금 부정확한 거 같아서 임의로 튜닝
	else
		pH(2) = (Poc_z + P_s(2));	// 카메라 or OpenFace가 조금 부정확한 거 같아서 임의로 튜닝

	return pH;
}

double radian_to_degree(double radian) {
	double degree = (180.0 / PI) * radian;
	return degree;
}

// Gaze 시 cam좌표값과 robot좌표값 반환
std::tuple< cv::Point3f, VectorXd > Gaze(cv::Mat1f eyescenter) {
	VectorXd gaze_point_robot;
	cv::Point3f gaze_point_cam;

	gaze_point_robot = cam_to_robot(eyescenter(0, 27), eyescenter(1, 27), eyescenter(2, 27));
	double corr = 0.4 * (gaze_point_robot(0) - 1050) + gaze_point_robot(0);
	for(int i = 0; i < 3; i++)
		gaze_point_robot(i) *= corr / gaze_point_robot(0);
	gaze_point_cam.x = eyescenter(0);
	gaze_point_cam.y = eyescenter(1);
	gaze_point_cam.z = eyescenter(2);

	return std::make_tuple(gaze_point_cam, gaze_point_robot );
}

std::tuple<cv::Point3f, VectorXd> EyeGaze(cv::Mat1f eyescenter, cv::Point3f ec, cv::Vec2d gazeAngle, double distance = DISTANCE2PLANE) {
	double _gamma = 0;
	cv::Point3f P_c; // cam 시점에서의 eyecenter

	cv::Point3f gaze_point_cam;
	VectorXd gaze_point_robot;

	// 클래스 맞춰주기
	P_c.x = eyescenter(0, 27);
	P_c.y = eyescenter(1, 27);
	P_c.z = eyescenter(2, 27);

	// 로봇 눈위치에서 계산하기 위해 EYE LEYER의 높이를 SUM
	// 로봇 눈 위치 중심으로 상하좌우 plane을 설치, cam 좌표계로 바꿔서 전달.
	double upperplane = robot_to_cam(0.0, 0.0, EYE_LAYER_HEIGHT + distance + 300)(1);
	double lowerplane = robot_to_cam(0.0, 0.0, EYE_LAYER_HEIGHT - distance - 300)(1);
	double rightplane = robot_to_cam(0.0, distance, 0.0)(0);
	double leftplane = robot_to_cam(0.0, -distance, 0.0)(0);

	// 전달된 좌표값을 통해 교점을 구한다. 
	if (gazeAngle(0) < 0) {
		_gamma = (-P_c.x + rightplane) / ec.x;
	}
	else if (gazeAngle(0) > 0) {
		_gamma = (-P_c.x + leftplane) / ec.x;
	}
	else {
		cout << "gamma is undefined" << endl;
	}
	gaze_point_cam = P_c + _gamma * ec;

	// y축 limit을 넘어가게 되면, 
	if (lowerplane < gaze_point_cam.y || gaze_point_cam.y < upperplane)
		// Gaze 선택
		return Gaze(eyescenter);
	gaze_point_robot = cam_to_robot(gaze_point_cam.x, gaze_point_cam.y, gaze_point_cam.z);

	return std::make_tuple(gaze_point_cam, gaze_point_robot);
}

/*
VectorXd EyeGaze(VectorXd Eyegaze_point_robot, double* L) {
	double nL = *L + 0.04;
	if (nL > 1) {
		*L = 1;
	}
	else
		*L += 0.04;
	return Eyegaze_point_robot;
}

VectorXd Gaze(VectorXd Gaze_point_robot, double* L) {
	double nL = *L - 0.04;
	if (nL < 0) {
		*L = 0;
	}
	else
		*L -= 0.04;
	return Gaze_point_robot;
}
*/

VectorXd Lag_Interp(VectorXd Eyegaze_point_robot, VectorXd Gaze_point_robot, double L) {
	VectorXd output = L * Eyegaze_point_robot + (1 - L) * Gaze_point_robot;
	// cout << "interp : " << output << L << endl;
	return output;
}

// 라그랑쥬 보간에 곱해지는 L값
double x(double t, double T, double Glansing_time) { // t = arrange(0, 1000, 40)
	double L; // L

	// Gaze point -> Glansing point
	if (t <= (T / 2.0)) {
		L = (1.0 / 2.0) * (1.0 - cos((PI / (T / 2.0)) * t));
	}
	// Glansing point에 도달했을 때에는 1로 고정
	else if ((T / 2.0) < t && t < Glansing_time + (T / 2.0)) { // Glansing
		L = 1.0;
	}
	// Glansing point -> Gaze point
	else if (Glansing_time + (T / 2.0) <= t) {
		L = (1.0 / 2.0) * (1.0 + cos((PI / (T / 2.0)) * (t - (T / 2.0 + Glansing_time))));
	}
	return L;
}

bool Stop_Glansing(int GlansingTime, int pre_GlansingTime) {
	bool Stop = abs(GlansingTime - pre_GlansingTime) > (60000 / FREQUENCY.count()) ? false : true;
	return Stop;
}
VectorXd virtual_point(VectorXd Eyegaze_point_robot, VectorXd Gaze_point_robot) {
	VectorXd Vp(3);
	Vp(0) = (Eyegaze_point_robot(0) + Gaze_point_robot(0)) / 2.0;
	Vp(1) = (Eyegaze_point_robot(1) + Gaze_point_robot(1)) / 2.0;
	Vp(2) = Gaze_point_robot(2) + (1.0 / 6.0) * (Eyegaze_point_robot(2) - Gaze_point_robot(2));

	return Vp;
}

// 
double Period(VectorXd Fixed_Eyegaze_point_robot, VectorXd Gaze_point_robot) { // d = plane 까지의 거리
	double goal_point_yaw = findGaze_RPY_Gamma(Fixed_Eyegaze_point_robot)(2);
	double start_point_yaw = findGaze_RPY_Gamma(Gaze_point_robot)(2);

	double max_yaw = MAX_YAW;
	double min_yaw = PI / 9.0;

	// min - max normalization
	// 최대 10s, 최소 2.0s
	double t = 8.0 * (abs(goal_point_yaw - start_point_yaw) - min_yaw) / (max_yaw - min_yaw) + 2.0;
	t *= 1000;
	return t;
}
/*
VectorXd Lag_interp2(VectorXd Eyegaze_point_robot, VectorXd Gaze_point_robot, VectorXd Vp, double X) {
	temp = ((slice_x[i] - virtual_point_x) * (slice_x[i] - x[1])) / ((x[0] - virtual_point_x) * (x[0] - x[1])) * y[0] \
		+ ((slice_x[i] - x[0]) * (slice_x[i] - x[1])) / ((virtual_point_x - x[0]) * (virtual_point_x - x[1])) * virtual_point_y \
		+ ((slice_x[i] - x[0]) * (slice_x[i] - virtual_point_x)) / ((x[1] - x[0]) * (x[1] - virtual_point_x)) * y[1]
}
*/

// cam 주기와 실제 주기가 맞지 않을 때, 외삽으로 t+1 에서의 rpy를 계산한다. 
VectorXd Linear_Extrap(double) {
	// x = t
	// Linear_Extrap = x, y, z
}


