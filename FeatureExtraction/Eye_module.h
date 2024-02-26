#pragma once
#include "MacrosAndFunctions_v0.h"
#include <conio.h>
#include <deque>
#include <tuple>

// queue ���� ��հ� �л��� �����ϴ� �޼��� ����. 
#include <numeric>
#include <math.h>



// ���� �ڷ� �� �� + ���� �� ���鼭 calibration 
#define Poc_x 200   + 200
#define Poc_y -150	+20
#define Poc_z 20	
#define EYE_LAYER_HEIGHT 200

// ������ ���� ����
#define DISTANCE2PLANE	450.0	// �κ��� �ü��� ������ ������ plane������ �Ÿ�
#define BACKWARD_LIMIT	-1000	// �κ��� �޹��� �Ѱ�


using namespace std;

// �̵���ռ������� ǥ�������� ���ؾ���. 
// �����ͼ��� 2s�� �������� ǥ�������� ���ؼ� �������� ���Ϸ� ������ ��, gaze �ϴ� ������ ����

// Į�����ʹ� ����� �ʾ� 22p
// �̵���������� ��ͺл� ���
double Moving_Avg_Filter(double X_k, double pre_avg, double X_kn, double N = 25) {
	double avg;
	avg = pre_avg + (X_k - X_kn) / N;
	return avg;
}
// �л���ÿ� ���� sum^2�� ��������� ���� ��
double recursive_sum2(double X_k, double pre_sum2, double X_kn) {
	double sum2 = pre_sum2 + (X_k * X_k - X_kn * X_kn);
	return sum2;
}

// ���� ����� ��ȭ��
double recursive_Variance(double X_k, double pre_avg, double pre_val, double X_kn, double N) {
	double avg = Moving_Avg_Filter(X_k, pre_avg, X_kn);
	double var = pre_val + (pow(X_k, 2) - pow(X_kn, 2) - pow(avg, 2) * N + pow(pre_avg, 2) * N) / (N);
	return var;
}

// �ʱ� ��� ���
double deque_mean(std::deque<double> q) {
	double initial_value = 0.0;
	// std::accumulate �� intial ���� �ڷ����� ����. 
	double sum = std::accumulate(q.begin(), q.end(), initial_value);
	double mean = sum / q.size();

	return mean;
}

// �ʱ� sum^2 ���
double deque_sum2(std::deque<double> q) {
	double initial_value = 0.0;
	while (q.size()) {
		initial_value += q.back() * q.back();
		q.pop_back();
	}

	return initial_value;
}

// sum^2�� ���� ��������� ������ �л� ���
double variance(double sum2, double avg, double N) {
	double var = (sum2 / N) - avg * avg;
	return var;
}

// �ʱ� �л� ���
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

// cam ��ǥ���� robot ��ǥ��� ��ȯ
VectorXd cam_to_robot(double x, double y, double z) {
	// ���� �� * ���������� ���
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
	pH(1) = (Poc_y + P_s(1));		// ī�޶� or OpenFace�� ���� ����Ȯ�� �� ���Ƽ� ���Ƿ� Ʃ��

	if (Poc_z + P_s(2) > 0)
		pH(2) = (Poc_z + P_s(2));	// ī�޶� or OpenFace�� ���� ����Ȯ�� �� ���Ƽ� ���Ƿ� Ʃ��
	else
		pH(2) = (Poc_z + P_s(2));	// ī�޶� or OpenFace�� ���� ����Ȯ�� �� ���Ƽ� ���Ƿ� Ʃ��

	return pH;
}

double radian_to_degree(double radian) {
	double degree = (180.0 / PI) * radian;
	return degree;
}

// Gaze �� cam��ǥ���� robot��ǥ�� ��ȯ
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
	cv::Point3f P_c; // cam ���������� eyecenter

	cv::Point3f gaze_point_cam;
	VectorXd gaze_point_robot;

	// Ŭ���� �����ֱ�
	P_c.x = eyescenter(0, 27);
	P_c.y = eyescenter(1, 27);
	P_c.z = eyescenter(2, 27);

	// �κ� ����ġ���� ����ϱ� ���� EYE LEYER�� ���̸� SUM
	// �κ� �� ��ġ �߽����� �����¿� plane�� ��ġ, cam ��ǥ��� �ٲ㼭 ����.
	double upperplane = robot_to_cam(0.0, 0.0, EYE_LAYER_HEIGHT + distance + 300)(1);
	double lowerplane = robot_to_cam(0.0, 0.0, EYE_LAYER_HEIGHT - distance - 300)(1);
	double rightplane = robot_to_cam(0.0, distance, 0.0)(0);
	double leftplane = robot_to_cam(0.0, -distance, 0.0)(0);

	// ���޵� ��ǥ���� ���� ������ ���Ѵ�. 
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

	// y�� limit�� �Ѿ�� �Ǹ�, 
	if (lowerplane < gaze_point_cam.y || gaze_point_cam.y < upperplane)
		// Gaze ����
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

// ��׶��� ������ �������� L��
double x(double t, double T, double Glansing_time) { // t = arrange(0, 1000, 40)
	double L; // L

	// Gaze point -> Glansing point
	if (t <= (T / 2.0)) {
		L = (1.0 / 2.0) * (1.0 - cos((PI / (T / 2.0)) * t));
	}
	// Glansing point�� �������� ������ 1�� ����
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
double Period(VectorXd Fixed_Eyegaze_point_robot, VectorXd Gaze_point_robot) { // d = plane ������ �Ÿ�
	double goal_point_yaw = findGaze_RPY_Gamma(Fixed_Eyegaze_point_robot)(2);
	double start_point_yaw = findGaze_RPY_Gamma(Gaze_point_robot)(2);

	double max_yaw = MAX_YAW;
	double min_yaw = PI / 9.0;

	// min - max normalization
	// �ִ� 10s, �ּ� 2.0s
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

// cam �ֱ�� ���� �ֱⰡ ���� ���� ��, �ܻ����� t+1 ������ rpy�� ����Ѵ�. 
VectorXd Linear_Extrap(double) {
	// x = t
	// Linear_Extrap = x, y, z
}


