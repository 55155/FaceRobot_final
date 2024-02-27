// get gaze direction
cv::Point3f e1 = gazeDirection0;
cv::Point3f e2 = gazeDirection1;
cv::Point3f ec = (e1 + e2) / 2;
ec = ec / norm(ec);


// Gaze point, Glansing point 
std::tie(Gaze_point_cam, Gaze_point_robot) = Gaze(eyescenter);
std::tie(Glansing_point_cam, Glansing_point_robot) = EyeGaze(eyescenter, ec, gazeAngle, DISTANCE2PLANE);

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

	// 1050mm 를 기점으로 좌표값이 이상하여, 보정
	double corr = 0.4 * (pH(0) - 1050) + pH(0);
	for (int i = 0; i < 3; i++)
		pH(i) *= corr / pH(0);

	return pH;
}


// Gaze 시 cam좌표값과 robot좌표값 반환
std::tuple< cv::Point3f, VectorXd > Gaze(cv::Mat1f eyescenter) {
	VectorXd gaze_point_robot;
	cv::Point3f gaze_point_cam;

	gaze_point_robot = cam_to_robot(eyescenter(0, 27), eyescenter(1, 27), eyescenter(2, 27));

	gaze_point_cam.x = eyescenter(0, 27);
	gaze_point_cam.y = eyescenter(1, 27);
	gaze_point_cam.z = eyescenter(2, 27);

	return std::make_tuple(gaze_point_cam, gaze_point_robot);
}

std::tuple<cv::Point3f, VectorXd> EyeGaze(cv::Mat1f eyescenter, cv::Point3f ec, cv::Vec2d gazeAngle, double distance = DISTANCE2PLANE) {
	double _gamma = 0;
	cv::Point3f P_c; // cam 시점에서의 eyecenter

	cv::Point3f Eyegaze_point_cam;
	VectorXd Eyegaze_point_robot;

	// 클래스 맞춰주기
	P_c.x = eyescenter(0, 27);
	P_c.y = eyescenter(1, 27);
	P_c.z = eyescenter(2, 27);

	// 로봇 눈위치에서 계산하기 위해 EYE LEYER의 높이를 SUM
	// 로봇 눈 위치 중심으로 상하좌우 plane을 설치, cam 좌표계로 바꿔서 전달.
	double upperplane = robot_to_cam(0.0, 0.0, EYE_LAYER_HEIGHT + distance)(1);
	double lowerplane = robot_to_cam(0.0, 0.0, EYE_LAYER_HEIGHT - distance)(1);
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
	Eyegaze_point_cam = P_c + _gamma * ec;

	// y축 limit을 넘어가게 되면, 
	if (lowerplane < Eyegaze_point_cam.y || Eyegaze_point_cam.y < upperplane)
		// Gaze 선택
		return Gaze(eyescenter);
	Eyegaze_point_robot = cam_to_robot(Eyegaze_point_cam.x, Eyegaze_point_cam.y, Eyegaze_point_cam.z);

	return std::make_tuple(Eyegaze_point_cam, Eyegaze_point_robot);
}