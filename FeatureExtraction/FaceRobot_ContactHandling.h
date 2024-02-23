#include "CNN1D.h"

#include<deque>

class ContactHandler
{
public:
		
	ContactHandler() : _NumMotors(4), _NumClasses(6) { InitializeContactHandler(); }
	ContactHandler(unsigned int NumMotors, unsigned int NumClasses) : _NumMotors(NumMotors), _NumClasses(NumClasses) { InitializeContactHandler(); }
	~ContactHandler() {}

	void resetContactHandler() { _reset = true; _contactClassified = false;	}
	int getContactResult(vector<int> original);
	bool isContactDetectedAndClassified() { return _contactClassified; }

private:

	void InitializeContactHandler();

	vector<int> MeanFilter(vector<int> original_current, unsigned int meanfilter, bool reset);
	bool isDeltaLarge(unsigned int motoridx, unsigned int stepdelta, int mode);
	int getPresentContactLabel(vector<int> filtered, unsigned int resetIgnore, bool reset);	
	vector<float> Normalize(vector<int> filtered, bool reset);
	MatrixXd getWindowFeature(bool do_classification);

	void DetermineClassificationExecution(int presentlabel, bool reset);
	
	int getPresentClass(bool do_classification, MatrixXd input);
	int getContactClass(bool do_classification, int present_class, bool reset);

	void NetworkInitialize();
	VectorXd NetworkForward(MatrixXd input);

	const unsigned int _NumMotors;
	const unsigned int _NumClasses;
	bool _reset;
	bool _contactClassified;
	
	// deque: [0]: 과거, [size-1]: 현재
	vector<deque<int>> _current_original;	// for mean filtering; vector size: NumMotors, deque size: filter size
	vector<deque<int>> _delta_step;			// for contact labeling; vector size: NumMotors
	vector<deque<float>> _current_norm;		// filtered + normalized current -> window feature
	deque<int> _ignore_contact_stack;		// ignore 영역의 contact label들 보관	
	deque<int> _check_contact_stack;		// check 영역의 contact label들 보관	
	bool _doClassifiacion;					// 접촉이라 판단돼서 contact classification을 실행할지 말지 여부

	NETWORK_1DCNN _model;
	LAYER_1DCNN _conv1, _conv2;
	LAYER_LINEAR _linear1;

	const int _meanfiltersize = 5;						// mean filter
	
	const int _deltaplusone = 55;						// contact labeling	
	const int _resetIgnore = 8;							// contact labeling - status 바뀌고 이 step만큼은 접촉 없다고 가정
	////const int _label_limit_delta2[5] = { 5,4,5,1,1 };	// contact labeling (커지면 민감도 작아짐)
	////const int _label_limit_delta3[5] = { 7,7,7,1,2 };	// contact labeling (커지면 민감도 작아짐)
	//const int _label_limit_delta2[5] = { 4,4,3,1,1 };	// contact labeling (커지면 민감도 작아짐)
	//const int _label_limit_delta3[5] = { 4,4,4,1,1 };	// contact labeling (커지면 민감도 작아짐)
	//const int _label_limit_med[5] = { 7,7,7,2,2 };		// contact labeling - 적당히 가해지는 접촉 감지
	//const int _label_limit_slow[5] = { 8,8,8,2,2 };		// contact labeling - 천천히 가해지는 접촉 감지용

	int _label_limit[5] = { 100,100,100,100,100 };
	
	const int _currentMax[5] = { 92,151,154,192,33 };	// normalize
	const int _currentMin[5] = { 0,0,0,-160,0 };		// normalize	
	
	const int _windowsize = 18;							// window feature
	const int _sampling = 2;							// window feature
	
	const int _continuous = 2;							// classification - continuous filter (<= _ignore)
	const int _ignore = 5;								// classification - contact라고 판단(continuous만큼 연속으로 contact)되고 얼마나 무시할지
	const int _check = 10;								// classification - 무시한 후에 몇 스텝까지 클래스 판단에 사용할지
	const int _classminnum = 5;							// classification - 한 클래스가 classminnum 이상 찍히면 해당 접촉은 해당 클래스로
		
	

	int getDequeMean(deque<int> dq);
	template <typename T>
	void resetVectorDeque(vector<deque<T>>& vdq, int vecsize);
	template <typename T>
	void resetVectorDeque(vector<deque<T>>& vdq, int vecsize, int dequesize, T dequeval);
	template <typename T>
	void resetVectorDeque(vector<deque<T>>& vdq, int vecsize, int dequesize, vector<T> dequevals);
	template <typename T>
	void printVectorDeque(vector<deque<T>> vdq);
	template <typename T>
	void resetDeque(deque<T>& dq, int dequesize, T dequeval);
	template <typename T>
	void resetDeque(deque<T>& dq) { dq.clear(); }
};

void ContactHandler::InitializeContactHandler() {
	
	resetVectorDeque(_current_original, _NumMotors);
	//resetVectorDeque(_delta_step, _NumMotors, _deltaplusone, 0);
	resetVectorDeque(_delta_step, _NumMotors);
	resetVectorDeque(_current_norm, _NumMotors);

	resetDeque(_ignore_contact_stack, _ignore + 1, 0);
	resetDeque(_check_contact_stack);
	_doClassifiacion = false;
	_contactClassified = false;
	_reset = false;

	NetworkInitialize();
}

int ContactHandler::getContactResult(vector<int> original_current) {
	
	// 1. contactlabel: 접촉 여부
	// 2. class: 어떤 접촉인지
	// a. present: 현재 시점 (시간 단위)
	// b. contact: 현재 접촉 (접촉 단위)

	if (_reset)
		resetContactHandler();

	// 항상
	vector<int> filtered = MeanFilter(original_current, _meanfiltersize, _reset);
	int present_contactlabel = getPresentContactLabel(filtered, _resetIgnore, _reset);

	Normalize(filtered, _reset);
	DetermineClassificationExecution(present_contactlabel, _reset);

	// 접촉일 때만
	MatrixXd windowfeature = getWindowFeature(_doClassifiacion);
	int present_class = getPresentClass(_doClassifiacion, windowfeature);
	int contact_class = getContactClass(_doClassifiacion, present_class, _reset);

	_reset = false;

	if (contact_class > 0) {
		_reset = true;
		_contactClassified = true;
	}		

	return contact_class;
}

vector<int> ContactHandler::MeanFilter(vector<int> original, unsigned int filtersize, bool reset) {
	
	// original size: _NumMotors
	static int filteriter = 0;
	if (reset) {
		filteriter = 0;
		resetVectorDeque(_current_original, _NumMotors);
	}		

	for (int i = 0; i < _NumMotors; i++)
		_current_original[i].push_back(original[i]);

	if (filteriter >= filtersize) {
		for (int i = 0; i < _NumMotors; i++)
			_current_original[i].pop_front();
	}
	else
		filteriter++;

	vector<int> filtered(_NumMotors);
	for (int i = 0; i < _NumMotors; i++)
		filtered[i] = getDequeMean(_current_original[i]);
	
	return filtered;
}

bool ContactHandler::isDeltaLarge(unsigned int motoridx, unsigned int stepdelta, int mode) {

	bool isDeltaLarge = false;
	int diff = abs(_delta_step[motoridx].end()[-1] - _delta_step[motoridx].end()[-1 - stepdelta]);

	if (mode == 1) {
		_label_limit[0] = 5; 
		_label_limit[1] = 5; 
		_label_limit[2] = 5; 
		_label_limit[3] = 3;
	}
	else if (mode == 2) {
		_label_limit[0] = 6; 
		_label_limit[1] = 6; 
		_label_limit[2] = 6; 
		_label_limit[3] = 3;
	}
	else if (mode == 3) {
		_label_limit[0] = 7; 
		_label_limit[1] = 7; 
		_label_limit[2] = 7;
		_label_limit[3] = 3;
	}
	else if (mode == 4) {
		_label_limit[0] = 8; 
		_label_limit[1] = 8; 
		_label_limit[2] = 8; 
		_label_limit[3] = 3;
	}
	else {
		cout << "isDeltaLarge ERROR" << endl;
	}

	if (diff > _label_limit[motoridx])
		isDeltaLarge = true;

	return isDeltaLarge;
}

int ContactHandler::getPresentContactLabel(vector<int> filtered, unsigned int resetIgnore, bool reset) {

	static int contactiter = 0;
	if (reset) {
		contactiter = 0;		
		resetVectorDeque(_delta_step, _NumMotors);
	}

	if (contactiter == 0) {
		resetVectorDeque(_delta_step, _NumMotors, _deltaplusone, filtered);
	}

	//cout << filtered[0] << "   " << filtered[1] << "   " << filtered[2] << "   " << filtered[3] << endl;
 
	int contact = 0;

	vector<bool> deltaContacts(_deltaplusone, false);

	for (int i = 0; i < _NumMotors; i++) {
		_delta_step[i].push_back(filtered[i]);
		_delta_step[i].pop_front();

		//for (int k = 0; k < _delta_step[i].size(); k++) {
		//	cout << _delta_step[i][k] << " ";
		//}
		//cout << endl;
			
		for (int k = 1; k < 3; k++) {
			if (!deltaContacts[k])
				deltaContacts[k] = isDeltaLarge(i, k, 1);
		}			
		for (int k = 3; k < 5; k++) {
			if (!deltaContacts[k])
				deltaContacts[k] = isDeltaLarge(i, k, 2);
		}			
		for (int k = 5; k < 9; k++){
			if (!deltaContacts[k])
				deltaContacts[k] = isDeltaLarge(i, k, 3);
		}			
		for (int k = 9; k < 51; k++) {
			if (!deltaContacts[k])
				deltaContacts[k] = isDeltaLarge(i, k, 4);
		}			
	}

	for (int i = 0; i < 25; i++) {
		if (deltaContacts[2 * i + 1] && deltaContacts[2 * i + 2]) {
			//cout << 2 * i + 1 << "  " << 2 * i + 2 << endl;
			contact = 1;
			break;
		}
	}

	if (contactiter < resetIgnore) {
		contact = 0;
		contactiter++;
	}

	//cout << contact << endl;

	return contact;
}

//bool ContactHandler::isDeltaLarge(unsigned int motoridx, unsigned int stepdelta, int mode) {
//	
//	bool isDeltaLarge = false;
//	int diff = abs(_delta_step[motoridx].end()[-1] - _delta_step[motoridx].end()[-1 - stepdelta]);
//
//	if (mode == 2) {
//		if (diff > _label_limit_delta2[motoridx])
//			isDeltaLarge = true;
//	}		
//	else if (mode == 3) {
//		if (diff > _label_limit_delta3[motoridx])
//			isDeltaLarge = true;
//	}		
//	else if (mode > 3 && mode < 10) {
//		if (diff > _label_limit_med[motoridx])
//			isDeltaLarge = true;
//	}		
//	else {
//		if (diff > _label_limit_slow[motoridx])
//			isDeltaLarge = true;
//	}		
//
//	return isDeltaLarge;
//}

//int ContactHandler::getPresentContactLabel(vector<int> filtered, unsigned int resetIgnore, bool reset) {
//	
//	static int contactiter = 0;
//	if (reset) {
//		contactiter = 0;
//		//resetVectorDeque(_delta_step, _NumMotors, _deltaplusone, 0);
//		resetVectorDeque(_delta_step, _NumMotors);
//	}
//
//	if (contactiter == 0) {
//		resetVectorDeque(_delta_step, _NumMotors, _deltaplusone, filtered);
//	}
//
//	int contact = 0;	
//
//	vector<bool> deltaContacts(_deltaplusone, false);
//
//	for (int i = 0; i < _NumMotors; i++) {
//		_delta_step[i].push_back(filtered[i]);
//		_delta_step[i].pop_front();
//
//		//if (contactiter >= _deltaplusone - 1) {	
//			//if (isDeltaLarge(i, 2, 2)) deltaContacts[2] = true;
//			//if (isDeltaLarge(i, 3, 3)) deltaContacts[3] = true;
//			//if (isDeltaLarge(i, 9, 5)) deltaContacts[9] = true;
//			//if (isDeltaLarge(i, 10, 5)) deltaContacts[10] = true;
//			//if (isDeltaLarge(i, 24, 10)) deltaContacts[24] = true;
//			//if (isDeltaLarge(i, 25, 10)) deltaContacts[25] = true;
//			//if (isDeltaLarge(i, 49, 10)) deltaContacts[49] = true;
//			//if (isDeltaLarge(i, 50, 10)) deltaContacts[50] = true;			
//			for (int k = 1; k < 3; k++)
//				deltaContacts[k] = isDeltaLarge(i, k, 1);
//			for (int k = 3; k < 5; k++)
//				deltaContacts[k] = isDeltaLarge(i, k, 2);
//			for (int k = 5; k < 9; k++)
//				deltaContacts[k] = isDeltaLarge(i, k, 3);			
//			for (int k = 9; k < 51; k++)
//				deltaContacts[k] = isDeltaLarge(i, k, 4);
//		//}
//	}
//
//	for (int i = 0; i < 25; i++) {
//		if (deltaContacts[2 * i + 1] && deltaContacts[2 * i + 2]) {
//			cout << 2 * i + 1 << "  " << 2 * i + 2 << endl;
//			contact = 1;
//			break;
//		}
//	}
//
//	//bool c1 = false, c2 = false, c3 = false, c4 = false;
//	//if (deltaContacts[2] && deltaContacts[3]) {
//	//	cout << " 1 ";
//	//	c1 = true;
//	//}		
//	//if (deltaContacts[9] && deltaContacts[10]) {
//	//	cout << " 2 ";
//	//	c2 = true;
//	//}		
//	//if (deltaContacts[24] && deltaContacts[25]) {
//	//	cout << " 3 ";
//	//	c3 = true;
//	//}		
//	//if (deltaContacts[49] && deltaContacts[50]) {
//	//	cout << " 4 ";
//	//	c4 = true;
//	//}	
//	//
//	//if (c1)
//	//	contact = 1;
//
//	////if (c1 || c2 || c3 || c4)
//	////	contact = 1;
//
//	if (contactiter < resetIgnore || contactiter < _deltaplusone - 1) {
//		contact = 0;
//		contactiter++;
//	}
//
//	cout << contact << endl;
//
//	return contact;
//}

vector<float> ContactHandler::Normalize(vector<int> filtered, bool reset) {
	
	static int normiter = 0;
	if (reset) {
		normiter = 0;
		resetVectorDeque(_current_norm, _NumMotors);
	}

	vector<float> normalized(_NumMotors);
	for (int i = 0; i < _NumMotors; i++)
		normalized[i] = float(-filtered[i] - _currentMin[i]) / (_currentMax[i] - _currentMin[i]);
	
	if (normiter == 0) {
		resetVectorDeque(_current_norm, _NumMotors, _windowsize + 1, normalized);
		normiter = 1;
	}
	else {
		for (int i = 0; i < _NumMotors; i++) {
			_current_norm[i].push_back(normalized[i]);
			_current_norm[i].pop_front();
		}
	}

	return normalized;
}

void ContactHandler::DetermineClassificationExecution(int presentlabel, bool reset) {

	// ex)
	// continous 2 steps, ignore 5 steps
	// 1 1 x x x o <- o 순간부터 classification 실행

	if (reset) {
		resetDeque(_ignore_contact_stack, _ignore + 1, 0);
		_doClassifiacion = false;
	}

	if (_doClassifiacion)
		return;

	_ignore_contact_stack.push_back(presentlabel);
	_ignore_contact_stack.pop_front();

	bool start_classification = true;
	for (int i = 0; i < _continuous; i++) {
		if (!_ignore_contact_stack[i])
			start_classification = false;
	}

	_doClassifiacion = start_classification;
}

MatrixXd ContactHandler::getWindowFeature(bool do_classification) {

	MatrixXd windowfeature = MatrixXd::Zero(_NumMotors, _windowsize / _sampling + 1);

	if (do_classification) {
		for (int i = 0; i < _NumMotors; i++) {
			for (int j = _windowsize; j >= 0; j -= _sampling)
				windowfeature(i, j / _sampling) = _current_norm[i][j];
		}
	}

	return windowfeature;
}

int ContactHandler::getPresentClass(bool do_classification, MatrixXd input) {

	if (!do_classification)
		return 0;

	// input: 4 x 10
	VectorXd networkOutput = NetworkForward(input);

	//cout << fixed;
	//cout.precision(3);
	//cout << networkOutput.transpose() << endl;
	//cout.unsetf(ios::fixed);

	Eigen::Index maxCol;
	networkOutput.maxCoeff(&maxCol);

	return maxCol + 1; // 1~6
}

int ContactHandler::getContactClass(bool do_classification, int present_class, bool reset) {
	
	static int classifyiter = 0;
	if (reset) {
		classifyiter = 0;
		resetDeque(_check_contact_stack);
	}
	
	int contactlabel = 0;
	
	if (do_classification) {
		 
		contactlabel = -1; // contact, but class undetermined yet
		
		_check_contact_stack.push_back(present_class);
		classifyiter++;

		// count number of class predict steps
		vector<int> numOfPreds(_NumClasses, 0);
		for (int i = 0; i < _check_contact_stack.size(); i++) {
			if (1 <= _check_contact_stack[i] && _check_contact_stack[i] <= _NumClasses)
				numOfPreds[_check_contact_stack[i] - 1]++;
		}		

		for (int i = 0; i < _NumClasses; i++){
			if (numOfPreds[i] >= _classminnum) {		
				contactlabel = i + 1;
				break;
			}
		}

		if (classifyiter == _check && contactlabel == -1) {
			contactlabel = -2; // unknown
		}
	}

	return contactlabel;
}

VectorXd ContactHandler::NetworkForward(MatrixXd input) {		
	
	// input: 4 x 10	
	MatrixXd h;
	h = _model.calc_conv1d(input, "relu", _conv1);
	h = _model.calc_conv1d(h, "relu", _conv2);
	h = _model.flatten(h);
	h = _model.calc_linear(h, "softmax", _linear1);
	return h;
}

void ContactHandler::NetworkInitialize() {
		
	//using std::filesystem::current_path;
	//cout << "Current working directory: " << current_path() << endl;

	_conv1 = _model.load_conv1d_kernelbias(_NumMotors, 20, 5, 1, 0, 1, "Network/conv1d_1_Kb.txt");
	_conv2 = _model.load_conv1d_kernelbias(20, 20, 5, 1, 0, 1, "Network/conv1d_2_Kb.txt");
	_linear1 = _model.load_linear_weightbias(40, _NumClasses, "Network/linear_Wb.txt");
}








int ContactHandler::getDequeMean(deque<int> dq) {
	float mean = 0;
	for (int i = 0; i < dq.size(); i++)
		mean += dq[i];
	return round(mean / dq.size());
}

template <typename T>
void ContactHandler::resetVectorDeque(vector<deque<T>>& vdq, int vecsize) {
	vdq.clear();
	vdq.resize(vecsize);
	for (int i = 0; i < vecsize; i++) {
		deque<T> tmp;
		tmp.clear();
		vdq[i] = tmp;
	}
}

template <typename T>
void ContactHandler::resetVectorDeque(vector<deque<T>>& vdq, int vecsize, int dequesize, T dequeval) {
	vdq.clear();
	vdq.resize(vecsize);
	for (int i = 0; i < vecsize; i++) {
		deque<T> tmp(dequesize, dequeval);		
		vdq[i] = tmp;
	}
}

template <typename T>
void ContactHandler::resetVectorDeque(vector<deque<T>>& vdq, int vecsize, int dequesize, vector<T> dequevals) {
	vdq.clear();
	vdq.resize(vecsize);
	for (int i = 0; i < vecsize; i++) {
		deque<T> tmp(dequesize, dequevals[i]);
		vdq[i] = tmp;
	}
}

template <typename T>
void ContactHandler::printVectorDeque(vector<deque<T>> vdq) {
	for (int i = 0; i < vdq.size(); i++) {
		for (int j = 0; j < vdq[0].size(); j++)
			cout << vdq[i][j] << " ";
		cout << endl;
	}
}

template <typename T>
void ContactHandler::resetDeque(deque<T>& dq, int dequesize, T dequeval) {
	dq.clear();
	deque<T> tmp(dequesize, dequeval);
	dq = tmp;
}