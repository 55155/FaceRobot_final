#pragma once

#include "MacrosAndFunctions.h"

//#include <vector>
//#include <Eigen/Core>
//#include<iostream>
//#include<string>
//#include<fstream>
//
//using namespace std;
//using namespace Eigen;

class NETWORK_ACTIVATIONS
{
public:
	MatrixXd act_linear(MatrixXd x) {
		return x;
	}
	MatrixXd act_tanh(MatrixXd x) {
		for (int i = 0; i < x.rows(); i++)
			for (int j = 0; j < x.cols(); j++)
				x(i, j) = tanh(x(i, j));
		return x;
	}
	MatrixXd act_relu(MatrixXd x) {
		x = (x.array() < 0).select(0, x);
		return x;
	}
	MatrixXd act_sigmoid(MatrixXd x) {
		for (int i = 0; i < x.rows(); i++)
			for (int j = 0; j < x.cols(); j++)
				x(i, j) = 1 / (1 + exp(-x(i, j)));
		return x;
	}
	MatrixXd act_softmax(MatrixXd x) {
		MatrixXd exps(x.rows(), x.cols());
		double totalsum = 0;
		for (int i = 0; i < x.rows(); i++) {
			for (int j = 0; j < x.cols(); j++) {
				exps(i, j) = exp(x(i, j));
				totalsum += exps(i, j);
			}
		}
		for (int i = 0; i < x.rows(); i++)
			for (int j = 0; j < x.cols(); j++)
				x(i, j) = exps(i, j) / totalsum;
		return x;
	}
};

struct LAYER_LINEAR
{
	MatrixXd _weight;
	VectorXd _bias;
};

class NETWORK_LINEAR : public NETWORK_ACTIVATIONS
{
public:
	MatrixXd calc_linear(VectorXd x_in, string activation, LAYER_LINEAR layerparams);
	LAYER_LINEAR load_linear_weightbias(int indim, int outdim, string filename);
};

MatrixXd NETWORK_LINEAR::calc_linear(VectorXd x_in, string activation, LAYER_LINEAR layerparams)
{
	MatrixXd W = layerparams._weight;
	VectorXd b = layerparams._bias;

	int f_in = W.cols();
	int f_out = W.rows();

	MatrixXd x_out = W * x_in + b;

	if (activation == "relu")
		x_out = (x_out.array() < 0).select(0, x_out);
	else if (activation == "sigmoid") {
		for (int i = 0; i < f_out; i++)
			x_out(i) = 1 / (1 + exp(-x_out(i)));
	}
	else if (activation == "softmax") {
		VectorXd exps(f_out);
		double totalsum = 0;
		for (int i = 0; i < f_out; i++) {
			exps(i) = exp(x_out(i));
			totalsum += exps(i);
		}
		for (int i = 0; i < f_out; i++)
			x_out(i) = exps(i) / totalsum;
	}

	return x_out;
}

LAYER_LINEAR NETWORK_LINEAR::load_linear_weightbias(int indim, int outdim, string filename)
{
	vector<double> weight_and_bias_from_text;

	ifstream file(filename);
	double num;
	if (file.is_open()) {
		while (file >> num)
			weight_and_bias_from_text.push_back(num);
	}
	else
		cout << "linear file read ERROR" << endl;
	file.close();

	if (weight_and_bias_from_text.size() != outdim * indim + outdim)
		cout << "linear layer input-output size error" << endl;

	MatrixXd weight(outdim, indim);
	VectorXd bias(outdim);

	int idx = 0;
	for (int io = 0; io < outdim; io++) {
		for (int ii = 0; ii < indim; ii++) {
			weight(io, ii) = weight_and_bias_from_text[idx];
			idx++;
		}
	}
	for (int io = 0; io < outdim; io++) {
		bias(io) = weight_and_bias_from_text[idx];
		idx++;
	}

	LAYER_LINEAR layerparams;

	layerparams._weight = weight;
	layerparams._bias = bias;

	return layerparams;
}

struct LAYER_1DCNN
{
	vector<MatrixXd> _kernel;
	VectorXd _bias;
	int _stride, _padding, _dilation;
};

class NETWORK_1DCNN : public NETWORK_LINEAR
{
public:
	MatrixXd calc_conv1d(MatrixXd x_in, string activation, struct LAYER_1DCNN layerparams);
	VectorXd flatten(MatrixXd x_in);
	LAYER_1DCNN load_conv1d_kernelbias(int ch_in, int ch_out, int kernel_size, int stride, int padding, int dilation, string filename);
};

MatrixXd NETWORK_1DCNN::calc_conv1d(MatrixXd x_in, string activation, struct LAYER_1DCNN layerparams)
{
	vector<MatrixXd> filter_list = layerparams._kernel;
	VectorXd bias_list = layerparams._bias;
	int pad_size = layerparams._padding;
	int dilation_size = layerparams._dilation;
	int stride_size = layerparams._stride;

	int f_in_x = x_in.cols();
	int in_ch = x_in.rows();
	int out_ch = filter_list.size();
	int kernel_size = filter_list[0].cols(); // rows(): in_ch

	int f_out_x = floor((f_in_x + 2 * pad_size - dilation_size * (kernel_size - 1) - 1) / stride_size) + 1;

	MatrixXd x_in_padded = MatrixXd::Zero(in_ch, f_in_x + 2 * pad_size);
	for (int i = 0; i < in_ch; i++)
		for (int j = 0; j < f_in_x; j++)
			x_in_padded(i, j + pad_size) = x_in(i, j);

	MatrixXd x_out(out_ch, f_out_x);
	for (int k = 0; k < out_ch; k++) {
		MatrixXd kernel = filter_list[k];
		for (int f = 0; f < f_out_x; f++) { // output index iteration
			x_out(k, f) = bias_list[k];
			for (int i = 0; i < in_ch; i++) {
				for (int fi = 0; fi < kernel_size; fi++) // kernel index iteration
					x_out(k, f) += x_in_padded(i, stride_size * f + dilation_size * fi) * kernel(i, fi);
			}
			if (activation == "relu")
				x_out(k, f) = x_out(k, f) < 0 ? 0 : x_out(k, f);
			else if (activation == "sigmoid")
				x_out(k, f) = 1 / (1 + exp(-x_out(k, f)));
		}
	}

	return x_out;
}

VectorXd NETWORK_1DCNN::flatten(MatrixXd x_in)
{
	int channels = x_in.rows();
	int finalsize = x_in.cols();

	VectorXd x_out(channels * finalsize);
	int idx = 0;
	for (int ch = 0; ch < channels; ch++) {
		for (int i = 0; i < finalsize; i++) {
			x_out(idx) = x_in(ch, i);
			idx++;
		}
	}

	return x_out;
}

LAYER_1DCNN NETWORK_1DCNN::load_conv1d_kernelbias(int ch_in, int ch_out, int kernel_size, int stride, int padding, int dilation, string filename)
{
	vector<double> kernel_and_bias_from_text;

	ifstream file(filename);
	double num;
	if (file.is_open()) {
		while (file >> num)
			kernel_and_bias_from_text.push_back(num);
	}
	else
		cout << "conv1d file read ERROR" << endl;
	file.close();

	if (kernel_and_bias_from_text.size() != ch_out * ch_in * kernel_size + ch_out)
		cout << "conv1d layer channel and kernel size error" << endl;

	vector<MatrixXd> conv1d_kernel;
	conv1d_kernel.resize(ch_out);

	int idx = 0;
	for (int cho = 0; cho < ch_out; cho++) {
		MatrixXd K(ch_in, kernel_size);
		for (int chi = 0; chi < ch_in; chi++) {
			for (int x = 0; x < kernel_size; x++) {
				K(chi, x) = kernel_and_bias_from_text[idx];
				idx++;
			}
		}
		conv1d_kernel[cho] = K;
	}
	VectorXd conv1d_bias(ch_out);
	for (int cho = 0; cho < ch_out; cho++) {
		conv1d_bias(cho) = kernel_and_bias_from_text[idx];
		idx++;
	}

	LAYER_1DCNN layerparams;

	layerparams._kernel = conv1d_kernel;
	layerparams._bias = conv1d_bias;
	layerparams._stride = stride;
	layerparams._padding = padding;
	layerparams._dilation = dilation;

	return layerparams;
}