#define _USE_MATH_DEFINES

#include "matplotlibcpp.h"
#include <vector>
#include <iostream>
#include <windows.h>
#include <math.h>



namespace plt = matplotlibcpp;
using namespace std;
int main()
{
    std::vector<int> X;
    std::vector<int> y;
    plt::ion();
    
    /*
    plt::plot({ 1,2,3,4 }, "*");
    plt::show();
    plt::detail::_interpreter::kill();
    */
    double iter = 0;
    int limit = 10000;
    double theta = 0;
    while (iter <= limit) {
        theta = (iter / 20) * (2 * M_PI);
        cout << theta << endl;
        X.push_back(200 * cos(theta));
        y.push_back(200 * sin(theta) + 200);

        plt::scatter(X, y);
        plt::show();
        plt::pause(0.1);
        iter += 1;
    }
 
    plt::detail::_interpreter::kill();
}