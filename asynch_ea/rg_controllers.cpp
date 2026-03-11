#include "rg_controllers.hpp"
#include <cmath>

using namespace ea_rg;

std::vector<double> RandomControl::update(const std::vector<double> &sensorValues){
    // std::cout << "current target: ";
    for(size_t i = 0; i < sensorValues.size(); i++){
        if(fabs(current_target[i]-sensorValues[i]) < 0.05)
            current_target[i] += _rand_num->rand_double(-0.5,0.5);
        // std::cout << current_target[i]*M_PI_2 << " ";
    }
    // std::cout << std::endl;

    return current_target;
}
