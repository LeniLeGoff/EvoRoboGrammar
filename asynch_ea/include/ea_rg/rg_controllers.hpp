#pragma once
#include "apear/control.hpp"

namespace ea_rg {

class RandomControl : public apear::Control
{
public:
    typedef std::shared_ptr<RandomControl> Ptr;
    typedef std::shared_ptr<const RandomControl> ConstPtr;

    RandomControl() : apear::Control(){}
    RandomControl(const apear::misc::RandNum::Ptr& rand_num, const apear::settings::ParametersMapPtr &param)
        : apear::Control(rand_num,param){}

    RandomControl(const RandomControl& ctrl) :
        apear::Control(ctrl)
    {}

    Control::Ptr clone() const override{
        return std::make_shared<RandomControl>(*this);
    }

    std::vector<double> update(const std::vector<double> &sensorValues) override;

    void init(int size){
        _size = size;
        current_target = _rand_num->rand_vectd(-1.0,1.0,size);
    }

    void set_size(int size){
        _size = size;
        current_target = std::vector<double>(size,0.0);
    }

private:
    int _size = 0;
    std::vector<double> current_target;

};
}
