#pragma once

#include "apear/environment.hpp"
#include "robot_design/sim.h"

namespace ea_rg{

namespace rd = robot_design;

class FlatTerrain : public apear::Environment{
public:
    FlatTerrain(){}
    void init() override;
    std::vector<double> fitness_function(const apear::Individual::Ptr &ind) override;
    void update_info(double time) override;
    void add_terrain(std::shared_ptr<rd::BulletSimulation> &sim);
private:
    rd::Prop _floor;
};

}
