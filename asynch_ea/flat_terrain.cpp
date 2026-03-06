#include "flat_terrain.hpp"

using namespace ea_rg;
namespace rd = robot_design;

void FlatTerrain::init(){
    _floor = rd::Prop(rd::PropShape::BOX, 0, 0.5, {40.0,1.0,10.0});
}

void FlatTerrain::add_terrain(std::shared_ptr<rd::BulletSimulation> &sim){
    sim->addProp(std::make_shared<rd::Prop>(_floor), {0,-1,0}, rd::Quaternion({1,0,0,0}));
}

std::vector<double> FlatTerrain::fitness_function(const apear::Individual::Ptr &ind){
    return {0};
}

void FlatTerrain::update_info(double time){

}
