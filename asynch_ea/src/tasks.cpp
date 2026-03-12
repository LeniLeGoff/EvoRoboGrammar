#include "ea_rg/tasks.hpp"

using namespace ea_rg;
using namespace ea_rg::fitness;
namespace apear_st = apear::settings;
namespace rd = robot_design;

Exploration::Exploration(const apear_st::ParametersMapPtr &param){
    grid_size = apear_st::getParameter<apear_st::Sequence<int>>(param,"#gridSize").value;
    cell_size = apear_st::getParameter<apear_st::Double>(param,"#cellSize").value;
    grid_zones = Eigen::MatrixXi::Zero(grid_size[0],grid_size[1]);
}

std::vector<double> Exploration::operator()(RoboGrammarSimulator &sim){
    return {static_cast<double>(grid_zones.sum())/static_cast<double>(grid_size[0]*grid_size[1])};
}

void Exploration::update(RoboGrammarSimulator &sim){
    rd::Matrix4 transform;
    sim.sim()->getLinkTransform(sim.get_robot_idx(),0,transform);
    rd::Vector3 position = transform.block<>(1,3,3,1);
    std::pair<int,int> idx = real_to_matrix_coord(position);
    grid_zones(idx.first,idx.second) = 1;

}

std::pair<int,int> Exploration::real_to_matrix_coord(const rd::Vector3& pos){
    std::pair<int,int> indexes;
    indexes.first = std::trunc(pos[0]/cell_size + std::round(grid_size[0]/2));
    indexes.second = std::trunc(pos[1]/cell_size + std::round(grid_size[1]/2));
    if(indexes.first == grid_size[0])
        indexes.first = grid_size[0] - 1;
    if(indexes.second == grid_size[1])
        indexes.second = grid_size[1] - 1;
    return indexes;
}

void FlatTerrain::init(Sim &sim){

    rd::Prop floor(rd::PropShape::BOX, 0, 0.5, {40.0,1.0,10.0});
    sim.sim()->addProp(std::make_shared<rd::Prop>(floor), {0,0,0}, rd::Quaternion({1,0,0,0}));
}


std::vector<double> FlatTerrain::fitness_function(Sim &sim){
    return (*_fitness_fct)(sim);
}

void FlatTerrain::update(double time, Sim &sim){
    _fitness_fct->update(sim);
}

void FlatArena::init(Sim &sim){

    rd::Prop floor(rd::PropShape::BOX, 0, 0.5, {_width,0.5,_length});
    sim.sim()->addProp(std::make_shared<rd::Prop>(floor), {0,-0.5,0}, rd::Quaternion({1,0,0,0}));

    rd::Prop wall(rd::PropShape::BOX,0,0.5,{0.05,0.5,_length});
    sim.sim()->addProp(std::make_shared<rd::Prop>(wall),{_width,0.5,0},{1.0,0.0,0.0,0.0});
    sim.sim()->addProp(std::make_shared<rd::Prop>(wall),{-_width,0.5,0},{1.0,0.0,0.0,0.0});
    rd::Prop wall2(rd::PropShape::BOX,0,0.5,{_width,0.5,0.0});
    sim.sim()->addProp(std::make_shared<rd::Prop>(wall2),{0,0.5,_length},{1.0,0.0,0.0,0.0});
    sim.sim()->addProp(std::make_shared<rd::Prop>(wall2),{0,0.5,-_length},{1.0,0.0,0.0,0.0});

}


std::vector<double> FlatArena::fitness_function(Sim &sim){
    return (*_fitness_fct)(sim);
}

void FlatArena::update(double time, Sim &sim){
    _fitness_fct->update(sim);
}
