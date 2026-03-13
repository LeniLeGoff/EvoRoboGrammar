#include "ea_rg/tasks.hpp"

using namespace ea_rg;
using namespace ea_rg::fitness;
namespace apear_st = apear::settings;
namespace rd = robot_design;

Exploration::Exploration(const apear_st::ParametersMapPtr &param){
    grid_size = apear_st::getParameter<apear_st::Sequence<int>>(param,"#gridSize").value;
    cell_size = apear_st::getParameter<apear_st::Double>(param,"#cellSize").value;
    verbose = apear_st::getParameter<apear_st::Boolean>(param,"#verbose").value;
    grid_zones = Eigen::MatrixXi::Zero(grid_size[0],grid_size[1]);
}

std::vector<double> Exploration::operator()(RoboGrammarSimulator &sim){
    double fitness =  static_cast<double>(grid_zones.sum())/static_cast<double>(grid_size[0]*grid_size[1]);
    grid_zones = Eigen::MatrixXi::Zero(grid_size[0],grid_size[1]);
    return {fitness};

}

bool Exploration::update(RoboGrammarSimulator &sim){
    rd::Vector3 position;
    rd::Quaternion orientation;
    sim.sim()->getRobotPositionAndOrientation(sim.get_robot_idx(),position,orientation);
    // std::cout << "robot position: " << position.transpose() << std::endl;

    std::pair<int,int> idx = real_to_matrix_coord(position);
    if(idx.first < 0 || idx.first >= grid_size[0] || idx.second < 0 || idx.second >= grid_size[1] || position[1] < 0){
        if(verbose)
            std::cout << "robot out of bounds with position " << position.transpose() << std::endl;
        return false;
    }
    grid_zones(idx.first,idx.second) = 1;
    return true;

}

std::pair<int,int> Exploration::real_to_matrix_coord(const rd::Vector3& pos){
    std::pair<int,int> indexes;
    indexes.first = std::trunc(pos[0]/cell_size + std::round(grid_size[0]/2));
    indexes.second = std::trunc(pos[2]/cell_size + std::round(grid_size[1]/2));
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

bool FlatTerrain::update(double time, Sim &sim){
    return _fitness_fct->update(sim);
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

bool FlatArena::update(double time, Sim &sim){
    return _fitness_fct->update(sim);
}
