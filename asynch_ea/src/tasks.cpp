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
    if(idx.first < 0 || idx.first >= grid_size[0] || idx.second < 0 || idx.second >= grid_size[1] || position[1] < -0.01){
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

MovementExploStability::MovementExploStability(const apear_st::ParametersMapPtr &param){
    stability_threshold = apear_st::getParameter<apear_st::Double>(param,"#stabilityThreshold").value;
    stability_reward = apear_st::getParameter<apear_st::Double>(param,"#stabilityReward").value;
    nbr_nearest_neighbours = apear_st::getParameter<apear_st::Integer>(param,"#nbrNearestNeighbours").value;
    x_norm = apear_st::getParameter<apear_st::Sequence<double>>(param,"#arenaSize").value[0];
    y_norm = apear_st::getParameter<apear_st::Sequence<double>>(param,"#arenaSize").value[1];
}

std::vector<double> MovementExploStability::operator()(RoboGrammarSimulator &sim){
    if(time_step == 0)
        return {0};
    double explo_score = 0;
    for(const double& score: explo_scores)
        explo_score += score;
    double stab_score = 0;
    for(const double& score: stab_scores)
        stab_score += score;
    double obj = explo_score + stab_score/time_step;
    stab_scores.clear();
    explo_scores.clear();
    poses_archive.clear();
    time_step = 0;
    return {obj};
}


bool MovementExploStability::update(RoboGrammarSimulator &sim){
    rd::Vector3 position;
    rd::Quaternion orientation;
    sim.sim()->getRobotPositionAndOrientation(sim.get_robot_idx(),position,orientation);
    Eigen::VectorXd pose(7);
    pose << position[0]/x_norm,position[1],position[2]/y_norm,
        orientation.x(),orientation.y(),orientation.z(),orientation.w();
    poses_archive.push_back(pose);

    time_step = poses_archive.size()-1;

    //compute stability score
    if(poses_archive.size() == 1)
        return true;

    compute_exploration_score();
    compute_stability_score();
    return true;
}

void MovementExploStability::compute_stability_score(){
    if((poses_archive[time_step-1].tail(4)-poses_archive[time_step].tail(4)).norm() < stability_threshold)
        stab_scores.push_back(stability_reward);
    else stab_scores.push_back(0);
}

void MovementExploStability::compute_exploration_score(){
    std::vector<Eigen::VectorXd> poses_archive_copy = poses_archive;
    poses_archive_copy.pop_back();
    std::vector<double> distances;
    for(const Eigen::VectorXd& pose: poses_archive_copy){
        distances.push_back((pose-poses_archive.back()).norm());
    }
    std::sort(distances.begin(),distances.end());
    double score = 0;
    for(int i = 0; i < nbr_nearest_neighbours; i++){
        if(static_cast<size_t>(i) >= distances.size())
            break;
        score += distances[i];
    }
    explo_scores.push_back(score/static_cast<double>(nbr_nearest_neighbours));
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
