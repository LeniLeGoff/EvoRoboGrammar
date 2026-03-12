#include "ea_rg/rg_simulator.hpp"
#include "apear/settings.hpp"
#include "ea_rg/tasks.hpp"
#include "ea_rg/rg_genome.hpp"
#include <cmath>

using namespace ea_rg;
namespace apear_st = apear::settings;



RoboGrammarSimulator::RoboGrammarSimulator(apear::settings::ParametersMapPtr &param, bool headless)
    : apear::Simulator<RoboGrammarInd>(param,headless){
    _max_episode_time = apear_st::getParameter<apear_st::Double>(_parameters,"#maxEpisodeTime").value;
    _time_step = apear_st::getParameter<apear_st::Double>(_parameters,"#simTimeStep").value;
    _sim = std::make_shared<rd::BulletSimulation>(_time_step);
}


bool RoboGrammarSimulator::init(const IndPtr &ind){
    std::vector<double> pos = apear_st::getParameter<apear_st::Sequence<double>>(_parameters,"#initPosition").value;
    std::vector<double> rot = apear_st::getParameter<apear_st::Sequence<double>>(_parameters,"#initOrientation").value;

    ind->decode();
    rd::Robot robot = std::dynamic_pointer_cast<RoboGrammarInd>(ind)->get_robot();
    _robot_idx = _sim->addRobot(std::make_shared<rd::Robot>(robot),{pos[0],pos[1],pos[2]},{rot[0],rot[1],rot[2],rot[3]});
    _sim->setJointTargetPositions(_robot_idx,
                                  rd::VectorX::Zero(_sim->getRobotDofCount(_robot_idx)));
    for(int i = 0; i < 100; i++)
        _sim->step();

    _state = apear::sim_state_t::INITIALIZED;
    if(!_headless){
        if(_viewer != nullptr)
            _viewer.reset();
        _viewer = std::make_shared<rd::GLFWViewer>();
        _viewer->camera_params_.yaw_ = -M_PI/4;
        _viewer->camera_params_.pitch_ = -M_PI/6;
        rd::Vector3 lower = {0,0,0};
        rd::Vector3 upper = {0,0,0};
        _sim->getRobotWorldAABB(_robot_idx,lower,upper);
        _viewer->camera_params_.distance_ = 2 * (upper - lower).squaredNorm();
    }
    return true;
}

void RoboGrammarSimulator::update_robot(const IndPtr &ind){
    int dof = _sim->getRobotDofCount(_robot_idx);
    rd::VectorX current_pos(dof);
    _sim->getJointPositions(_robot_idx,current_pos);
    std::vector<double> current_pos_std(current_pos.rows());
    for(int i = 0; i < current_pos.rows(); i++){
        // std::cout << current_pos[i] << " ";
        current_pos_std[i] = current_pos[i]/M_PI_2; //scale the joint positions to [-1,1]
    }
    // std::cout << std::endl;
    std::vector<double> next_pos_std = ind->get_control()->update(current_pos_std);
    rd::VectorX next_pos(next_pos_std.size());
    for(size_t i = 0; i < next_pos_std.size(); i++)
        next_pos[i] = next_pos_std[i]*M_PI/2; //scale the control output to [-pi/2,pi/2]
    _sim->setJointTargetPositions(_robot_idx,next_pos);
}

bool RoboGrammarSimulator::step(){
    _sim->step();
    if(!_headless){
        _viewer->update(_time_step);
        _viewer->render(*_sim);
    }
    _time += _time_step;
    if(_time >= _max_episode_time)
        stop();
    return true;
}

bool RoboGrammarSimulator::stop(){
    _sim->removeRobot(_robot_idx);
    _state = apear::sim_state_t::FINISHED;
    _time = 0;
    return true;
}



apear::sim_state_t RoboGrammarSimulator::state(){
    return _state;
}

double RoboGrammarSimulator::time(){
    return _time;
}

void RoboGrammarSimulator::reconnect(){

}
