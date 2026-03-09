#include "rg_simulator.hpp"
#include "apear/settings.hpp"
#include "flat_terrain.hpp"
#include "rg_genome.hpp"
#include <cmath>

using namespace ea_rg;
namespace apear_st = apear::settings;



RoboGrammarSimulator::RoboGrammarSimulator(apear::settings::ParametersMapPtr &param, bool headless)
    : apear::Simulator<RoboGrammarInd>(param,headless){
    _max_episode_time = apear_st::getParameter<apear_st::Double>(_parameters,"#maxEpisodeTime").value;
    _time_step = apear_st::getParameter<apear_st::Double>(_parameters,"#simTimeStep").value;
    _sim = std::make_shared<rd::BulletSimulation>(_time_step);
}



bool RoboGrammarSimulator::init_environment(const apear::Environment::Ptr &env){
    if(env == nullptr)
        return false;
    std::dynamic_pointer_cast<FlatTerrain>(env)->add_terrain(_sim);
    return true;
}

bool RoboGrammarSimulator::init(const IndPtr &ind){
    std::vector<double> pos = apear_st::getParameter<apear_st::Sequence<double>>(_parameters,"#initPosition").value;
    std::vector<double> rot = apear_st::getParameter<apear_st::Sequence<double>>(_parameters,"#initOrientation").value;

    ind->decode();
    rd::Robot robot = std::dynamic_pointer_cast<RoboGrammarInd>(ind)->get_robot();
    _robot_idx = _sim->addRobot(std::make_shared<rd::Robot>(robot),{pos[0],pos[1],pos[2]},{rot[0],rot[1],rot[2],rot[3]});
    _state = apear::sim_state_t::INITIALIZED;
    if(!_headless){
        if(_viewer != nullptr)
            _viewer.reset();
        _viewer = std::make_shared<rd::GLFWViewer>();
        _viewer->camera_params_.yaw_ = -M_PI_2;
        _viewer->camera_params_.pitch_ = -M_PI/6;
        rd::Vector3 lower = {0,0,0};
        rd::Vector3 upper = {0,0,0};
        _sim->getRobotWorldAABB(_robot_idx,lower,upper);
        _viewer->camera_params_.distance_ = 10 * (upper - lower).squaredNorm();
    }
    return true;
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

void RoboGrammarSimulator::update_ind(IndPtr &ind, const apear::Environment::Ptr& env){
    ind->set_objectives({env->fitness_function(ind)});
    _state = apear::sim_state_t::IDLE;
}

apear::sim_state_t RoboGrammarSimulator::state(){
    return _state;
}

double RoboGrammarSimulator::time(){
    return _time;
}

void RoboGrammarSimulator::reconnect(){

}
