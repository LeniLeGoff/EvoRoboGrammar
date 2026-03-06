#include "rg_simulator.hpp"
#include "apear/settings.hpp"
#include "flat_terrain.hpp"
#include "rg_genome.hpp"

using namespace ea_rg;
namespace apear_st = apear::settings;



RoboGrammarSimulator::RoboGrammarSimulator(apear::settings::ParametersMapPtr &param) : apear::Simulator<RoboGrammarInd>(param){
    double time_step = apear_st::getParameter<apear_st::Double>(_parameters,"#simTimeStep").value;
    _sim = std::make_shared<rd::BulletSimulation>(time_step);
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
    _sim->addRobot(std::make_shared<rd::Robot>(robot),{pos[0],pos[1],pos[2]},{rot[0],rot[1],rot[2],rot[3]});
    return true;
}

bool RoboGrammarSimulator::step(){
    _sim->step();
    return true;
}

bool RoboGrammarSimulator::stop(){
    return true;
}

void RoboGrammarSimulator::update_ind(IndPtr &ind, const apear::Environment::Ptr& env){
    ind->set_objectives({env->fitness_function(ind)});
}

apear::sim_state_t RoboGrammarSimulator::state(){
    return apear::sim_state_t::IDLE;
}

double RoboGrammarSimulator::time(){
    return 0.0;
}

void RoboGrammarSimulator::reconnect(){

}
