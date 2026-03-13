#include "test.hpp"
#include "ea_rg/rg_simulator.hpp"
#include "ea_rg/rg_controllers.hpp"
#include "ea_rg/tasks.hpp"
#include "apear/async_dealer.hpp"


using namespace ea_rg;

void RandomInd::init(){
    _morph_genome->init();
    _morph_genome->random();
}


void RandomInd::_create_controller(){
    _control = std::make_shared<RandomControl>(_rand_num,_parameters);
    int dof = get_robot_dof();
    std::dynamic_pointer_cast<RandomControl>(_control)->init(dof);
}

int main(int argc, char** argv){
    if(argc < 2){
        std::cout << "usage: arg 1: path to parameters file" << std::endl;
        return 1;
    }

    apear::settings::ParametersMapPtr param = std::make_shared<apear::settings::ParametersMap>(
        apear::settings::loadParameters(argv[1]));

    int seed = apear::settings::getParameter<apear::settings::Integer>(param,"#seed").value;
    if(seed == -1){
        std::random_device rd;
        seed = rd();
        apear::settings::random::parameters->emplace("#seed",std::make_shared<const apear::settings::Integer>(seed));
    }

    apear::misc::RandNum::Ptr rand_num = std::make_shared<apear::misc::RandNum>(seed);
    apear::AsyncDealer<RandomInd,RoboGrammarSimulator> dealer(param,rand_num);
    ea_rg::FlatArena::Ptr env = std::make_shared<ea_rg::FlatArena>(4,4);
    env->set_fitness_fct(std::make_shared<ea_rg::fitness::Exploration>(param));
    apear::EA<RandomInd>::Ptr res = std::make_unique<apear::RandomElitistSearch<RandomInd>>(rand_num,param);
    res->init();
    dealer.set_ea(res);
    dealer.set_environment(env);
    dealer.init(1,false);
    while(dealer.update_simulators()){
        usleep(5000);
    }
    return 0;
}
