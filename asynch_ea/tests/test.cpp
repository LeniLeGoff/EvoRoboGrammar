#include "ea_rg/rg_simulator.hpp"
#include "ea_rg/tasks.hpp"
#include "ea_rg/rg_genome.hpp"
#include "apear/async_dealer.hpp"


using namespace ea_rg;

int main(int argc, char** argv){
    if(argc < 2){
        std::cout << "usage: arg 1: path to parameters file" << std::endl;
        return 1;
    }

    apear::settings::ParametersMapPtr param = std::make_shared<apear::settings::ParametersMap>(
        apear::settings::loadParameters(argv[1]));

    int seed = apear::settings::getParameter<apear::settings::Integer>(param,"#seed").value;
    if(seed < 0){
        std::random_device rd;
        seed = rd();
        apear::settings::random::parameters->emplace("#seed",std::make_shared<const apear::settings::Integer>(seed));
    }

    apear::misc::RandNum::Ptr rand_num = std::make_shared<apear::misc::RandNum>(seed);
    apear::AsyncDealer<RoboGrammarInd,RoboGrammarSimulator> dealer(param,rand_num);
    ea_rg::FlatArena::Ptr env = std::make_shared<ea_rg::FlatArena>(4,4);
    env->set_fitness_fct(std::make_shared<ea_rg::fitness::Exploration>(param));
    apear::EA<RoboGrammarInd>::Ptr res = std::make_unique<apear::RandomElitistSearch<RoboGrammarInd>>(rand_num,param);
    res->init();
    dealer.set_ea(res);
    dealer.set_environment(env);
    dealer.init(1,false);
    while(dealer.update_simulators()){
        usleep(5000);
    }
    return 0;
}
