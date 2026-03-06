#include "rg_simulator.hpp"
#include "flat_terrain.hpp"
#include "apear/async_dealer.hpp"
#include "rg_genome.hpp"

using namespace ea_rg;

int main(int argc, char** argv){
    apear::settings::ParametersMapPtr param = std::make_shared<apear::settings::ParametersMap>(
    apear::settings::loadParameters("/home/leni/git/EvoRoboGrammar/asynch_ea/test_parameters.csv"));

    apear::misc::RandNum::Ptr rand_num = std::make_shared<apear::misc::RandNum>(apear::settings::getParameter<apear::settings::Integer>(param,"#seed").value);
    apear::AsyncDealer<RoboGrammarInd,RoboGrammarSimulator> dealer(param,rand_num);
    ea_rg::FlatTerrain::Ptr env = std::make_shared<ea_rg::FlatTerrain>();
    env->init();
    apear::EA<RoboGrammarInd>::Ptr res = std::make_unique<apear::RandomElitistSearch<RoboGrammarInd>>(rand_num,param);
    res->init();
    dealer.set_ea(res);
    dealer.set_environment(env);
    dealer.init(1);
    while(dealer.update_simulators()){
        usleep(5000);
    }
    return 0;
}
