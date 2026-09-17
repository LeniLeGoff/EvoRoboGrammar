#include "ea_rg/rg_simulator.hpp"
#include "ea_rg/rg_controllers.hpp"
#include "ea_rg/tasks.hpp"
#include "apear/async_dealer.hpp"
#include "apear/misc/utilities.hpp"


using namespace ea_rg;

int main(int argc, char** argv){

    apear::settings::ParametersMapPtr parameters = std::make_shared<apear::settings::ParametersMap>();
    parameters->emplace("#maxEpisodeTime",std::make_shared<apear::settings::Double>(-1));



    ea_rg::FlatArena env(4,4,ea_rg::fitness::Dummy());


    ea_rg::RoboGrammarSimulator sim(parameters,nullptr,false);
    env.init(sim);
    sim.init(nullptr);
    while(sim.step()){}


    return 0;
}
