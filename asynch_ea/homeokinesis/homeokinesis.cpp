#include "homeokinesis.hpp"
#include "apear/algorithms/cpgrbfhk_controller.hpp"
#include "apear/algorithms/homeokinetic_controller.hpp"
#include "apear/settings.hpp"
#include "apear/misc/rand_num.hpp"
#include "ea_rg/tasks.hpp"
#include "ea_rg/io.hpp"

using namespace apear;


int main(int argc, char** argv){
    if(argc == 1){
        std::cout << "usage: arg1 : path to parameters file" << std::endl;
        return 1;
    }


    //load parameters
    settings::ParametersMapPtr parameters = std::make_shared<settings::ParametersMap>(
        settings::loadParameters(argv[1]));
    //initialise random number generator
    int seed = settings::getParameter<settings::Integer>(parameters,"#seed").value;
    if(seed == -1){
        std::random_device rd;
        seed = rd();
        settings::random::parameters->emplace("#seed",std::make_shared<const settings::Integer>(seed));
    }
    apear::misc::RandNum::Ptr rand_num = std::make_shared<apear::misc::RandNum>(seed);

    std::vector<RoboGrammarGenome::rule_idx_t> rule_seq;
    std::vector<int> rules = settings::getParameter<settings::Sequence<int>>(parameters,"#robotRules").value;
    if(rules.size()%2 != 0){
        std::cout << "invalid rule sequence format, needs an even number of rules" << std::endl;
        return 1;
    }
    for(int i = 0; i < rules.size(); i+=2){
        rule_seq.push_back(std::make_pair(rules[i],rules[i+1]));
    }

    double ctrl_freq = settings::getParameter<settings::Double>(parameters,"#ctrlFreq").value;
    double time_step = settings::getParameter<settings::Double>(parameters,"#simTimeStep").value;
    std::vector<double> arena_size = settings::getParameter<settings::Sequence<double>>(parameters,"#arenaSize").value;
    ea_rg::FlatArena env(arena_size[0],arena_size[1],ea_rg::fitness::Dummy());

    HKInd::Ptr ind = std::make_shared<HKInd>(rand_num,parameters);
    ind->init();
    ind->set_rules(rule_seq);
    ea_rg::RoboGrammarSimulator sim(parameters,rand_num,false);
    env.init(sim);
    sim.init(ind);
    int count = 0;
    do{
        sim.update_robot(ind);
        int step_counter = static_cast<int>(std::round(sim.time()/time_step));
        int ctrl_step  = static_cast<int>(std::round(ctrl_freq/time_step));
        if(step_counter%ctrl_step == 0)
            print::rollout(sim,false);
        print::torques(sim);
        // print::pose(sim);
        sleep(time_step);
        sim.get_sim()->saveState();
        std::stringstream sstr;
        sstr << "simulation_state_" << count;
        sim.get_sim()->write_state(sstr.str());
        count++;
    }while(sim.step());


    return 0;
}
