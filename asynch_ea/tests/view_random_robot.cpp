#include "ea_rg/rg_simulator.hpp"
#include "ea_rg/rg_controllers.hpp"
#include "ea_rg/tasks.hpp"
#include "apear/async_dealer.hpp"


using namespace ea_rg;

class RandomInd : public RoboGrammarInd{
public:
    RandomInd() : RoboGrammarInd(){
    }
    RandomInd(const apear::misc::RandNum::Ptr& rn, const apear::settings::ParametersMapPtr &param) :
        RoboGrammarInd(rn,param){
        _morph_genome = std::make_shared<RoboGrammarGenome>(rn,param);
        _ctrl_genome = std::make_shared<apear::EmptyGenome>();
    }
    RandomInd(const RoboGrammarGenome::Ptr &morph_gen,const apear::EmptyGenome::Ptr &ctrl_gen) :
        RoboGrammarInd(morph_gen,ctrl_gen){}
    RandomInd(const RandomInd &ind) : RoboGrammarInd(ind){}

    Individual::Ptr clone() override{
        return std::make_shared<RandomInd>(*this);
    }

    void init() override{
        _morph_genome->init();
        _morph_genome->random();
    }

private:
    void _create_controller() override{}
};

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

    std::vector<double> arena_size = apear::settings::getParameter<apear::settings::Sequence<double>>(param,"#arenaSize").value;
    apear::misc::RandNum::Ptr rand_num = std::make_shared<apear::misc::RandNum>(seed);
    ea_rg::FlatArena env(arena_size[0],arena_size[1]);
    env.set_fitness_fct(std::make_shared<ea_rg::fitness::Dummy>());

    RandomInd::Ptr ind = std::make_shared<RandomInd>(rand_num,param);
    ind->init();

    ea_rg::RoboGrammarSimulator sim(param,false);
    env.init(sim);
    sim.init(ind);
    while(sim.step()){}


    return 0;
}
