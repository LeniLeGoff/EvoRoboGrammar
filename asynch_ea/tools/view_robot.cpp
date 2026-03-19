#include "ea_rg/rg_simulator.hpp"
#include "ea_rg/rg_controllers.hpp"
#include "ea_rg/tasks.hpp"
#include "apear/async_dealer.hpp"
#include "apear/misc/utilities.hpp"


using namespace ea_rg;

class ViewInd : public RoboGrammarInd{
public:
    using Ptr = std::shared_ptr<ViewInd>;
    using ConstPtr = std::shared_ptr<const ViewInd>;
    ViewInd() : RoboGrammarInd(){
        _morph_genome = std::make_shared<RoboGrammarGenome>();
        _ctrl_genome = std::make_shared<apear::EmptyGenome>();
    }
    ViewInd(const apear::misc::RandNum::Ptr& rn, const apear::settings::ParametersMapPtr &param) :
        RoboGrammarInd(rn,param){
        _morph_genome = std::make_shared<RoboGrammarGenome>(rn,param);
        _ctrl_genome = std::make_shared<apear::EmptyGenome>();
    }
    ViewInd(const RoboGrammarGenome::Ptr &morph_gen,const apear::EmptyGenome::Ptr &ctrl_gen) :
        RoboGrammarInd(morph_gen,ctrl_gen){}
    ViewInd(const ViewInd &ind) : RoboGrammarInd(ind){}

    Individual::Ptr clone() override{
        return std::make_shared<ViewInd>(*this);
    }

    void init() override{
        _morph_genome->init();
    }
    void set_rules(const std::vector<RoboGrammarGenome::rule_idx_t> &rule_seq){
        std::dynamic_pointer_cast<RoboGrammarGenome>(_morph_genome)->set_rule_seq(rule_seq);
        std::dynamic_pointer_cast<RoboGrammarGenome>(_morph_genome)->make_graph();
    }

private:
    void _create_controller() override{}
};

int main(int argc, char** argv){
    if(argc == 1){
        std::cout << "usage: arg1 : grammar file" << std::endl
                  << "\targs: rule sequence" << std::endl;
        return 1;
    }
    apear::settings::ParametersMapPtr parameters = std::make_shared<apear::settings::ParametersMap>();
    parameters->emplace("#grammarFile",std::make_shared<apear::settings::String>(std::string(argv[1])));
    parameters->emplace("#maxEpisodeTime",std::make_shared<apear::settings::Double>(-1));
    std::vector<RoboGrammarGenome::rule_idx_t> rule_seq;
    for(int i = 2; i < argc; i++){
        std::string arg(argv[i]);
        std::vector<std::string> tokens;
        apear::misc::split_line(arg,",",tokens);
        if(tokens.size() != 2){
            std::cout << "invalid rule sequence format, expected format: ruleType,ruleIdx" << std::endl;
            return 1;
        }
        int rule_type = std::stoi(tokens[0]);
        int rule_idx = std::stoi(tokens[1]);
        rule_seq.push_back(std::make_pair(rule_type,rule_idx));
    }


    ea_rg::FlatArena env(2,2,ea_rg::fitness::Dummy());

    ViewInd::Ptr ind = std::make_shared<ViewInd>(apear::misc::RandNum::Ptr(),parameters);
    ind->init();
    ind->set_rules(rule_seq);
    ea_rg::RoboGrammarSimulator sim(parameters,false);
    env.init(sim);
    sim.init(ind);
    while(sim.step()){}


    return 0;
}
