#include <string>
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
    int load_rollout(const std::string& filename){
        std::ifstream file(filename);
        if(!file){
            std::cerr << "Unable to open file: " << filename << std::endl;
            return 0;
        }
        for(std::string line;std::getline(file,line);){
            apear::act_obs_t ao;
            ao.from_string(line);
            _rollout.push_back(ao);
        }
        return _rollout.size();
    }

private:
    void _create_controller() override{
        _control = std::make_shared<ViewerControl>(_rand_num,_parameters);
        std::dynamic_pointer_cast<ViewerControl>(_control)->set_rollout(_rollout);
    }
    apear::rollout_t _rollout;

};

int main(int argc, char** argv){
    if(argc == 1){
        std::cout << "usage: \n\targ1 : grammar file" << std::endl
                  << "\targ2: time step" << std::endl
                  << "\targ3: rollout file (csv)" << std::endl
                  << "\targs: rule sequence" << std::endl;
        return 1;
    }
    apear::settings::ParametersMapPtr parameters = std::make_shared<apear::settings::ParametersMap>();
    parameters->emplace("#grammarFile",std::make_shared<apear::settings::String>(std::string(argv[1])));
    parameters->emplace("#simTimeStep",std::make_shared<apear::settings::Double>(std::stod(argv[2])));
    // parameters->emplace("#initPosition",std::make_shared<apear::settings::Sequence<double>>(std::vector<double>({0,10,0})));
    std::vector<RoboGrammarGenome::rule_idx_t> rule_seq;
    for(int i = 4; i < argc; i++){
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


    ea_rg::FlatArena env(4,4,ea_rg::fitness::Dummy());

    ViewInd::Ptr ind = std::make_shared<ViewInd>(apear::misc::RandNum::Ptr(),parameters);
    ind->init();
    ind->set_rules(rule_seq);
    int nbr_of_step = ind->load_rollout(std::string(argv[3]));
    parameters->emplace("#maxEpisodeTime",std::make_shared<apear::settings::Double>(nbr_of_step * std::stod(argv[2])));
    ind->set_parameters(parameters);


    ea_rg::RoboGrammarSimulator sim(parameters,false);
    env.init(sim);
    sim.init(ind);
    int dof = sim.sim()->getRobotDofCount(sim.get_robot_idx());
    rd::VectorX torques(dof);
    while(sim.step()){
        sim.update_robot(ind);
        sim.sim()->getJointTorques(sim.get_robot_idx(),torques);
        std::cout << "torques: " << torques.transpose() << std::endl;
    }


    return 0;
}
