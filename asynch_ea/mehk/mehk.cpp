#include "mehk.hpp"
#include "apear/algorithms/homeokinetic_controller.hpp"
#include "apear/async_dealer.hpp"
#include "apear/ame.hpp"

#include "ea_rg/tasks.hpp"

using namespace ea_rg;
namespace apear_st = apear::settings;

void MEHKInd::init(){
    _morph_genome->init();
    _morph_genome->random();
}


void MEHKInd::_create_controller(){
    int dof = get_robot_dof();
    if(dof == 0)
        return;
    _control = std::make_shared<apear::hk::Homeokinesis>(dof,dof);
    _control->set_random_number(_rand_num);
    _control->set_parameters(_parameters);

    if(apear_st::getParameter<apear_st::Boolean>(_parameters,"#initHKNoise").value)
        std::dynamic_pointer_cast<apear::hk::Homeokinesis>(_control)->add_noise(
            apear_st::getParameter<apear_st::Double>(_parameters,"#HKNoiseStrength").value);
}

void RGGenomeLog::register_data(const IndPtr &ind,const RoboGrammarSimulator &sim){
    std::stringstream genome;
    genome << ind->get_morph_genome()->to_string();
    _data.push_back(genome.str());
}

void RGGenomeLog::saveLog(){
    std::ofstream log_file;
    if(openOLogFile(log_file)){
        for(const std::string &line : _data){
            log_file << line << std::endl;
        }
        log_file.close();
    }
}


int main(int argc, char** argv){
    if(argc < 4){
        std::cout << "usage: " << std::endl
                  << "\targ 1: path to parameters file" << std::endl
                  << "\targ 2: number of simulators" << std::endl
                  << "\targ 3: headless mode (0|1)" << std::endl;
        return 1;
    }
    int nbr_sim = std::stoi(argv[2]);
    bool headless = std::stoi(argv[3]) == 1;


    apear_st::ParametersMapPtr param = std::make_shared<apear_st::ParametersMap>(
       apear_st::loadParameters(argv[1]));

    apear::logging::create_log_folder(apear_st::getParameter<apear_st::String>(param,"#experimentName").value);

    int seed = apear_st::getParameter<apear_st::Integer>(param,"#seed").value;
    if(seed == -1){
        std::random_device rd;
        seed = rd();
        apear_st::random::parameters->emplace("#seed",std::make_shared<const apear_st::Integer>(seed));
    }

    apear::misc::RandNum::Ptr rand_num = std::make_shared<apear::misc::RandNum>(seed);
    apear::AsyncDealer<MEHKInd,RoboGrammarSimulator> dealer(param,rand_num);

    dealer.add_logging(std::make_shared<RGGenomeLog>(apear_st::getParameter<apear_st::String>(param,"#genomeLogFile").value));

    std::vector<double> arena_size = apear_st::getParameter<apear_st::Sequence<double>>(param,"#arenaSize").value;
    ea_rg::FlatArena::Ptr env = std::make_shared<ea_rg::FlatArena>(arena_size[0],arena_size[1]);
    env->set_fitness_fct(std::make_shared<ea_rg::fitness::Exploration>(param));
    apear::EA<MEHKInd>::Ptr ame = std::make_unique<apear::AsyncMorphoEvolution<MEHKInd,RoboGrammarGenome>>(rand_num,param);
    ame->init();
    dealer.set_ea(ame);
    dealer.set_environment(env);
    dealer.init(nbr_sim,headless);
    while (dealer.update_simulators()) {}
    return 0;
}
