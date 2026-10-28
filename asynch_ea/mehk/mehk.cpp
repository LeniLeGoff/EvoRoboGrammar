#include "mehk.hpp"
#include "apear/algorithms/homeokinetic_controller.hpp"
#include "apear/async_dealer.hpp"
#include "ea_rg/tasks.hpp"
#include "ea_rg/loggings.hpp"

using namespace ea_rg;
namespace apear_st = apear::settings;


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

    //load parameters
    apear_st::ParametersMapPtr param = std::make_shared<apear_st::ParametersMap>(
       apear_st::loadParameters(argv[1]));

    //create log folder
    std::string log_repo = apear_st::getParameter<apear_st::String>(param,"#logRepository").value;
    apear::logging::create_log_folder(log_repo + "/" + apear_st::getParameter<apear_st::String>(param,"#experimentName").value);

    //initialise random number generator
    int seed = apear_st::getParameter<apear_st::Integer>(param,"#seed").value;
    if(seed == -1){
        std::random_device rd;
        seed = rd();
        apear_st::random::parameters->emplace("#seed",std::make_shared<const apear_st::Integer>(seed));
    }
    apear::misc::RandNum::Ptr rand_num = std::make_shared<apear::misc::RandNum>(seed);

    //create the asynchronous dealer
    apear::AsyncDealer<HKInd,RoboGrammarSimulator> dealer(param,rand_num);

    //add logging classes
    dealer.add_logging(std::make_shared<RGGenomeLog<HKInd>>(apear_st::getParameter<apear_st::String>(param,"#genomeLogFile").value));
    dealer.add_logging(std::make_shared<FitnessLog<HKInd>>(apear_st::getParameter<apear_st::String>(param,"#fitnessLogFile").value));
    dealer.add_logging(std::make_shared<ParentsLog>(apear_st::getParameter<apear_st::String>(param,"#parentsLogFile").value));
    dealer.add_logging(std::make_shared<RolloutLog<HKInd>>());
    dealer.add_logging(std::make_shared<TrajectoryLog<HKInd>>());


    //create the evolutionary algorithm: Asynchrounous Morpho-Evolution with Homeokinetic controller
    apear::EA<HKInd>::Ptr ame = std::make_unique<apear::AsyncMorphoEvolution<HKInd,RoboGrammarGenome>>(rand_num,param);
    ame->init();

    //initialise the asynchronous dealer
    dealer.set_ea(ame);
    dealer.init(nbr_sim,headless);

    //define the environment and task
    std::vector<double> arena_size = apear_st::getParameter<apear_st::Sequence<double>>(param,"#arenaSize").value;

    dealer.set_environment<ea_rg::FlatArena>(arena_size[0],arena_size[1],ea_rg::fitness::MovementExploStability(param));

    apear_st::saveParameters(apear::logging::log_folder + "/parameters.csv",param);

    //run the experiment
    while (dealer.update_simulators()) {}

    return 0;
}

//LOGGING CLASSES

void ParentsLog::saveLog(const apear::EA<HKInd>::Ptr &ea){
    int pop_size = apear_st::getParameter<apear_st::Integer>(ea->get_parameters(),"#populationSize").value;
    if(static_cast<apear::AsyncMorphoEvolution<HKInd,RoboGrammarGenome>*>(ea.get())->get_parent_pool().size() < pop_size)
        return;
    std::ofstream log_file;
    if(openOLogFile(log_file)){
        const std::vector<IndPtr> &parents = static_cast<apear::AsyncMorphoEvolution<HKInd,RoboGrammarGenome>*>(ea.get())->get_parent_pool();
        log_file << parents.front()->id();
        for(size_t i = 1; i < parents.size(); i++){
            log_file << "," << parents[i]->id();
        }
        log_file << std::endl;
        log_file.close();
    }
}


