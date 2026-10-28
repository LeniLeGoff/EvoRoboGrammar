#include "../mehk/mehk.hpp"
#include "apear/async_dealer.hpp"
#include "apear/algorithms/archives.hpp"
#include "apear/ame_qd.hpp"
#include "ea_rg/tasks.hpp"
#include "ea_rg/loggings.hpp"
#include "robot_design/graph.h"

using namespace ea_rg;
namespace apear_st = apear::settings;

using Archive = apear::UnstructuredArchive<HKInd::Ptr,int>;
using MEHKQD = apear::AsyncMorphoEvolutionQD<HKInd,RoboGrammarGenome,Archive>;


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


    //create the evolutionary algorithm: Asynchrounous Morpho-Evolution with Quality-Diversity and Homeokinetic controller MEHKQD
    apear::EA<HKInd>::Ptr ame = std::make_unique<MEHKQD>(rand_num,param);
    ame->init();

    //initialise an unstructured archive
    Archive::comparator_t comparator([](const HKInd::Ptr& a, const HKInd::Ptr& b) -> bool{//comparator for quality
        return a->get_objectives()[0] > b->get_objectives()[0];
    });
    Archive::distance_fct_t distance_fct([](const HKInd::Ptr& a, const HKInd::Ptr& b) -> int{//distance for diversity
        return robot_design::tree_edit_distance(std::dynamic_pointer_cast<RoboGrammarGenome>(a->get_morph_genome())->get_graph(),
                                                std::dynamic_pointer_cast<RoboGrammarGenome>(b->get_morph_genome())->get_graph());
    });
    dynamic_cast<MEHKQD*>(ame.get())->init_archive(1,distance_fct,comparator);

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


void ParentsLog::saveLog(const apear::EA<HKInd>::Ptr &ea){
    const std::vector<HKInd::Ptr>& inds = static_cast<MEHKQD*>(ea.get())->get_parent_pool().get_solutions();
    if(inds.empty())
        return;

    std::ifstream ifs;
    std::string l,line;
    if(this->openILogFile(ifs))
        while(std::getline(ifs,l))
            line = l;
    else return;

    std::stringstream sstr;
    sstr << inds.front()->get_morph_genome()->id();
    for(size_t i = 1; i < inds.size(); i++)
        sstr << "," << inds.at(i)->get_morph_genome()->id();
    if(!line.empty() && sstr.str() == line) //genome pool didn't change
        return;
    std::ofstream ofs;
    if(!openOLogFile(ofs))
        return;
    ofs << sstr.str() << std::endl;
    ofs.close();
}

