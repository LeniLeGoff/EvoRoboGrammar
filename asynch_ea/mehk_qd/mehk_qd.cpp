#include "mehk_qd.hpp"
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
    int selector_type = apear_st::getParameter<apear_st::Integer>(param,"#archiveSelectorType").value;
    if(selector_type == ea_rg::RANDOM)
        dynamic_cast<MEHKQD*>(ame.get())->init_archive(1,distance_fct,comparator,ea_rg::selectors::random);
    else if(selector_type == ea_rg::SPARSITY)
        dynamic_cast<MEHKQD*>(ame.get())->init_archive(1,distance_fct,comparator,ea_rg::selectors::sparsity);
    else if(selector_type == ea_rg::PARETO)
            dynamic_cast<MEHKQD*>(ame.get())->init_archive(1,distance_fct,comparator,ea_rg::selectors::pareto_tournament);

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
apear::selectors::selector_t<HKInd::Ptr> ea_rg::selectors::random = apear::selectors::random<HKInd::Ptr>;
apear::selectors::selector_t<HKInd::Ptr> ea_rg::selectors::sparsity = [](const std::vector<HKInd::Ptr>& solutions,const apear::misc::RandNum::Ptr & rand_num) -> const HKInd::Ptr &{
    if(solutions.size() == 1)
        return solutions.at(0);
    std::vector<float> sparsity_scores;
    std::vector<int> distances;
    for(const HKInd::Ptr &ind1: solutions){
        for(const HKInd::Ptr &ind2: solutions){
            if(ind1->get_morph_genome()->id() == ind2->get_morph_genome()->id())
                continue;
            distances.push_back(robot_design::tree_edit_distance(std::dynamic_pointer_cast<RoboGrammarGenome>(ind1->get_morph_genome())->get_graph(),
                                                                 std::dynamic_pointer_cast<RoboGrammarGenome>(ind2->get_morph_genome())->get_graph()));
            std::sort(distances.begin(),distances.end());
            float sparsity_score = 0;
            size_t nbr_ngb = std::min(static_cast<size_t>(15),distances.size());
            for(size_t i = 0; i < nbr_ngb; i++){
                sparsity_score += static_cast<float>(distances[i]);
            }
            sparsity_scores.push_back(sparsity_score/static_cast<float>(nbr_ngb));
        }
    }
    std::vector<size_t> indices(sparsity_scores.size());
    std::iota(indices.begin(),indices.end(),0);
    std::sort(indices.begin(),indices.end(),[&](size_t i,size_t j){return sparsity_scores[i] > sparsity_scores[j];});
    return solutions[indices[0]];
};

apear::selectors::selector_t<HKInd::Ptr> ea_rg::selectors::pareto_tournament = [](const std::vector<HKInd::Ptr>& solutions,const apear::misc::RandNum::Ptr & rand_num) -> const HKInd::Ptr &{
    if(solutions.size() == 1)
        return solutions.at(0);
    std::vector<float> sparsity_scores;
    std::vector<int> distances;
    for(const HKInd::Ptr &ind1: solutions){
        for(const HKInd::Ptr &ind2: solutions){
            if(ind1->get_morph_genome()->id() == ind2->get_morph_genome()->id())
                continue;
            distances.push_back(robot_design::tree_edit_distance(std::dynamic_pointer_cast<RoboGrammarGenome>(ind1->get_morph_genome())->get_graph(),
                                                                 std::dynamic_pointer_cast<RoboGrammarGenome>(ind2->get_morph_genome())->get_graph()));
            std::sort(distances.begin(),distances.end());
            float sparsity_score = 0;
            size_t nbr_ngb = std::min(static_cast<size_t>(15),distances.size());
            for(size_t i = 0; i < nbr_ngb; i++){
                sparsity_score += static_cast<float>(distances[i]);
            }
            sparsity_scores.push_back(sparsity_score/static_cast<float>(nbr_ngb));
        }
    }
    std::vector<size_t> pareto_front;
    for(size_t i = 0; i < solutions.size(); i++){
        bool dominating = true;
        for(size_t j = 0; j < solutions.size(); j++){
            if(solutions[i]->get_morph_genome()->id() == solutions[j]->get_morph_genome()->id())
                continue;
            if(solutions[i]->get_objectives()[0] < solutions[j]->get_objectives()[0] ||
                sparsity_scores[i] < sparsity_scores[j]){
                dominating = false;
                break;
            }
        }
        if(dominating)
            pareto_front.push_back(i);
    }
    if(pareto_front.empty()){
        int index = rand_num->rand_int(0, static_cast<int>(solutions.size()) - 1);
        return solutions.at(index);
    }
    int index = rand_num->rand_int(0, static_cast<int>(pareto_front.size()) - 1);
    return solutions.at(pareto_front[index]);
};

void ParentsLog::saveLog(const apear::EA<HKInd>::Ptr &ea){
    const std::vector<HKInd::Ptr>& inds = static_cast<MEHKQD*>(ea.get())->get_parent_pool().get_solutions();
    if(inds.empty())
        return;

    std::ifstream ifs;
    std::string l,line;
    if(this->openILogFile(ifs))
        while(std::getline(ifs,l))
            line = l;

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

