#include <fstream>
#include <string>
#include <memory>

#include "mehk_qd.hpp"
#include "apear/async_dealer.hpp"
#include "apear/ame_qd.hpp"
#include "apear/misc/utilities.hpp"
#include "ea_rg/tasks.hpp"
#include "ea_rg/loggings.hpp"
#include "robot_design/graph.h"

using namespace ea_rg;
namespace apear_st = apear::settings;
using Archive = apear::UnstructuredArchive<HKInd::Ptr,int>;
using MEHKQD = apear::AsyncMorphoEvolutionQD<HKInd,RoboGrammarGenome,Archive>;

double selectors::pareto_goal_tolerance = 0;
double selectors::pareto_sparsity_tolerance = 0;
int selectors::sparsity_top_nbr = 1;

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

    //initialise an unstructured archive
    Archive::comparator_t comparator([](const HKInd::Ptr& a, const HKInd::Ptr& b) -> bool{//comparator for quality
        return a->get_objectives()[0] > b->get_objectives()[0];
    });
    Archive::distance_fct_t distance_fct([](const HKInd::Ptr& a, const HKInd::Ptr& b) -> int{//distance for diversity
        return robot_design::tree_edit_distance(std::dynamic_pointer_cast<RoboGrammarGenome>(a->get_morph_genome())->get_graph(),
                                                std::dynamic_pointer_cast<RoboGrammarGenome>(b->get_morph_genome())->get_graph());
    });
    int selector_type = apear_st::getParameter<apear_st::Integer>(param,"#archiveSelectorType").value;
    int nbr_ngb = apear_st::getParameter<apear_st::Integer>(param,"#sparsityNbrNeighbors").value;
    if(selector_type == ea_rg::RANDOM)
        dynamic_cast<MEHKQD*>(ame.get())->init_archive(1,distance_fct,comparator,ea_rg::selectors::random);
    else if(selector_type == ea_rg::SPARSITY){
        selectors::sparsity_top_nbr = apear_st::getParameter<apear_st::Integer>(param,"#sparsityTopNumber").value;
        dynamic_cast<MEHKQD*>(ame.get())->init_archive(1,distance_fct,comparator,ea_rg::selectors::sparsity,nbr_ngb);
    }
    else if(selector_type == ea_rg::PARETO){
        selectors::pareto_goal_tolerance = apear_st::getParameter<apear_st::Double>(param,"#paretoGoalTolerance").value;
        selectors::pareto_sparsity_tolerance = apear_st::getParameter<apear_st::Double>(param,"#paretoSparsityTolerance").value;
        dynamic_cast<MEHKQD*>(ame.get())->init_archive(1,distance_fct,comparator,ea_rg::selectors::pareto_tournament,nbr_ngb);
    }

    //initialise the asynchronous dealer
    dealer.set_ea(ame);
    dealer.init(nbr_sim,headless);

    //define the environment and task
    std::vector<double> arena_size = apear_st::getParameter<apear_st::Sequence<double>>(param,"#arenaSize").value;

    dealer.set_environment<ea_rg::FlatArena>(arena_size[0],arena_size[1],ea_rg::fitness::MovementExploStability(param));

    apear_st::saveParameters(apear::logging::log_folder + "/parameters.csv",param);

    //run the experiment
    // do{
    //     dealer.update_simulators();
    // }while(dealer.update_ea());
    dealer.run_threaded_updates();

    return 0;

}
apear::selectors::selector_t<HKInd::Ptr> ea_rg::selectors::random = apear::selectors::random<HKInd::Ptr>;
apear::selectors::selector_t<HKInd::Ptr> ea_rg::selectors::sparsity = [](const std::vector<HKInd::Ptr>& solutions,const apear::misc::RandNum::Ptr & rand_num, std::vector<HKInd::Ptr>& selection){
    std::cout << "SPARSITY SELECTOR" << std::endl;
    if(solutions.size() == 1){
        for(size_t i = 0; i < selection.size(); i++)
            selection[i] = solutions.at(0);
        return;
    }

    std::vector<double> sparsity_scores;
    std::vector<int> distances;
    for(const HKInd::Ptr &ind1: solutions){
        sparsity_scores.push_back(ind1->get_sparsity_score());
    }
    std::vector<size_t> indices(sparsity_scores.size());
    std::iota(indices.begin(),indices.end(),0);
    std::sort(indices.begin(),indices.end(),[&](size_t i,size_t j){return sparsity_scores[i] > sparsity_scores[j];});
    for(size_t i = 0; i < selection.size(); i++){
        if(selectors::sparsity_top_nbr == 0){
            selection[i] = solutions[indices[0]];
            continue;
        }
        int index = rand_num->rand_int(0, std::min(selectors::sparsity_top_nbr-1,static_cast<int>(indices.size()-1)));
        selection[i] = solutions[indices[index]];
    }
};

apear::selectors::selector_t<HKInd::Ptr> ea_rg::selectors::pareto_tournament = [](const std::vector<HKInd::Ptr>& solutions,const apear::misc::RandNum::Ptr & rand_num, std::vector<HKInd::Ptr>& selection){
    std::cout << "PARETO TOURNAMENT SELECTOR" << std::endl;
    if(solutions.size() == 1){
        for(size_t s = 0; s < selection.size(); s++)

            selection[s] = solutions.at(0);
        return;
    }
    std::cout << "Computing Pareto front ..." << std::endl;
    std::vector<size_t> pareto_front;
    std::vector<bool> undominated(solutions.size(),true);
    bool dominating_sparsity = false;
    bool dominating_quality = false;
    double goal_average = 0;
    double sparsity_average = 0;
    for(const HKInd::Ptr &ind: solutions){
        goal_average += ind->get_objectives()[0];
        sparsity_average += static_cast<double>(ind->get_sparsity_score());
    }
    goal_average = goal_average/static_cast<double>(solutions.size());
    sparsity_average = sparsity_average/static_cast<double>(solutions.size());
    double goal_stddev = 0;
    double sparsity_stddev = 0;
    for(const HKInd::Ptr &ind: solutions){
        goal_stddev += (ind->get_objectives()[0] - goal_average)*(ind->get_objectives()[0] - goal_average);
        sparsity_stddev += (static_cast<double>(ind->get_sparsity_score()) - sparsity_average)*(static_cast<double>(ind->get_sparsity_score()) - sparsity_average);
    }
    goal_stddev = std::sqrt(goal_stddev/static_cast<double>(solutions.size()));
    sparsity_stddev = std::sqrt(sparsity_stddev/static_cast<double>(solutions.size()));
    std::cout << "goal stddev: " << goal_stddev << " sparsity stddev " << sparsity_stddev << std::endl;
    for(size_t i = 0; i < solutions.size(); i++){
        for(size_t j = i; j < solutions.size(); j++){
            if(undominated[i] == false)
                break;
            if(undominated[j] == false)
                continue;
            const HKInd::Ptr &ind1 = solutions.at(i);
            const HKInd::Ptr &ind2 = solutions.at(j);
            if(ind1->get_morph_genome()->id() == ind2->get_morph_genome()->id())
                continue;
            std::cout << "ind1: " << ind1->get_morph_genome()->id() << " fitness: " << ind1->get_objectives()[0] << " sparsity: " << ind1->get_sparsity_score() << " vs "
                      << "ind2: " << ind2->get_morph_genome()->id() << " fitness: " << ind2->get_objectives()[0] << " sparsity: " << ind2->get_sparsity_score() << std::endl;
            dominating_quality = false;
            dominating_sparsity = false;
            if(ind1->get_objectives()[0] > ind2->get_objectives()[0] + goal_stddev)
                dominating_quality = true;
            if(ind1->get_sparsity_score() > ind2->get_sparsity_score() + sparsity_stddev)
                dominating_sparsity = true;
            if(dominating_quality && dominating_sparsity)
                undominated[j] = false;
            dominating_quality = false;
            dominating_sparsity = false;
            if(ind2->get_objectives()[0] > ind1->get_objectives()[0] + goal_stddev)
                dominating_quality = true;
            if(ind2->get_sparsity_score() > ind1->get_sparsity_score() + sparsity_stddev)
                dominating_sparsity = true;
            if(dominating_quality && dominating_sparsity)
                undominated[i] = false;
        }
    }
    for(size_t i = 0; i < undominated.size(); i++){
        if(undominated[i])
            pareto_front.push_back(i);
    }
    for(size_t s = 0; s < selection.size(); s++){
        if(pareto_front.empty()){
            std::cout << "Pareto front is empty, selecting random solution ..." << std::endl;
            int index = rand_num->rand_int(0, static_cast<int>(solutions.size()) - 1);
            selection[s] = solutions.at(index);
            continue;
        }
        std::cout << "Pareto front size: " << pareto_front.size() << std::endl;
        int index = rand_num->rand_int(0, static_cast<int>(pareto_front.size()) - 1);
        selection[s] = solutions.at(pareto_front[index]);
    }
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
    sstr << inds.front()->get_morph_genome()->id() << "," << inds.front()->get_objectives()[0] << "," << inds.front()->get_sparsity_score();
    for(size_t i = 1; i < inds.size(); i++)
        sstr << ";" << inds.at(i)->get_morph_genome()->id() << "," << inds.at(i)->get_objectives()[0] << "," << inds.at(i)->get_sparsity_score();
    if(!line.empty() && sstr.str() == line) //genome pool didn't change
        return;
    std::ofstream ofs;
    if(!openOLogFile(ofs))
        return;
    ofs << sstr.str() << std::endl;
    ofs.close();
}
