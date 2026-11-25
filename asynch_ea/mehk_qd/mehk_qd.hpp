#pragma once
#include "apear/algorithms/archives.hpp"
#include "ea_rg/rg_genome.hpp"
#include "ea_rg/rg_simulator.hpp"
#include "apear/logging.hpp"
#include "robot_design/graph.h"


//ARCHIVE SELECTORS
namespace ea_rg {

enum ArchiveType{
    UNSTRUCTURED = 0,
    GRID
};

enum SelectorType{
    RANDOM = 0,
    SPARSITY,
    PARETO
};

void init_grid_archive(apear::EA<HKInd>::Ptr &ame);
void init_unstructured_archive(apear::EA<HKInd>::Ptr &ame);

struct selectors {
    template<class ind_t>
    using deterministic_selector_t = std::function< std::vector<ind_t>(const std::vector<ind_t>&)>;
    static apear::selectors::selector_t<HKInd::Ptr> random;
    static int sparsity_top_nbr;
    static apear::selectors::selector_t<HKInd::Ptr> sparsity;
    static double pareto_goal_tolerance;
    static double pareto_sparsity_tolerance;
    static apear::selectors::selector_t<HKInd::Ptr> pareto_tournament;
    template<class ind_t>
    static inline deterministic_selector_t<ind_t> pareto = [](const std::vector<ind_t>& solutions) -> std::vector<ind_t>{
        std::vector<ind_t> selection;
        if(solutions.size() == 1){
            selection.push_back(solutions.at(0));
            return selection;
        }
        std::vector<size_t> pareto_front;
        std::vector<bool> undominated(solutions.size(),true);
        bool dominating_sparsity = false;
        bool dominating_quality = false;
        for(size_t i = 0; i < solutions.size(); i++){
            for(size_t j = i; j < solutions.size(); j++){
                if(undominated[i] == false)
                    break;
                if(undominated[j] == false)
                    continue;
                const ind_t &ind1 = solutions.at(i);
                const ind_t &ind2 = solutions.at(j);
                if(ind1->get_morph_genome()->id() == ind2->get_morph_genome()->id())
                    continue;
                dominating_quality = false;
                dominating_sparsity = false;
                if(ind1->get_objectives()[0] > ind2->get_objectives()[0])
                    dominating_quality = true;
                if(ind1->get_sparsity_score() > ind2->get_sparsity_score())
                    dominating_sparsity = true;
                if(dominating_quality && dominating_sparsity)
                    undominated[j] = false;
                dominating_quality = false;
                dominating_sparsity = false;
                if(ind2->get_objectives()[0] > ind1->get_objectives()[0])
                    dominating_quality = true;
                if(ind2->get_sparsity_score() > ind1->get_sparsity_score())
                    dominating_sparsity = true;
                if(dominating_quality && dominating_sparsity)
                    undominated[i] = false;
            }
        }
        for(size_t i = 0; i < undominated.size(); i++){
            if(undominated[i])
                pareto_front.push_back(i);
        }
        std::cout << std::endl;
        for(size_t i : pareto_front){
            selection.push_back(solutions.at(i));
        }
        return selection;
    };
};




//LOGGING CLASSES
class ParentsLog: public apear::Logging<HKInd,RoboGrammarSimulator>{
public:
    ParentsLog(): Logging(){}
    ParentsLog(const std::string &file) : Logging(file){}
    ParentsLog(const ParentsLog& l) : Logging(l){}
    void saveLog(const apear::EA<HKInd>::Ptr&) override;
};

class DescriptorsLog: public apear::Logging<HKInd,RoboGrammarSimulator>{
public:
    DescriptorsLog(): Logging(){}
    DescriptorsLog(const std::string &file) : Logging(file){}
    DescriptorsLog(const DescriptorsLog& l) : Logging(l){}
    void saveLog(const apear::EA<HKInd>::Ptr&) override;
};
}//ea_rg
