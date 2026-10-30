#pragma once

#include "ea_rg/rg_genome.hpp"
#include "ea_rg/rg_simulator.hpp"
#include "apear/logging.hpp"
#include "apear/algorithms/selectors.hpp"
#include "robot_design/graph.h"



//ARCHIVE SELECTORS
namespace ea_rg {

enum SelectorType{
    RANDOM = 0,
    SPARSITY,
    PARETO
};

struct selectors {
    static apear::selectors::selector_t<HKInd::Ptr> random;
    static apear::selectors::selector_t<HKInd::Ptr> sparsity;
    static apear::selectors::selector_t<HKInd::Ptr> pareto_tournament;
};



//LOGGING CLASSES
class ParentsLog: public apear::Logging<HKInd,RoboGrammarSimulator>{
public:
    ParentsLog(): Logging(){}
    ParentsLog(const std::string &file) : Logging(file){}
    ParentsLog(const ParentsLog& l) : Logging(l){}
    void saveLog(const apear::EA<HKInd>::Ptr&) override;
};
}
