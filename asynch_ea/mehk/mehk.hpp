#pragma once

#include "ea_rg/rg_genome.hpp"
#include "ea_rg/rg_simulator.hpp"
#include "apear/logging.hpp"
#include "apear/ame.hpp"

using namespace ea_rg;



//LOGGING CLASSES


class ParentsLog: public apear::Logging<HKInd,RoboGrammarSimulator>{
public:
    ParentsLog(): Logging(){}
    ParentsLog(const std::string &file) : Logging(file){}
    ParentsLog(const ParentsLog& l) : Logging(l){}
    void saveLog(const apear::EA<HKInd>::Ptr&) override;
};


