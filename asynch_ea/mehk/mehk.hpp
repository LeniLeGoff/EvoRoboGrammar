#pragma once

#include "ea_rg/rg_genome.hpp"
#include "ea_rg/rg_simulator.hpp"
#include "apear/logging.hpp"

using namespace ea_rg;

class MEHKInd : public RoboGrammarInd{
public:
    using Ptr = std::shared_ptr<MEHKInd>;
    using ConsPtr = std::shared_ptr<const MEHKInd>;
    MEHKInd(): RoboGrammarInd(){}
    MEHKInd(const apear::misc::RandNum::Ptr& rn, const apear::settings::ParametersMapPtr &param) :
        RoboGrammarInd(rn,param){
        _morph_genome = std::make_shared<RoboGrammarGenome>(rn,param);
        _ctrl_genome = std::make_shared<apear::EmptyGenome>();
    }
    MEHKInd(const RoboGrammarGenome::Ptr &morph_gen,const apear::EmptyGenome::Ptr &ctrl_gen) :
       RoboGrammarInd(morph_gen,ctrl_gen){}
    MEHKInd(const MEHKInd &ind) : RoboGrammarInd(ind){}
    apear::Individual::Ptr clone() override{
        return std::make_shared<MEHKInd>(*this);
    }
    void init() override;
private:
    void _create_controller() override;
};

class RGGenomeLog: public apear::Logging<MEHKInd,RoboGrammarSimulator>{
public:
    RGGenomeLog() : Logging(){}
    RGGenomeLog(const std::string &file) : Logging(file){}
    RGGenomeLog(const RGGenomeLog& l) : Logging<MEHKInd,RoboGrammarSimulator>(l){}
    void register_data(const IndPtr &ind,const RoboGrammarSimulator &sim) override;
    void saveLog() override;
private:
    std::vector<std::string> _data;
};
