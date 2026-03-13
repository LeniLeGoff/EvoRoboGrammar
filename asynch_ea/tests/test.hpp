#pragma once
#include "apear/individual.hpp"
#include "ea_rg/rg_genome.hpp"

using namespace ea_rg;

class RandomInd : public RoboGrammarInd{
public:
    RandomInd() : RoboGrammarInd(){
    }
    RandomInd(const apear::misc::RandNum::Ptr& rn, const apear::settings::ParametersMapPtr &param) :
        RoboGrammarInd(rn,param){
        _morph_genome = std::make_shared<RoboGrammarGenome>(rn,param);
        _ctrl_genome = std::make_shared<apear::EmptyGenome>();
    }
    RandomInd(const RoboGrammarGenome::Ptr &morph_gen,const apear::EmptyGenome::Ptr &ctrl_gen) :
       RoboGrammarInd(morph_gen,ctrl_gen){}
    RandomInd(const RandomInd &ind) : RoboGrammarInd(ind){}

    Individual::Ptr clone() override{
        return std::make_shared<RandomInd>(*this);
    }

    void init() override;

private:
    void _create_controller() override;
};
