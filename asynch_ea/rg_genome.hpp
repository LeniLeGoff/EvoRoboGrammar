#pragma once

#include "apear/genome.hpp"
#include "robot_design/graph.h"

namespace ea_rg{

namespace rd = robot_design;

class RoboGrammarGenome: public apear::Genome{
public:
    RoboGrammarGenome() : apear::Genome(){}
    RoboGrammarGenome(const apear::misc::RandNum::Ptr& rn, const apear::settings::ParametersMapPtr &param)
        : apear::Genome(rn,param){}

    Genome::Ptr clone() const override;
    void init() override;
    void mutate() override;
    void crossover(const Genome::Ptr& partner,Genome::Ptr child1) override;
    void symmetrical_crossover(const Genome::Ptr& partner,Genome::Ptr child1,Genome::Ptr child2) override;
    void random() override;

    std::string to_string() const override;
    void from_string(const std::string&) override;

private:

    void _add_rule();
    void _remove_rule();

    std::vector<rd::Rule> _rules;
    rd::Graph _grammar;
};

}//ea_rg
