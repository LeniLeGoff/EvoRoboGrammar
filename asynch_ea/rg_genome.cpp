#include "rg_genome.hpp"

using namespace ea_rg;
namespace rd = robot_design;

void RoboGrammarGenome::init(){
    //TODO: Load grammar
}

void RoboGrammarGenome::mutate(){


}

void RoboGrammarGenome::_add_rule(){
    rd::Rule &last_rule = _rules.back();
    std::vector<rd::GraphMapping> matches = rd::findMatches(last_rule.lhs_,_grammar);
    std::vector<rd::GraphMapping> cand_rules;
    for(rd::GraphMapping &match: matches){
        if(rd::checkRuleApplicability(last_rule,_grammar,match))
            cand_rules.push_back(match);
    }
    if(cand_rules.empty())
        return;

    int rnd_idx = _rand_num->rand_int(0,cand_rules.size()-1);
    _rules.push_back(cand_rules[rnd_idx]);
}

void RoboGrammarGenome::_remove_rule(){

}
