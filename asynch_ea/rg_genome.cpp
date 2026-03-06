#include "rg_genome.hpp"

using namespace ea_rg;
namespace rd = robot_design;
namespace apear_st = apear::settings;

void RoboGrammarGenome::init(){
    std::string grammar_file = apear_st::getParameter<apear_st::String>(_parameters,"#grammarFile").value;
    for(const rd::Graph &graph : rd::loadGraphs(grammar_file))
        _grammar.push_back(rd::createRuleFromGraph(graph));
    _graph = _make_initial_graph();
    _rule_seq.clear();
}

void RoboGrammarGenome::random(){
    int rule_seq_init_size = apear_st::getParameter<apear_st::Integer>(_parameters,"#ruleSeqInitSize").value;
    int rule_seq_size = _rand_num->rand_int(1,rule_seq_init_size);
    for(int i = 0; i < rule_seq_size; ++i)
        _grow_graph();
}

void RoboGrammarGenome::mutate(){
    double mut_rate = apear_st::getParameter<apear_st::Double>(_parameters,"#mutationRate").value;
    double rnd = _rand_num->rand_double(0.0,1.0);
    if(rnd > mut_rate)
        return;
    double prob_grow = apear_st::getParameter<apear_st::Double>(_parameters,"#probGrow").value;
    double prob_prune = apear_st::getParameter<apear_st::Double>(_parameters,"#probPrune").value;
    double rule_seq_max_size = apear_st::getParameter<apear_st::Integer>(_parameters,"#ruleSeqMaxSize").value;
    rnd = _rand_num->rand_double(0.0,1.0);
    if(rnd < prob_prune && _rule_seq.size() > 1)
        _prune_graph();
    rnd = _rand_num->rand_double(0.0,1.0);
    if(rnd < prob_grow && _rule_seq.size() < rule_seq_max_size)
        _grow_graph();
}

std::string RoboGrammarGenome::to_string() const{
    std::string str = "";
    for(int rule_idx : _rule_seq)
        str += std::to_string(rule_idx) + " ";
    return str;
}

void RoboGrammarGenome::from_string(const std::string &str){
    _rule_seq.clear();
    std::istringstream in(str);
    int rule_idx;
    while(in >> rule_idx)
        _rule_seq.push_back(rule_idx);
    _graph = _make_graph();
}

void RoboGrammarGenome::set_rule_seq(const std::vector<int> &rule_seq){
    _rule_seq = rule_seq;
    _graph = _make_graph();
}

rd::Graph RoboGrammarGenome::_make_initial_graph(){
    rd::Node n;
    n.name_ = "robot";
    n.attrs_.label_ = "robot";
    rd::Graph graph;
    graph.nodes_.push_back(n);
    return graph;
}

void RoboGrammarGenome::_grow_graph(){
    std::vector<std::pair<size_t,std::vector<rd::GraphMapping>>> appl_rules;
    //find applicable rules
    for(size_t i = 0; i < _grammar.size(); ++i){
        std::vector<rd::GraphMapping> matches;
        for(rd::GraphMapping &match: rd::findMatches(_grammar[i].lhs_,_graph)){
            if(rd::checkRuleApplicability(_grammar[i],_graph,match))
                matches.push_back(match);
        }
        if(matches.empty())
            continue;
        appl_rules.push_back(std::make_pair(i,matches)); //store the applicable rule index and its matches
    }
    if(appl_rules.empty())
        return;

    int rnd_rule = _rand_num->rand_int(0,appl_rules.size()-1);
    // int rnd_match = _rand_num->rand_int(0,appl_rules[rnd_rule].second.size()-1);
    //expand graph and save the index in the rule sequence
    _graph = rd::applyRule(_grammar[appl_rules[rnd_rule].first],_graph,appl_rules[rnd_rule].second[0]);
    _rule_seq.push_back(appl_rules[rnd_rule].first);
}

void RoboGrammarGenome::_prune_graph(){
    _rule_seq.erase(_rule_seq.end()-1);
    _graph = _make_graph();
}

rd::Graph RoboGrammarGenome::_make_graph(){
    rd::Graph graph = _make_initial_graph();
    for(const int &rule_idx : _rule_seq){
        std::vector<rd::GraphMapping> matches;
        for(rd::GraphMapping &match: rd::findMatches(_grammar[rule_idx].lhs_,graph)){
            if(rd::checkRuleApplicability(_grammar[rule_idx],graph,match))
                matches.push_back(match);
        }
        if(matches.empty())
            throw std::runtime_error("The rule sequence is not consistent with the graph");
        graph = rd::applyRule(_grammar[rule_idx],graph,matches[0]);
    }
    return graph;
}

void RoboGrammarInd::init(){
    _morph_genome->init();
    _morph_genome->random();
}

void RoboGrammarInd::_create_morphology(){
    _robot = rd::buildRobot(std::static_pointer_cast<RoboGrammarGenome>(_morph_genome)->get_graph());

}

void RoboGrammarInd::_create_controller(){
    //TODO create a controller based on the morphology
}
