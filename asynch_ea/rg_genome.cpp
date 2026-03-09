#include "rg_genome.hpp"

using namespace ea_rg;
namespace rd = robot_design;
namespace apear_st = apear::settings;

void RoboGrammarGenome::init(){
    std::string grammar_file = apear_st::getParameter<apear_st::String>(_parameters,"#grammarFile").value;
    for(const rd::Graph &graph : rd::loadGraphs(grammar_file)){
        rd::Rule rule = rd::createRuleFromGraph(graph);
        int type = 0;
        if(rule.lhs_.nodes_.size() < rule.rhs_.nodes_.size())
            type = 0; //growing rule
        else
            type = 1; //building rule
        _grammar[type].push_back(rule);

    }
    _graph = _make_initial_graph();
    _rule_seq.clear();
}

void RoboGrammarGenome::random(){
    int rule_seq_init_size = apear_st::getParameter<apear_st::Integer>(_parameters,"#ruleSeqInitSize").value;
    int rnd_size = _rand_num->rand_int(0,rule_seq_init_size);
    for(int i = 0; i < rnd_size; ++i){
        _grow_graph();
    }
    while(has_nonterminals(_graph)){
        _close_graph();
    }



}

void RoboGrammarGenome::mutate(){
    double mut_rate = apear_st::getParameter<apear_st::Double>(_parameters,"#mutationRate").value;
    double rnd = _rand_num->rand_double(0.0,1.0);
    if(rnd > mut_rate)
        return;
    _rule_seq.erase(std::remove_if(_rule_seq.begin(),_rule_seq.end(),[](std::pair<int,int> r){return r.first == 1;}),_rule_seq.end());
    _graph = _make_graph();
    double prob_grow = apear_st::getParameter<apear_st::Double>(_parameters,"#probGrow").value;
    double prob_prune = apear_st::getParameter<apear_st::Double>(_parameters,"#probPrune").value;
    double rule_seq_max_size = apear_st::getParameter<apear_st::Integer>(_parameters,"#ruleSeqMaxSize").value;
    rnd = _rand_num->rand_double(0.0,1.0);
    if(rnd < prob_prune && _rule_seq.size() > 1)
        _prune_graph();
    rnd = _rand_num->rand_double(0.0,1.0);
    if(rnd < prob_grow && _rule_seq.size() < rule_seq_max_size)
        _grow_graph();
    while(has_nonterminals(_graph)){
        _close_graph();
    }
}

std::string RoboGrammarGenome::to_string() const{
    std::string str = "";
    for(const std::pair<int,int> &rule_idx : _rule_seq)
        str += std::to_string(rule_idx.first) + "," + std::to_string(rule_idx.second) + " ";
    return str;
}

void RoboGrammarGenome::from_string(const std::string &str){
    // _rule_seq.clear();
    // std::istringstream in(str);
    // int rule_idx;
    // while(in >> rule_idx)
    //     _rule_seq.push_back(rule_idx);
    // _graph = _make_graph();
}

bool RoboGrammarGenome::has_nonterminals(const rd::Graph &graph){
    for(const rd::Node &node: graph.nodes_)
        if(node.attrs_.shape_ == rd::LinkShape::NONE)
            return true;
    for(const rd::Edge &edge: graph.edges_)
        if(edge.attrs_.joint_type_ == rd::JointType::NONE)
            return true;
    return false;
}

void RoboGrammarGenome::set_rule_seq(const std::vector<std::pair<int,int>> &rule_seq){
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

void RoboGrammarGenome::_add_rule(int rule_type){
    std::vector<std::pair<size_t,std::vector<rd::GraphMapping>>> appl_rules;
    //find applicable rules
    std::vector<rd::Rule> &rules = _grammar[rule_type];
    for(size_t i = 0; i < rules.size(); ++i){
        std::vector<rd::GraphMapping> matches;
        for(rd::GraphMapping &match: rd::findMatches(rules[i].lhs_,_graph)){
            if(rd::checkRuleApplicability(rules[i],_graph,match))
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
    _graph = rd::applyRule(rules[appl_rules[rnd_rule].first],_graph,appl_rules[rnd_rule].second[0]);
    _rule_seq.push_back(std::make_pair(rule_type,appl_rules[rnd_rule].first));
}

void RoboGrammarGenome::_grow_graph(){
    _add_rule(0);
}

void RoboGrammarGenome::_prune_graph(){
    _rule_seq.erase(_rule_seq.end()-1);
    _graph = _make_graph();
}

void RoboGrammarGenome::_close_graph(){
    _add_rule(1);
}


rd::Graph RoboGrammarGenome::_make_graph(){
    rd::Graph graph = _make_initial_graph();
    for(const std::pair<int,int> &rule_idx : _rule_seq){
        const int &type = rule_idx.first;
        const int &idx = rule_idx.second;
        std::vector<rd::GraphMapping> matches;
        for(rd::GraphMapping &match: rd::findMatches(_grammar[type][idx].lhs_,graph)){
            if(rd::checkRuleApplicability(_grammar[type][idx],graph,match))
                matches.push_back(match);
        }
        if(matches.empty())
            throw std::runtime_error("The rule sequence is not consistent with the graph");
        graph = rd::applyRule(_grammar[type][idx],graph,matches[0]);
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
