#include "rg_genome.hpp"
#include "rg_controllers.hpp"

using namespace ea_rg;
namespace rd = robot_design;
namespace apear_st = apear::settings;

int RoboGrammarGenome::_highest_id = 0;

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
    _rule_seq.clear();
    _rule_seq.push_back(std::make_pair(0,0)); //start with the first growing rule
    _graph = _make_graph(_rule_seq);
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

    double prob_grow = apear_st::getParameter<apear_st::Double>(_parameters,"#probGrow").value;
    double prob_prune = apear_st::getParameter<apear_st::Double>(_parameters,"#probPrune").value;
    double prob_swap = apear_st::getParameter<apear_st::Double>(_parameters,"#probSwap").value;
    double rule_seq_max_size = apear_st::getParameter<apear_st::Integer>(_parameters,"#ruleSeqMaxSize").value;
    rnd = _rand_num->rand_double(0.0,1.0);
    if(rnd < prob_swap)
        _swap_rule();
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
    for(const rule_idx_t &rule_idx : _rule_seq)
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

void RoboGrammarGenome::set_rule_seq(const std::vector<rule_idx_t> &rule_seq){
    _rule_seq = rule_seq;
    _graph = _make_graph(_rule_seq);
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
    _graph = rd::applyRule(rules[appl_rules[rnd_rule].first],_graph,appl_rules[rnd_rule].second[0]);
    _rule_seq.push_back(std::make_pair(rule_type,appl_rules[rnd_rule].first));
}

void RoboGrammarGenome::_swap_rule(){
    if(_rule_seq.size() <= 1)
        return;
    std::vector<rule_idx_t> swapable_rules;
    int rnd_idx;
    do{
        rnd_idx = _rand_num->rand_int(0,_rule_seq.size()-1);
        swapable_rules = _find_swappable_rules(_rule_seq[rnd_idx]);
    }while(swapable_rules.empty());
    int swap_rnd_idx = _rand_num->rand_int(0,swapable_rules.size()-1);
    _rule_seq[rnd_idx] = swapable_rules[swap_rnd_idx];
    _graph = _make_graph(_rule_seq);
}

std::vector<RoboGrammarGenome::rule_idx_t> RoboGrammarGenome::_find_swappable_rules(const rule_idx_t &rule_idx){
    std::vector<rule_idx_t> swappable_rules;
    rd::Graph rule_lhs = _grammar[rule_idx.first][rule_idx.second].lhs_;
    for(size_t idx = 0; idx < _grammar[rule_idx.first].size(); idx++){
        if(idx == rule_idx.second)
            continue;
        rd::Graph lhs = _grammar[rule_idx.first][idx].lhs_;
        if(lhs.nodes_.size() == rule_lhs.nodes_.size()){
            bool swappable = true;
            if(lhs.nodes_.size() > 1){
                for(size_t i = 0; i < lhs.edges_.size(); i++){
                    swappable = swappable && (lhs.edges_[i].attrs_.require_label_ == rule_lhs.edges_[i].attrs_.require_label_);
                    if(!swappable)
                        break;
                }
            }
            if(!swappable)
                continue;
            for(size_t i = 0; i < lhs.nodes_.size(); i++){
                swappable = swappable && (lhs.nodes_[i].name_ == rule_lhs.nodes_[i].name_);
                if(!swappable)
                    break;
            }
            if(swappable)
                swappable_rules.push_back(std::make_pair(rule_idx.first,idx));
        }
    }
    return swappable_rules;
}

void RoboGrammarGenome::_grow_graph(){
    if(_rule_seq.empty()){
        _add_rule(0);
        return;
    }
    //remove closing rules
    std::vector<rule_idx_t> closing_rule_seq = _rule_seq;
    closing_rule_seq.erase(std::remove_if(closing_rule_seq.begin(),closing_rule_seq.end(),[](rule_idx_t r){return r.first == 0;}),closing_rule_seq.end());
    _rule_seq.erase(std::remove_if(_rule_seq.begin(),_rule_seq.end(),[](rule_idx_t r){return r.first == 1;}),_rule_seq.end());
    //remake the graph without closing rules
    _graph = _make_graph(_rule_seq);
    //add a growing rule
    _add_rule(0);

    if(!closing_rule_seq.empty()){// if any reaply matching closing rules
        for(const rule_idx_t &rule_idx : closing_rule_seq){
            std::vector<rd::GraphMapping> matches;
            for(rd::GraphMapping &match: rd::findMatches(_grammar[1][rule_idx.second].lhs_,_graph)){
                if(rd::checkRuleApplicability(_grammar[1][rule_idx.second],_graph,match))
                    matches.push_back(match);
            }
            if(matches.empty())
                continue;
            _graph = rd::applyRule(_grammar[1][rule_idx.second],_graph,matches[0]);
            _rule_seq.push_back(rule_idx);
        }
    }
}

void RoboGrammarGenome::_prune_graph(){
    if(_rule_seq.size() <= 1){
        return;
    }
    //remove closing rules
    std::vector<rule_idx_t> closing_rule_seq = _rule_seq;
    closing_rule_seq.erase(std::remove_if(closing_rule_seq.begin(),closing_rule_seq.end(),[](rule_idx_t r){return r.first == 0;}),closing_rule_seq.end());
    _rule_seq.erase(std::remove_if(_rule_seq.begin(),_rule_seq.end(),[](rule_idx_t r){return r.first == 1;}),_rule_seq.end());
    //remake the graph without closing rules
    _graph = _make_graph(_rule_seq);
    //remove last growing rule to remove (except first rule)
    _rule_seq.erase(_rule_seq.begin() + _rule_seq.size()-1);
    if(!closing_rule_seq.empty()){// if any reaply matching closing rules
        for(const rule_idx_t &rule_idx : closing_rule_seq){
            std::vector<rd::GraphMapping> matches;
            for(rd::GraphMapping &match: rd::findMatches(_grammar[1][rule_idx.second].lhs_,_graph)){
                if(rd::checkRuleApplicability(_grammar[1][rule_idx.second],_graph,match))
                    matches.push_back(match);
            }
            if(matches.empty())
                continue;
            _graph = rd::applyRule(_grammar[1][rule_idx.second],_graph,matches[0]);
            _rule_seq.push_back(rule_idx);
        }
    }
}

void RoboGrammarGenome::_close_graph(){
    _add_rule(1);
}


bool RoboGrammarGenome::_check_rule_applicability(const rd::Rule &rule,const rd::Graph &graph){
    for(rd::GraphMapping &match: rd::findMatches(rule.lhs_,graph)){
        if(rd::checkRuleApplicability(rule,graph,match))
            return true;
    }
    return false;
}

rd::Graph RoboGrammarGenome::_make_graph(const std::vector<rule_idx_t> &rule_seq){
    rd::Graph graph = _make_initial_graph();
    for(const rule_idx_t &rule_idx : rule_seq){
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
    _control = std::make_shared<RandomControl>(_rand_num,_parameters);
    int dof = 0;
    for(const rd::Link &link: _robot.links_){
        if(link.joint_type_ != rd::JointType::FIXED)
            dof++;
    }
    std::dynamic_pointer_cast<RandomControl>(_control)->init(dof);
}
