#pragma once

#include "apear/genome.hpp"
#include "apear/individual.hpp"
#include "robot_design/graph.h"
#include "robot_design/sim.h"

namespace ea_rg{

namespace rd = robot_design;

class RoboGrammarGenome: public apear::Genome{
public:
    typedef std::shared_ptr<RoboGrammarGenome> Ptr;
    typedef std::shared_ptr<const RoboGrammarGenome> ConstPtr;
    using rule_idx_t = std::pair<size_t,size_t>; //first: rule type, second: rule idx in the type

    RoboGrammarGenome() : apear::Genome(){
                _id = _highest_id++;
    }
    RoboGrammarGenome(const apear::misc::RandNum::Ptr& rn, const apear::settings::ParametersMapPtr &param)
        : apear::Genome(rn,param){
        _id = _highest_id++;
    }

    RoboGrammarGenome(const RoboGrammarGenome &genome)
        : apear::Genome(genome),
        _grammar(genome._grammar),
        _rule_seq(genome._rule_seq),
        _graph(genome._graph){}

    Genome::Ptr clone() const override{
        return std::make_shared<RoboGrammarGenome>(*this);
    }
    void init() override;
    void mutate() override;
    void crossover(const Genome::Ptr& partner,Genome::Ptr child1) override{}//TODO
    void symmetrical_crossover(const Genome::Ptr& partner,Genome::Ptr child1,Genome::Ptr child2) override{}//TODO
    void random() override;

    bool has_nonterminals(const rd::Graph &graph);

    void set_rule_seq(const std::vector<rule_idx_t> &rule_seq);
    const std::vector<rule_idx_t> &get_rule_seq() const{return _rule_seq;}

    const rd::Graph& get_graph() const{return _graph;}

    std::string to_string() const override;
    void from_string(const std::string&) override;

private:

    static int _highest_id;
    robot_design::Graph _make_initial_graph();

    //mutation operators
    void _add_rule(int rule_type);
    /**
     * @brief expand graph with a random applicable growing rule
     */
    void _grow_graph();
    /**
     * @brief apply a random applicable closing rules
     */
    void _close_graph();

    void _swap_rule();

    /**
     * @brief prune randomly an end link of the graph
     */
    void _prune_graph();

    std::vector<rule_idx_t> _find_swappable_rules(const rule_idx_t &rule_idx);

    rd::Graph _make_graph(const std::vector<rule_idx_t>& rule_seq);

    bool _check_rule_applicability(const rd::Rule &rule,const rd::Graph &graph);

    std::array<std::vector<rd::Rule>,2> _grammar; //set of existing rules, 0: growing rules, 1: closing rules
    std::vector<rule_idx_t> _rule_seq; //the sequence of applied rules, use for logging and rebuild the graph, first: rule type, second, rule idx
    rd::Graph _graph; //graph representation of the robot design, use to build the robot
};

class RoboGrammarInd : public apear::Individual{
public:
    RoboGrammarInd() : apear::Individual(){
    }
    RoboGrammarInd(const apear::misc::RandNum::Ptr& rn, const apear::settings::ParametersMapPtr &param) :
        apear::Individual(rn,param){
        _morph_genome = std::make_shared<RoboGrammarGenome>(rn,param);
        _ctrl_genome = std::make_shared<apear::EmptyGenome>();
    }
    RoboGrammarInd(const RoboGrammarGenome::Ptr &morph_gen,const apear::EmptyGenome::Ptr &ctrl_gen) :
        apear::Individual(morph_gen,ctrl_gen){}
    RoboGrammarInd(const RoboGrammarInd &ind) : apear::Individual(ind), _robot(ind._robot){}

    Individual::Ptr clone() override{
        return std::make_shared<RoboGrammarInd>(*this);
    }

    void init() override;


    const rd::Robot &get_robot() const{return _robot;}
    void set_simulator(const std::shared_ptr<rd::BulletSimulation> &sim){_sim = sim;}
private:
    void _create_morphology() override;
    void _create_controller() override;
    rd::Robot _robot;
    std::shared_ptr<rd::BulletSimulation> _sim = nullptr;
};

}//ea_rg
