#pragma once

#include "apear/genome.hpp"
#include "apear/individual.hpp"
#include "robot_design/graph.h"

namespace ea_rg{

namespace rd = robot_design;

class RoboGrammarGenome: public apear::Genome{
public:
    typedef std::shared_ptr<RoboGrammarGenome> Ptr;
    typedef std::shared_ptr<const RoboGrammarGenome> ConstPtr;

    RoboGrammarGenome() : apear::Genome(){}
    RoboGrammarGenome(const apear::misc::RandNum::Ptr& rn, const apear::settings::ParametersMapPtr &param)
        : apear::Genome(rn,param){}

    Genome::Ptr clone() const override{
        return std::make_shared<RoboGrammarGenome>(*this);
    }
    void init() override;
    void mutate() override;
    void crossover(const Genome::Ptr& partner,Genome::Ptr child1) override{}//TODO
    void symmetrical_crossover(const Genome::Ptr& partner,Genome::Ptr child1,Genome::Ptr child2) override{}//TODO
    void random() override;

    void set_rule_seq(const std::vector<int> &rule_seq);
    const std::vector<int> &get_rule_seq() const{return _rule_seq;}

    const rd::Graph& get_graph() const{return _graph;}

    std::string to_string() const override;
    void from_string(const std::string&) override;

private:

    robot_design::Graph _make_initial_graph();

    //mutation operators
    /**
     * @brief expand graph with a random applicable rule
     */
    void _grow_graph();
    /**
     * @brief prune randomly an end link of the graph
     */
    void _prune_graph();

    rd::Graph _make_graph();

    std::vector<rd::Rule> _grammar; //list of possible building rules
    std::vector<int> _rule_seq; //the sequence of applied rules, use for logging and rebuild the graph
    rd::Graph _graph; //graph representation of the robot design, use to build the robot
};

class RoboGrammarInd : public apear::Individual{
public:
    RoboGrammarInd() : apear::Individual(){
        std::cout << "constructing" << std::endl;
    }
    RoboGrammarInd(const apear::misc::RandNum::Ptr& rn, const apear::settings::ParametersMapPtr &param) :
        apear::Individual(rn,param){
        _morph_genome = std::make_shared<RoboGrammarGenome>(rn,param);
    }
    RoboGrammarInd(const RoboGrammarGenome::Ptr &morph_gen,const apear::Genome::Ptr &ctrl_gen) :
        apear::Individual(morph_gen,ctrl_gen){}

    Individual::Ptr clone() override{
        return std::make_shared<RoboGrammarInd>(*this);
    }

    void init() override;


    const rd::Robot &get_robot() const{return _robot;}

private:
    void _create_morphology() override;
    void _create_controller() override;
    rd::Robot _robot;
};

}//ea_rg
