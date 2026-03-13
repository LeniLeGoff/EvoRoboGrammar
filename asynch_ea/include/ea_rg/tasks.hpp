#pragma once

#include <functional>
#include "apear/environment.hpp"
#include "ea_rg/rg_simulator.hpp"

namespace ea_rg{

namespace rd = robot_design;

namespace fitness{

struct Function{
    using Ptr = std::shared_ptr<Function>;
    using ConstPtr = std::shared_ptr<const Function>;
    virtual std::vector<double> operator()(RoboGrammarSimulator&) = 0;
    virtual bool update(RoboGrammarSimulator&){return true;}
};

struct Dummy: public Function{
    using Ptr = std::shared_ptr<Dummy>;
    using ConstPtr = std::shared_ptr<const Dummy>;

    std::vector<double> operator()(RoboGrammarSimulator &) override{
        return {0};
    }
};

struct Exploration: public Function{
    using Ptr = std::shared_ptr<Exploration>;
    using ConstPtr = std::shared_ptr<const Exploration>;

    Exploration(const apear::settings::ParametersMapPtr& param);

    std::vector<double> operator()(RoboGrammarSimulator &) override;
    bool update(RoboGrammarSimulator&) override;
    std::pair<int,int> real_to_matrix_coord(const rd::Vector3&);
    std::vector<int> grid_size = {8,8};
    Eigen::MatrixXi grid_zones;
    double cell_size;
    bool verbose = false;
};
}//fitness

using obj_fcts_t = std::function<std::vector<double>()>;

struct obj_fcts{
    static obj_fcts_t exploration;
};

class FlatTerrain : public apear::Environment<RoboGrammarSimulator>{
public:
    using Ptr = std::shared_ptr<FlatTerrain>;
    using ConstPtr = std::shared_ptr<const FlatTerrain>;

    FlatTerrain(){}
    void init(Sim &sim) override;
    std::vector<double> fitness_function(Sim &sim) override;
    bool update(double time,Sim &sim) override;
    void set_fitness_fct(const fitness::Function::Ptr& fct){_fitness_fct = fct;}
private:
    fitness::Function::Ptr _fitness_fct = nullptr;
};

class FlatArena: public apear::Environment<RoboGrammarSimulator>{
public:
    using Ptr = std::shared_ptr<FlatArena>;
    using ConstPtr = std::shared_ptr<const FlatArena>;

    FlatArena(double width, double length)
        : _width(width),
        _length(length)
    {}
    void init(Sim &sim) override;
    std::vector<double> fitness_function(Sim &sim) override;
    bool update(double time,Sim &sim) override;
    void set_fitness_fct(const fitness::Function::Ptr& fct){_fitness_fct = fct;}
private:
    fitness::Function::Ptr _fitness_fct = nullptr;
    double _width = 0;
    double _length = 0;
};

}//ea_rg
