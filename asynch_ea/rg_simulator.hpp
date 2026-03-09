#pragma once

#include "apear/simulator.hpp"
#include "rg_genome.hpp"
#include "robot_design/sim.h"
#include "robot_design/glfw_viewer.h"


namespace ea_rg{

namespace rd = robot_design;

class RoboGrammarSimulator: public apear::Simulator<RoboGrammarInd>{
public:
    RoboGrammarSimulator() = delete;
    RoboGrammarSimulator(apear::settings::ParametersMapPtr &param, bool headless = true);

    bool init_environment(const apear::Environment::Ptr &env) override;
    bool init(const IndPtr &ind) override;
    bool step() override;
    bool stop() override;
    void update_ind(IndPtr &ind, const apear::Environment::Ptr& env) override;
    apear::sim_state_t state() override;
    double time() override;
    void reconnect() override;

private:
    std::shared_ptr<rd::BulletSimulation> _sim = nullptr;
    double _max_episode_time = 0;
    double _time = 0;
    double _time_step = 0;
    std::shared_ptr<rd::GLFWViewer> _viewer = nullptr;
    int _robot_idx = 0;
};

}
