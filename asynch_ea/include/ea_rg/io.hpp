#pragma once
#include "ea_rg/rg_simulator.hpp"

using namespace ea_rg;

struct print{
    static void pose(const RoboGrammarSimulator &sim);
    static void rollout(const RoboGrammarSimulator &sim, bool normalized = true);
    static void torques(const RoboGrammarSimulator &sim);
};
