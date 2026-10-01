#include "ea_rg/io.hpp"

void print::pose(const RoboGrammarSimulator &sim){
    rd::Vector3 pos;
    rd::Quaternion ori;
    sim.get_sim()->getRobotPositionAndOrientation(sim.get_robot_idx(),pos,ori);
    apear::waypoint_t wp;
    wp.position = {pos[0],pos[1],pos[2]};
    wp.quat_ori = {ori.x(),ori.y(),ori.z(),ori.w()};
    wp.time = sim.time();
    std::cout << wp.to_string() << std::endl;
}
void print::rollout(const RoboGrammarSimulator &sim){
    int dof  = sim.get_sim()->getRobotDofCount(sim.get_robot_idx());
    rd::VectorX act(dof);
    sim.get_sim()->getJointTargetPositions(sim.get_robot_idx(),act);
    rd::VectorX obs(dof);
    sim.get_sim()->getJointPositions(sim.get_robot_idx(),obs);
    std::vector<double> action(act.rows()), observation(obs.rows());
    for(int i = 0; i < act.rows(); i++)
        action[i] = act[i]/M_PI_2; //scale the action to [-1,1]
    for(int i = 0; i < obs.rows(); i++)
        observation[i] = obs[i]/M_PI_2; //scale the observation to [-1,1]
    std::cout << apear::act_obs_t(sim.time(),observation,action).to_string() << std::endl;
}
