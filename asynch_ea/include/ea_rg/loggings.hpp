#pragma once
#include "apear/logging.hpp"
#include "ea_rg/rg_genome.hpp"
#include "ea_rg/rg_simulator.hpp"

namespace ea_rg {

template<class ind_t>
class RGGenomeLog: public apear::Logging<ind_t,RoboGrammarSimulator>{
public:
    using IndPtr = std::shared_ptr<ind_t>;
    RGGenomeLog() : apear::Logging<ind_t,RoboGrammarSimulator>(){}
    RGGenomeLog(const std::string &file) : apear::Logging<ind_t,RoboGrammarSimulator>(file){}
    RGGenomeLog(const RGGenomeLog& l) : apear::Logging<ind_t,RoboGrammarSimulator>(l){}
    void saveLog(const typename apear::EA<ind_t>::Ptr &ea) override{
        std::ofstream log_file;
        if(this->openOLogFile(log_file)){
            for(const IndPtr &ind: ea->evaluated()){
                log_file << ind->get_morph_genome()->id() << " " << ind->get_morph_genome()->to_string() << std::endl;
            }
            log_file.close();
        }
    }
};

template<class ind_t>
class FitnessLog: public apear::Logging<ind_t,RoboGrammarSimulator>{
public:
    using IndPtr = std::shared_ptr<ind_t>;
    FitnessLog(): apear::Logging<ind_t,RoboGrammarSimulator>(){}
    FitnessLog(const std::string &file) : apear::Logging<ind_t,RoboGrammarSimulator>(file){}
    FitnessLog(const FitnessLog& l) : apear::Logging<ind_t,RoboGrammarSimulator>(l){}
    void saveLog(const typename apear::EA<ind_t>::Ptr &ea) override{
        std::ofstream log_file;
        if(this->openOLogFile(log_file)){
            for(const IndPtr &ind: ea->evaluated()){
                log_file << ind->get_morph_genome()->id()
                << "," << ind->get_morph_genome()->get_parents_ids()[0]
                << "," << ind->get_morph_genome()->get_parents_ids()[1]
                << "," << ind->get_objectives()[0] << std::endl;
            }
            log_file.close();
        }
    }
};

template<class ind_t>
class RolloutLog: public apear::Logging<ind_t,RoboGrammarSimulator>{
public:
    using IndPtr = std::shared_ptr<ind_t>;
    RolloutLog(): apear::Logging<ind_t,RoboGrammarSimulator>(){}
    RolloutLog(const std::string &file) : apear::Logging<ind_t,RoboGrammarSimulator>(file){}
    RolloutLog(const RolloutLog& l) : apear::Logging<ind_t,RoboGrammarSimulator>(l){}
    void saveLog(const typename apear::EA<ind_t>::Ptr& ea) override{
        std::ofstream log_file;
        for(const IndPtr &ind: ea->evaluated()){
            if(_data.find(ind->id()) == _data.end())
                continue;
            std::stringstream sstr;
            sstr << "rollout_" << ind->id() << ".csv";
            if(this->openOLogFile(log_file,sstr.str())){
                for(const apear::act_obs_t &ao: _data[ind->id()])
                    log_file << ao.to_string() << std::endl;
            }
            log_file.close();
            _data.erase(ind->id());
        }
    }
protected:
    void _register_data(const IndPtr& ind, const RoboGrammarSimulator& sim) override{
        if(ind->get_control() == nullptr){
            return;
        }
        if(_data.find(ind->id()) == _data.end())
            _data[ind->id()] = apear::rollout_t();
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
        _data[ind->id()].push_back(apear::act_obs_t(sim.time(),observation,action));
    }
private:
    std::map<int,apear::rollout_t> _data;
};

template<class ind_t>
class TrajectoryLog: public apear::Logging<ind_t,RoboGrammarSimulator>{
public:
    using IndPtr = std::shared_ptr<ind_t>;
    TrajectoryLog(): apear::Logging<ind_t,RoboGrammarSimulator>(){}
    TrajectoryLog(const std::string &file) : apear::Logging<ind_t,RoboGrammarSimulator>(file){}
    TrajectoryLog(const TrajectoryLog& l) : apear::Logging<ind_t,RoboGrammarSimulator>(l){}
    void saveLog(const typename apear::EA<ind_t>::Ptr& ea) override{
        std::ofstream log_file;
        for(const IndPtr &ind: ea->evaluated()){
            if(_data.find(ind->id()) == _data.end())
                continue;
            std::stringstream sstr;
            sstr << "traj_" << ind->id() << ".csv";
            if(this->openOLogFile(log_file,sstr.str())){
                for(const apear::waypoint_t &wp: _data[ind->id()])
                    log_file << wp.to_string() << std::endl;
            }
            log_file.close();
            _data.erase(ind->id());
        }
    }

protected:
    void _register_data(const IndPtr& ind, const RoboGrammarSimulator& sim) override{
        if(ind->get_control() == nullptr){
            return;
        }
        if(_data.find(ind->id()) == _data.end())
            _data[ind->id()] = apear::trajectory_t();
        rd::Vector3 pos;
        rd::Quaternion ori;
        sim.get_sim()->getRobotPositionAndOrientation(sim.get_robot_idx(),pos,ori);

        apear::waypoint_t wp;
        wp.position = {pos[0],pos[1],pos[2]};
        wp.quat_ori = {ori.x(),ori.y(),ori.z(),ori.w()};
        wp.time = sim.time();
        _data[ind->id()].push_back(wp);
    }
private:
    std::map<int,apear::trajectory_t> _data;
};

}//ea_rg
