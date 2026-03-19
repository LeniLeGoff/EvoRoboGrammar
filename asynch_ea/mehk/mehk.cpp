#include "mehk.hpp"
#include "apear/algorithms/homeokinetic_controller.hpp"
#include "apear/async_dealer.hpp"

#include "ea_rg/tasks.hpp"

using namespace ea_rg;
namespace apear_st = apear::settings;

void MEHKInd::init(){
    _morph_genome->init();
    _morph_genome->random();
}


void MEHKInd::_create_controller(){
    int dof = get_robot_dof();
    if(dof == 0)
        return;
    _control = std::make_shared<apear::hk::Homeokinesis>(dof,dof);
    _control->set_random_number(_rand_num);
    _control->set_parameters(_parameters);

    if(apear_st::getParameter<apear_st::Boolean>(_parameters,"#initHKNoise").value)
        std::dynamic_pointer_cast<apear::hk::Homeokinesis>(_control)->add_noise(
            apear_st::getParameter<apear_st::Double>(_parameters,"#HKNoiseStrength").value);
}




int main(int argc, char** argv){
    if(argc < 4){
        std::cout << "usage: " << std::endl
                  << "\targ 1: path to parameters file" << std::endl
                  << "\targ 2: number of simulators" << std::endl
                  << "\targ 3: headless mode (0|1)" << std::endl;
        return 1;
    }
    int nbr_sim = std::stoi(argv[2]);
    bool headless = std::stoi(argv[3]) == 1;

    //load parameters
    apear_st::ParametersMapPtr param = std::make_shared<apear_st::ParametersMap>(
       apear_st::loadParameters(argv[1]));

    //create log folder
    std::string log_repo = apear_st::getParameter<apear_st::String>(param,"#logRepository").value;
    apear::logging::create_log_folder(log_repo + "/" + apear_st::getParameter<apear_st::String>(param,"#experimentName").value);

    //initialise random number generator
    int seed = apear_st::getParameter<apear_st::Integer>(param,"#seed").value;
    if(seed == -1){
        std::random_device rd;
        seed = rd();
        apear_st::random::parameters->emplace("#seed",std::make_shared<const apear_st::Integer>(seed));
    }
    apear::misc::RandNum::Ptr rand_num = std::make_shared<apear::misc::RandNum>(seed);

    //create the asynchronous dealer
    apear::AsyncDealer<MEHKInd,RoboGrammarSimulator> dealer(param,rand_num);

    //add logging classes
    dealer.add_logging(std::make_shared<RGGenomeLog>(apear_st::getParameter<apear_st::String>(param,"#genomeLogFile").value));
    dealer.add_logging(std::make_shared<FitnessLog>(apear_st::getParameter<apear_st::String>(param,"#fitnessLogFile").value));
    dealer.add_logging(std::make_shared<ParentsLog>(apear_st::getParameter<apear_st::String>(param,"#parentsLogFile").value));
    dealer.add_logging(std::make_shared<RolloutLog>());
    dealer.add_logging(std::make_shared<TrajectoryLog>());


    //create the evolutionary algorithm: Asynchrounous Morpho-Evolution with Homeokinetic controller
    apear::EA<MEHKInd>::Ptr ame = std::make_unique<apear::AsyncMorphoEvolution<MEHKInd,RoboGrammarGenome>>(rand_num,param);
    ame->init();

    //initialise the asynchronous dealer
    dealer.set_ea(ame);
    dealer.init(nbr_sim,headless);

    //define the environment and task
    std::vector<double> arena_size = apear_st::getParameter<apear_st::Sequence<double>>(param,"#arenaSize").value;

    dealer.set_environment<ea_rg::FlatArena>(arena_size[0],arena_size[1],ea_rg::fitness::Exploration(param));

    apear_st::saveParameters(apear::logging::log_folder + "/parameters.csv",param);

    //run the experiment
    while (dealer.update_simulators()) {}

    return 0;
}

void RGGenomeLog::saveLog(const apear::EA<MEHKInd>::Ptr &ea){
    std::ofstream log_file;
    if(openOLogFile(log_file)){
        for(const IndPtr &ind: ea->evaluated()){
            log_file << ind->get_morph_genome()->id() << " " << ind->get_morph_genome()->to_string() << std::endl;
        }
        log_file.close();
    }
}

void FitnessLog::saveLog(const apear::EA<MEHKInd>::Ptr &ea){
    std::ofstream log_file;
    if(openOLogFile(log_file)){
        for(const IndPtr &ind: ea->evaluated()){
            log_file << ind->get_morph_genome()->id()
            << "," << ind->get_morph_genome()->get_parents_ids()[0]
            << "," << ind->get_morph_genome()->get_parents_ids()[1]
            << "," << ind->get_objectives()[0] << std::endl;
        }
        log_file.close();
    }
}

void ParentsLog::saveLog(const apear::EA<MEHKInd>::Ptr &ea){
    int pop_size = apear_st::getParameter<apear_st::Integer>(ea->get_parameters(),"#populationSize").value;
    if(static_cast<apear::AsyncMorphoEvolution<MEHKInd,RoboGrammarGenome>*>(ea.get())->get_parent_pool().size() < pop_size)
        return;
    std::ofstream log_file;
    if(openOLogFile(log_file)){
        const std::vector<IndPtr> &parents = static_cast<apear::AsyncMorphoEvolution<MEHKInd,RoboGrammarGenome>*>(ea.get())->get_parent_pool();
        log_file << parents.front()->id();
        for(size_t i = 1; i < parents.size(); i++){
            log_file << "," << parents[i]->id();
        }
        log_file << std::endl;
        log_file.close();
    }
}

void RolloutLog::_register_data(const IndPtr& ind, const RoboGrammarSimulator &sim){
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

void RolloutLog::saveLog(const apear::EA<MEHKInd>::Ptr& ea){
    std::ofstream log_file;
    for(const IndPtr &ind: ea->evaluated()){
        if(_data.find(ind->id()) == _data.end())
            continue;
        std::stringstream sstr;
        sstr << "rollout_" << ind->id() << ".csv";
        if(openOLogFile(log_file,sstr.str())){
            for(const apear::act_obs_t &ao: _data[ind->id()])
                log_file << ao.to_string() << std::endl;
        }
        log_file.close();
        _data.erase(ind->id());
    }
}

void TrajectoryLog::_register_data(const IndPtr& ind, const RoboGrammarSimulator &sim){
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

void TrajectoryLog::saveLog(const apear::EA<MEHKInd>::Ptr& ea){
    std::ofstream log_file;
    for(const IndPtr &ind: ea->evaluated()){
        if(_data.find(ind->id()) == _data.end())
            continue;
        std::stringstream sstr;
        sstr << "traj_" << ind->id() << ".csv";
        if(openOLogFile(log_file,sstr.str())){
            for(const apear::waypoint_t &wp: _data[ind->id()])
                log_file << wp.to_string() << std::endl;
        }
        log_file.close();
        _data.erase(ind->id());
    }
}
