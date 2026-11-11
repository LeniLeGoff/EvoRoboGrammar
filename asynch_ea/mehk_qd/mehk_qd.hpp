#pragma once
#include "apear/algorithms/archives.hpp"
#include "ea_rg/rg_genome.hpp"
#include "ea_rg/rg_simulator.hpp"
#include "apear/logging.hpp"
#include "robot_design/graph.h"



//ARCHIVE SELECTORS
namespace ea_rg {


enum SelectorType{
    RANDOM = 0,
    SPARSITY,
    PARETO
};

struct selectors {
    static apear::selectors::selector_t<HKInd::Ptr> random;
    static int sparsity_top_nbr;
    static apear::selectors::selector_t<HKInd::Ptr> sparsity;
    static double pareto_goal_tolerance;
    static double pareto_sparsity_tolerance;
    static apear::selectors::selector_t<HKInd::Ptr> pareto_tournament;
};

template<class ind_t>
void load_archive(apear::UnstructuredArchive<std::shared_ptr<ind_t>,int> &archive,const std::string &log_folder, const std::string &grammar_file){
    using ind_t_ptr = std::shared_ptr<ind_t>;
    std::ifstream gen_file, fit_file, parent_file;

    parent_file.open(log_folder + "/parent_pool.csv");
    if(!parent_file){
        std::cerr << "Unable to open " << log_folder << "/parent_pool.csv" << std::endl;
        return;
    }

    //Load ids from parent pool file
    std::string line, last_line;
    while(std::getline(parent_file,line))
        last_line = line;
    std::vector<std::string> string_ids;
    apear::misc::split_line(last_line,",",string_ids);
    std::vector<int> parent_ids;
    for(const std::string &id: string_ids)
        parent_ids.push_back(std::stoi(id));
    parent_file.close();

    gen_file.open(log_folder + "/genomes.csv");
    if(!gen_file){
        std::cerr << "Unable to open " << log_folder << "/genomes.csv" << std::endl;
        return;
    }

    fit_file.open(log_folder + "/fitnesses.csv");
    if(!fit_file){
        std::cerr << "Unable to open " << log_folder << "/fitnesses.csv" << std::endl;
        return;
    }

    std::map<int,ind_t_ptr> parents;
    for(const int &id: parent_ids){
        std::cout << "create individual " << id << std::endl;
        RoboGrammarGenome::Ptr gen = std::make_shared<RoboGrammarGenome>();
        gen->make_grammar(grammar_file);
        gen->set_id(id);
        parents[id] = std::make_shared<ind_t>(gen,std::make_shared<apear::EmptyGenome>());
    }
    //Extract genomes
    std::cout << "Extract genomes " << std::endl;
    for(std::string line; std::getline(gen_file,line);){
        std::cout << line << std::endl;
        std::vector<std::string> splitted_line;
        apear::misc::split_line(line," ",splitted_line);
        int id = std::stoi(splitted_line[0]);
        if(parents.find(id) == parents.end())
            continue;
        std::vector<rule_idx_t> rule_seq;
        for(size_t i = 1; i < splitted_line.size(); i++){
            std::vector<std::string> tokens(2);
            apear::misc::split_line(splitted_line[i],",",tokens);
            int rule_type = std::stoi(tokens[0]);
            int rule_idx = std::stoi(tokens[1]);
            rule_seq.push_back(std::make_pair(rule_type,rule_idx));
        }
        parents[id]->set_rules(rule_seq);
    }
    gen_file.close();

    //extract fitness scores
    std::cout << "Extract fitnesses " << std::endl;
    for(std::string line; std::getline(fit_file,line);){
        std::cout << line << std::endl;
        std::vector<std::string> splitted_line;
        apear::misc::split_line(line,",",splitted_line);
        int id = std::stoi(splitted_line[0]);
        if(parents.find(id) == parents.end())
            continue;
        if(std::stoi(splitted_line[0]) == id){
            parents[id]->set_objectives({std::stod(splitted_line[3])});
            parents[id]->get_morph_genome()->set_parents_ids({std::stoi(splitted_line[1]),std::stoi(splitted_line[2])});
        }
    }
    fit_file.close();
    for(const auto& [id,parent]: parents)
        archive.push_back(parent);
}


//LOGGING CLASSES
class ParentsLog: public apear::Logging<HKInd,RoboGrammarSimulator>{
public:
    ParentsLog(): Logging(){}
    ParentsLog(const std::string &file) : Logging(file){}
    ParentsLog(const ParentsLog& l) : Logging(l){}
    void saveLog(const apear::EA<HKInd>::Ptr&) override;
};
}
