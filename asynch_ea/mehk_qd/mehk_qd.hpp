#pragma once
#include "apear/algorithms/archives.hpp"
#include "ea_rg/rg_genome.hpp"
#include "ea_rg/rg_simulator.hpp"
#include "apear/logging.hpp"
#include "robot_design/graph.h"


//ARCHIVE SELECTORS
namespace ea_rg {

enum ArchiveType{
    UNSTRUCTURED = 0,
    GRID
};

enum SelectorType{
    RANDOM = 0,
    SPARSITY,
    PARETO
};

void init_grid_archive(apear::EA<HKInd>::Ptr &ame);
void init_unstructured_archive(apear::EA<HKInd>::Ptr &ame);

struct selectors {
    template<class ind_t>
    using deterministic_selector_t = std::function< std::vector<ind_t>(const std::vector<ind_t>&)>;
    static apear::selectors::selector_t<HKInd::Ptr> random;
    static int sparsity_top_nbr;
    static apear::selectors::selector_t<HKInd::Ptr> sparsity;
    static double pareto_goal_tolerance;
    static double pareto_sparsity_tolerance;
    static apear::selectors::selector_t<HKInd::Ptr> pareto_tournament;
    template<class ind_t>
    static inline deterministic_selector_t<ind_t> pareto = [](const std::vector<ind_t>& solutions) -> std::vector<ind_t>{
        std::vector<ind_t> selection;
        if(solutions.size() == 1){
            selection.push_back(solutions.at(0));
            return selection;
        }
        std::vector<size_t> pareto_front;
        std::vector<bool> undominated(solutions.size(),true);
        bool dominating_sparsity = false;
        bool dominating_quality = false;
        for(size_t i = 0; i < solutions.size(); i++){
            for(size_t j = i; j < solutions.size(); j++){
                if(undominated[i] == false)
                    break;
                if(undominated[j] == false)
                    continue;
                const ind_t &ind1 = solutions.at(i);
                const ind_t &ind2 = solutions.at(j);
                if(ind1->get_morph_genome()->id() == ind2->get_morph_genome()->id())
                    continue;
                dominating_quality = false;
                dominating_sparsity = false;
                if(ind1->get_objectives()[0] > ind2->get_objectives()[0])
                    dominating_quality = true;
                if(ind1->get_sparsity_score() > ind2->get_sparsity_score())
                    dominating_sparsity = true;
                if(dominating_quality && dominating_sparsity)
                    undominated[j] = false;
                dominating_quality = false;
                dominating_sparsity = false;
                if(ind2->get_objectives()[0] > ind1->get_objectives()[0])
                    dominating_quality = true;
                if(ind2->get_sparsity_score() > ind1->get_sparsity_score())
                    dominating_sparsity = true;
                if(dominating_quality && dominating_sparsity)
                    undominated[i] = false;
            }
        }
        for(size_t i = 0; i < undominated.size(); i++){
            if(undominated[i])
                pareto_front.push_back(i);
        }
        std::cout << std::endl;
        for(size_t i : pareto_front){
            selection.push_back(solutions.at(i));
        }
        return selection;
    };
};



template<class ind_t>
void load_archive(apear::UnstructuredArchive<std::shared_ptr<ind_t>,int> &archive,const std::string &log_folder, const std::string &grammar_file){
    using ind_t_ptr = std::shared_ptr<ind_t>;
    std::ifstream gen_file, parent_file;

    parent_file.open(log_folder + "/parent_pool.csv");
    if(!parent_file){
        std::cerr << "Unable to open " << log_folder << "/parent_pool.csv" << std::endl;
        return;
    }

    //Load ids from parent pool file
    std::string line, last_line;
    while(std::getline(parent_file,line))
        last_line = line;
    std::vector<std::string> string_ind;
    apear::misc::split_line(last_line,";",string_ind);
    std::vector<int> parent_ids;
    std::vector<double> fitnesses, sparsity_scores;
    for(const std::string &ind: string_ind){
        std::vector<std::string> infos;
        apear::misc::split_line(ind,",",infos);
        parent_ids.push_back(std::stoi(infos[0]));
        fitnesses.push_back(std::stod(infos[1]));
        sparsity_scores.push_back(std::stod(infos[2]));
    }
    parent_file.close();

    gen_file.open(log_folder + "/genomes.csv");
    if(!gen_file){
        std::cerr << "Unable to open " << log_folder << "/genomes.csv" << std::endl;
        return;
    }


    std::map<int,ind_t_ptr> parents;
    for(int i = 0; i < parent_ids.size(); i++){
        int id = parent_ids[i];
        // std::cout << "create individual " << id << std::endl;
        RoboGrammarGenome::Ptr gen = std::make_shared<RoboGrammarGenome>();
        gen->make_grammar(grammar_file);
        gen->set_id(id);
        parents[id] = std::make_shared<ind_t>(gen,std::make_shared<apear::EmptyGenome>());
        parents[id]->set_objectives({fitnesses[i]});
        parents[id]->set_sparsity_score(sparsity_scores[i]);
    }
    //Extract genomes
    std::cout << "Extract genomes " << std::endl;
    for(std::string line; std::getline(gen_file,line);){
        // std::cout << line << std::endl;
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

class DescriptorsLog: public apear::Logging<HKInd,RoboGrammarSimulator>{
public:
    DescriptorsLog(): Logging(){}
    DescriptorsLog(const std::string &file) : Logging(file){}
    DescriptorsLog(const DescriptorsLog& l) : Logging(l){}
    void saveLog(const apear::EA<HKInd>::Ptr&) override;
};
}//ea_rg
