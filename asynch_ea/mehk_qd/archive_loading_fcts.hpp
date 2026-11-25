#pragma once
#include <fstream>
#include "apear/algorithms/archives.hpp"
#include "ea_rg/rg_genome.hpp"

namespace ea_rg{


template<class ind_t>
void load_unstructured_archive(apear::UnstructuredArchive<std::shared_ptr<ind_t>,int> &archive,const std::string &log_folder, const std::string &grammar_file){
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

    std::map<int,ind_t_ptr> parents;
    for(size_t i = 0; i < parent_ids.size(); i++){
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
    gen_file.open(log_folder + "/genomes.csv");
    if(!gen_file){
        std::cerr << "Unable to open " << log_folder << "/genomes.csv" << std::endl;
        return;
    }

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

template<class ind_t>
void load_infinite_grid_archive(apear::InfiniteGridArchive<std::shared_ptr<ind_t>> &archive,const std::string &log_folder, const std::string &grammar_file){
    using ind_t_ptr = std::shared_ptr<ind_t>;
    std::ifstream gen_file, parent_file, desc_file;

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

    std::map<int,ind_t_ptr> parents;
    for(size_t i = 0; i < parent_ids.size(); i++){
        int id = parent_ids[i];
        // std::cout << "create individual " << id << std::endl;
        RoboGrammarGenome::Ptr gen = std::make_shared<RoboGrammarGenome>();
        gen->make_grammar(grammar_file);
        gen->set_id(id);
        parents[id] = std::make_shared<ind_t>(gen,std::make_shared<apear::EmptyGenome>());
        parents[id]->set_objectives({fitnesses[i]});
        parents[id]->set_sparsity_score(sparsity_scores[i]);
    }

    //load genome files
    gen_file.open(log_folder + "/genomes.csv");
    if(!gen_file){
        std::cerr << "Unable to open " << log_folder << "/genomes.csv" << std::endl;
        return;
    }

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


    //load descriptors
    desc_file.open(log_folder + "/descriptors.csv");
    if(!desc_file){
        std::cerr << "Unable to open " << log_folder << "/descriptors.csv" << std::endl;
        return;
    }

    for(std::string line; std::getline(desc_file,line);){
        std::vector<std::string> tokens;
        apear::misc::split_line(line,",",tokens);
        int id = std::stoi(tokens[0]);
        if(parents.find(id) == parents.end())
            continue;
        std::vector<double> desc;
        for(size_t i = 1; i < tokens.size(); i++)
            desc.push_back(std::stod(tokens[i]));
        parents[id]->set_descriptors(desc);
    }
    for(const auto& [id,parent]: parents)
        archive.push_back(parent);

}

}//ea_rg
