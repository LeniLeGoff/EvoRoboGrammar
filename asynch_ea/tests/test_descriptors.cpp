#include "ea_rg/rg_genome.hpp"
#include "ea_rg/rg_descriptors.hpp"
#include "apear/misc/utilities.hpp"


using namespace ea_rg;

std::vector<rule_idx_t> extract_rule_seq(char** argv,int argc){
    std::vector<rule_idx_t> rule_seq;
    for(int i = 0; i < argc; i++){
        std::string arg(argv[i]);
        if(arg == "-")
            break;
        std::vector<std::string> tokens;
        apear::misc::split_line(arg,",",tokens);
        if(tokens.size() != 2){
            std::cout << "invalid rule sequence format, expected format: ruleType,ruleIdx" << std::endl;
            exit(1);
        }
        int rule_type = std::stoi(tokens[0]);
        int rule_idx = std::stoi(tokens[1]);
        rule_seq.push_back(std::make_pair(rule_type,rule_idx));
    }
    return rule_seq;
}

int main(int argc, char** argv){
    if(argc == 2){
        std::cout << "usage: arg1 : grammar file" << std::endl
                  << "\targs: rule sequences" << std::endl;
        return 1;
    }
    std::vector<rule_idx_t> rule_seq = extract_rule_seq(argv+2,argc-2);


    RoboGrammarGenome rgg;
    rgg.make_grammar(argv[1]);
    rgg.set_rule_seq(rule_seq);

    RoboGrammarDescriptors descriptors;
    descriptors.use_all_descriptors();

    std::vector<double> desc_vect = descriptors.compute_descriptor_vector(rgg.get_graph());
    for(size_t i = 0; i < desc_vect.size(); i++)
        std::cout << descriptors.descriptor_names[i] << ": " << desc_vect[i] << std::endl;
    return 0;
}
