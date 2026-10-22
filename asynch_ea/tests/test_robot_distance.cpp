#include "ea_rg/rg_genome.hpp"
#include "apear/misc/utilities.hpp"


using namespace ea_rg;

std::vector<RoboGrammarGenome::rule_idx_t> extract_rule_seq(char** argv,int argc){
    std::vector<RoboGrammarGenome::rule_idx_t> rule_seq;
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
    std::vector<RoboGrammarGenome::rule_idx_t> rule_seq1 = extract_rule_seq(argv+2,argc-2);
    std::vector<RoboGrammarGenome::rule_idx_t> rule_seq2 = extract_rule_seq(argv+3+rule_seq1.size(),
                                                                            argc-3-rule_seq1.size());

    RoboGrammarGenome rgg1;
    rgg1.make_grammar(argv[1]);
    rgg1.set_rule_seq(rule_seq1);
    RoboGrammarGenome rgg2;
    rgg2.make_grammar(argv[1]);
    rgg2.set_rule_seq(rule_seq2);

    int distance = robot_design::tree_edit_distance(rgg1.get_graph(),rgg2.get_graph());
    std::cout << "Distance: " << distance << std::endl;

    return 0;
}
