#include "mehk_qd/mehk_qd.hpp"
using namespace ea_rg;
using Archive = apear::UnstructuredArchive<RoboGrammarInd::Ptr,int>;


int main(int argc, char** argv){
    if(argc < 3){
        std::cout << "usage : arg 1 grammar file" << std::endl
                  << "\t arg 2: path to log folder" << std::endl;
    }
    Archive::comparator_t comparator([](const RoboGrammarInd::Ptr& a, const RoboGrammarInd::Ptr& b) -> bool{//comparator for quality
        return a->get_objectives()[0] > b->get_objectives()[0];
    });
    Archive::distance_fct_t distance_fct([](const RoboGrammarInd::Ptr& a, const RoboGrammarInd::Ptr& b) -> int{//distance for diversity
        return robot_design::tree_edit_distance(std::dynamic_pointer_cast<RoboGrammarGenome>(a->get_morph_genome())->get_graph(),
                                                std::dynamic_pointer_cast<RoboGrammarGenome>(b->get_morph_genome())->get_graph());
    });
    Archive archive(1,distance_fct,comparator,apear::selectors::random<RoboGrammarInd::Ptr>,5);

    ea_rg::load_archive<ea_rg::RoboGrammarInd>(archive,argv[2],argv[1]);

    archive.compute_sparsity_scores(5);
    std::vector<ea_rg::RoboGrammarInd::Ptr> pareto_front = ea_rg::selectors::pareto<ea_rg::RoboGrammarInd::Ptr>(archive.get_solutions());
    for(const ea_rg::RoboGrammarInd::Ptr &ind: pareto_front){
        std::cout << ind->get_morph_genome()->id() << " fitness: " << ind->get_objectives()[0] << " sparsity: " << ind->get_sparsity_score() << std::endl;
    }
    std::cout << "Pareto front size: " << pareto_front.size() << std::endl;

    return 0;
}
