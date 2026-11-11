#include "mehk_qd/mehk_qd.hpp"

int main(int argc, char** argv){
    if(argc < 3){
        std::cout << "usage : arg 1 grammar file" << std::endl
                  << "\t arg 2: path to log folder" << std::endl;
    }

    apear::UnstructuredArchive<ea_rg::RoboGrammarInd::Ptr,int> archive;

    ea_rg::load_archive<ea_rg::RoboGrammarInd>(archive,argv[2],argv[1]);


    return 0;
}
