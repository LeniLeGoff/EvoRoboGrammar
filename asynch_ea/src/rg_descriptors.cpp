#include "ea_rg/rg_descriptors.hpp"
#include <iostream>
#include <string>

using namespace ea_rg;
namespace rg = robot_design;

void RoboGrammarDescriptors::use_all_descriptors(){
    descriptors = {number_of_parts,
                number_of_body_fixed_joints,
                number_of_body_lift_joints,
                number_of_body_roll_joints,
                number_of_body_swing_joints,
                number_of_limb_roll_joints,
                number_of_limb_lift_joints,
                number_of_limb_swing_joints,
                number_of_limbs,
                median_limbs_length,
                max_limbs_length,
                min_limbs_length,
                length_of_body};
}

void RoboGrammarDescriptors::set_descriptors(const std::vector<descriptor_fct> &desc){
    descriptors = desc;
}

std::vector<double> RoboGrammarDescriptors::compute_descriptor_vector(const rg::Graph& graph){
    std::vector<double> desc_vect;
    for(const descriptor_fct &desc: descriptors)
        desc_vect.push_back(desc(graph));
    return desc_vect;
}

descriptor_fct RoboGrammarDescriptors::number_of_parts = [](const rg::Graph& graph) -> double{
    double descr = 0;
    for(const rg::Edge& edge: graph.edges_){
        std::string type;
        std::stringstream label(edge.attrs_.label_);
        std::getline(label,type,'_');
        if(type == "body")
            descr+=1;
        else if(type == "limb")
            descr+=2;
        else{
            std::cerr << "Error RoboGrammarDescriptors::number_of_parts: Unknown part type :" << type << std::endl;
            return -1;
        }
    }

    for(const rg::Node& node: graph.nodes_){
        std::string type;
        std::stringstream label(node.name_);
        std::getline(label,type,'_');
        if(type == "body")
            descr+=1;
        else if(type == "limb")
            descr+=2;
        else{
            std::cerr << "Error RoboGrammarDescriptors::number_of_parts: Unknown part type :" << type << std::endl;
            return -1;
        }
    }
    return descr;
};

descriptor_fct RoboGrammarDescriptors::number_of_limbs = [](const rg::Graph& graph) -> double{
    double desc = 0;
    for(const rg::Edge& edge: graph.edges_){
        if(edge.attrs_.label_ == "body_limb_mount")
            desc+=1;
    }
    return desc;
};

descriptor_fct RoboGrammarDescriptors::length_of_body = [](const rg::Graph& graph) -> double{
    double descr = 0;

    for(const rg::Node& node: graph.nodes_){
        std::string type;
        std::stringstream label(node.name_);
        std::getline(label,type,'_');
        if(type == "body")
            descr+=1;
    }
    return descr;
};

descriptor_fct RoboGrammarDescriptors::median_limbs_length = [](const rg::Graph& graph) -> double{
    double desc = 0;
    //First find the nodes label with limb mount
    //Then count with edges how many nodes are connect to these

    return desc;
};
