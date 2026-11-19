#include "ea_rg/rg_descriptors.hpp"
#include <iostream>
#include <string>
#include <map>
#include <math.h>


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
    descriptor_names = {GET_VARIABLE_NAME(number_of_parts),
                        GET_VARIABLE_NAME(number_of_body_fixed_joints),
                        GET_VARIABLE_NAME(number_of_body_lift_joints),
                        GET_VARIABLE_NAME(number_of_body_roll_joints),
                        GET_VARIABLE_NAME(number_of_body_swing_joints),
                        GET_VARIABLE_NAME(number_of_limb_roll_joints),
                        GET_VARIABLE_NAME(number_of_limb_lift_joints),
                        GET_VARIABLE_NAME(number_of_limb_swing_joints),
                        GET_VARIABLE_NAME(number_of_limbs),
                        GET_VARIABLE_NAME(median_limbs_length),
                        GET_VARIABLE_NAME(max_limbs_length),
                        GET_VARIABLE_NAME(min_limbs_length),
                        GET_VARIABLE_NAME(length_of_body)};
}

void RoboGrammarDescriptors::set_descriptors(const std::vector<descriptor_fct> &desc){
    descriptors = desc;
   //TO DO insert variable names
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
        if(edge.attrs_.label_ == "body_limb_mount")
            continue;
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

descriptor_fct RoboGrammarDescriptors::number_of_body_fixed_joints = [](const rg::Graph& graph) -> double{
    double desc = 0;
    for(const rg::Edge& edge: graph.edges_){
        if(edge.attrs_.label_ == "body_fixed_joint")
            desc+=1;
    }
    return desc;
};

descriptor_fct RoboGrammarDescriptors::number_of_body_lift_joints = [](const rg::Graph& graph) -> double{
    double desc = 0;
    for(const rg::Edge& edge: graph.edges_){
        if(edge.attrs_.label_ == "body_lift_joint")
            desc+=1;
    }
    return desc;
};

descriptor_fct RoboGrammarDescriptors::number_of_body_swing_joints = [](const rg::Graph& graph) -> double{
    double desc = 0;
    for(const rg::Edge& edge: graph.edges_){
        if(edge.attrs_.label_ == "body_swing_joint")
            desc+=1;
    }
    return desc;
};

descriptor_fct RoboGrammarDescriptors::number_of_body_roll_joints = [](const rg::Graph& graph) -> double{
    double desc = 0;
    for(const rg::Edge& edge: graph.edges_){
        if(edge.attrs_.label_ == "body_roll_joint")
            desc+=1;
    }
    return desc;
};

descriptor_fct RoboGrammarDescriptors::number_of_limb_lift_joints = [](const rg::Graph& graph) -> double{
    double desc = 0;
    for(const rg::Edge& edge: graph.edges_){
        if(edge.attrs_.label_ == "limb_lift_joint")
            desc+=2;
    }
    return desc;
};

descriptor_fct RoboGrammarDescriptors::number_of_limb_swing_joints = [](const rg::Graph& graph) -> double{
    double desc = 0;
    for(const rg::Edge& edge: graph.edges_){
        if(edge.attrs_.label_ == "limb_swing_joint")
            desc+=2;
    }
    return desc;
};

descriptor_fct RoboGrammarDescriptors::number_of_limb_roll_joints = [](const rg::Graph& graph) -> double{
    double desc = 0;
    for(const rg::Edge& edge: graph.edges_){
        if(edge.attrs_.label_ == "limb_left_roll_joint" || edge.attrs_.label_ == "limb_right_roll_joint")
            desc+=2;
    }
    return desc;
};

descriptor_fct RoboGrammarDescriptors::max_limbs_length = [](const rg::Graph& graph) -> double{
    if(number_of_limbs(graph) == 0)
        return 0;
    double desc = 0;
    //First find the nodes label with limb mount
    std::map<int,double> limb_lengths;
    for(size_t i = 0; i < graph.nodes_.size(); i++){
        if(graph.nodes_[i].attrs_.label_ == "limb_mount")
            limb_lengths[i] = 1;
    }
    std::vector<const rg::Edge *> limb_links;
    for(const rg::Edge& edge: graph.edges_){
        std::string type;
        std::stringstream label(edge.attrs_.label_);
        std::getline(label,type,'_');
        if(type == "limb"){
            limb_links.push_back(&edge);
        }
    }

    //Then count with edges how many nodes are connect to these
    size_t i = 0;
    while(limb_links.size() > 0){
        const rg::Edge *edge = limb_links[0];
        if(limb_lengths.find(edge->head_) != limb_lengths.end()){
            limb_lengths[edge->tail_] = limb_lengths[edge->head_] + 1;
            limb_lengths.erase(edge->head_);
            limb_links.erase(limb_links.begin() + i);
        }else if(limb_lengths.find(edge->tail_) != limb_lengths.end()){
            limb_lengths[edge->head_] = limb_lengths[edge->tail_] + 1;
            limb_lengths.erase(edge->tail_);
            limb_links.erase(limb_links.begin() + i);
        }else{
            i++;
            if(i >= limb_links.size())
                i = 0;
        }
    }
    desc = limb_lengths.begin()->second;
    for(const auto &limb_length: limb_lengths){
        if(limb_length.second > desc)
            desc = limb_length.second;
    }

    return desc;
};


descriptor_fct RoboGrammarDescriptors::min_limbs_length = [](const rg::Graph& graph) -> double{
    if(number_of_limbs(graph) == 0)
        return 0;
    double desc = 0;
    //First find the nodes label with limb mount
    std::map<int,double> limb_lengths;
    for(size_t i = 0; i < graph.nodes_.size(); i++){
        if(graph.nodes_[i].attrs_.label_ == "limb_mount")
            limb_lengths[i] = 1;
    }
    std::vector<const rg::Edge *> limb_links;
    for(const rg::Edge& edge: graph.edges_){
        std::string type;
        std::stringstream label(edge.attrs_.label_);
        std::getline(label,type,'_');
        if(type == "limb"){
            limb_links.push_back(&edge);
        }
    }

    //Then count with edges how many nodes are connect to these
    size_t i = 0;
    while(limb_links.size() > 0){
        const rg::Edge *edge = limb_links[0];
        if(limb_lengths.find(edge->head_) != limb_lengths.end()){
            limb_lengths[edge->tail_] = limb_lengths[edge->head_] + 1;
            limb_lengths.erase(edge->head_);
            limb_links.erase(limb_links.begin() + i);
        }else if(limb_lengths.find(edge->tail_) != limb_lengths.end()){
            limb_lengths[edge->head_] = limb_lengths[edge->tail_] + 1;
            limb_lengths.erase(edge->tail_);
            limb_links.erase(limb_links.begin() + i);
        }else{
            i++;
            if(i >= limb_links.size())
                i = 0;
        }
    }
    desc = limb_lengths.begin()->second;
    for(const auto &limb_length: limb_lengths){
        if(limb_length.second < desc)
            desc = limb_length.second;
    }

    return desc;
};


descriptor_fct RoboGrammarDescriptors::median_limbs_length = [](const rg::Graph& graph) -> double{
    if(number_of_limbs(graph) == 0)
        return 0;
    double desc = 0;
    //First find the nodes label with limb mount
    std::map<int,double> limb_lengths;
    for(size_t i = 0; i < graph.nodes_.size(); i++){
        if(graph.nodes_[i].attrs_.label_ == "limb_mount")
            limb_lengths[i] = 1;
    }
    std::vector<const rg::Edge *> limb_links;
    for(const rg::Edge& edge: graph.edges_){
        std::string type;
        std::stringstream label(edge.attrs_.label_);
        std::getline(label,type,'_');
        if(type == "limb"){
            limb_links.push_back(&edge);
        }
    }

    //Then count with edges how many nodes are connect to these
    size_t i = 0;
    while(limb_links.size() > 0){
        const rg::Edge *edge = limb_links[0];
        if(limb_lengths.find(edge->head_) != limb_lengths.end()){
            limb_lengths[edge->tail_] = limb_lengths[edge->head_] + 1;
            limb_lengths.erase(edge->head_);
            limb_links.erase(limb_links.begin() + i);
        }else if(limb_lengths.find(edge->tail_) != limb_lengths.end()){
            limb_lengths[edge->head_] = limb_lengths[edge->tail_] + 1;
            limb_lengths.erase(edge->tail_);
            limb_links.erase(limb_links.begin() + i);
        }else{
            i++;
            if(i >= limb_links.size())
                i = 0;
        }
    }
    std::vector<double> lengths;
    for(const auto &limb_length: limb_lengths)
        lengths.push_back(limb_length.second);
    std::sort(lengths.begin(),lengths.end());
    if(lengths.size()%2 == 0){
        size_t mid = lengths.size()/2;
        desc = (lengths[mid] + lengths[mid-1])/2.0;
    }
    else{
        size_t mid = std::round(static_cast<float>(lengths.size()/2));
        desc = lengths[mid];
    }

    return desc;
};
