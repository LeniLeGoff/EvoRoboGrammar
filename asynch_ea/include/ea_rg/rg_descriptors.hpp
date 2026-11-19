#pragma once

#include <functional>
#include <robot_design/graph.h>
#define GET_VARIABLE_NAME(Variable) (#Variable)

namespace ea_rg{

namespace rg = robot_design;

using descriptor_fct = std::function<double(const rg::Graph&)>;

struct RoboGrammarDescriptors{
    static descriptor_fct number_of_parts;
    static descriptor_fct number_of_limbs;
    static descriptor_fct median_limbs_length;
    static descriptor_fct max_limbs_length;
    static descriptor_fct min_limbs_length;
    static descriptor_fct length_of_body;
    static descriptor_fct number_of_body_swing_joints;
    static descriptor_fct number_of_body_roll_joints;
    static descriptor_fct number_of_body_lift_joints;
    static descriptor_fct number_of_body_fixed_joints;
    static descriptor_fct number_of_limb_swing_joints;
    static descriptor_fct number_of_limb_lift_joints;
    static descriptor_fct number_of_limb_roll_joints;

    static std::string to_string(const std::vector<double>&);

    void use_all_descriptors();
    void set_descriptors(const std::vector<descriptor_fct> &);
    std::vector<double> compute_descriptor_vector(const robot_design::Graph &graph);
    std::vector<std::string> descriptor_names;
private:
    std::vector<descriptor_fct> descriptors;

};

}
