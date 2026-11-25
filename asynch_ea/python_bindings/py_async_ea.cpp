#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>

#include "apear/algorithms/archives.hpp"
#include "ea_rg/rg_genome.hpp"
#include "ea_rg/rg_descriptors.hpp"
#include "mehk_qd/mehk_qd.hpp"
#include "mehk_qd/archive_loading_fcts.hpp"

namespace py = pybind11;
using namespace apear;
using namespace ea_rg;

using UArchive = UnstructuredArchive<RoboGrammarInd::Ptr,int>;
using GArchive = InfiniteGridArchive<RoboGrammarInd::Ptr>;
using selector_t = apear::selectors::selector_t<RoboGrammarInd::Ptr>;

PYBIND11_MODULE(py_async_ea, m, py::mod_gil_not_used()){
    py::class_<RoboGrammarGenome,std::shared_ptr<RoboGrammarGenome>>(m,"RoboGrammarGenome")
        .def(py::init<>())
        .def("get_graph",&RoboGrammarGenome::get_graph)
        .def("get_rule_seq",&RoboGrammarGenome::get_rule_seq);

    py::class_<RoboGrammarInd,std::shared_ptr<RoboGrammarInd>>(m,"RoboGrammarInd")
        .def(py::init<>())
        .def(py::init<RoboGrammarGenome::Ptr,apear::Genome::Ptr>())
        .def("get_sparsity_score",&RoboGrammarInd::get_sparsity_score)
        .def("get_objectives",&RoboGrammarInd::get_objectives)
        .def("id",&RoboGrammarInd::id)
        .def("get_morph_genome",&RoboGrammarInd::get_morph_genome);

    py::class_<UArchive>(m,"UnstructuredArchive")
        .def(py::init<>())
        .def(py::init<int,UArchive::distance_fct_t,UArchive::comparator_t,selector_t,int>())
        .def("compute_sparsity_score",&UArchive::compute_sparsity_scores)
        .def("get_solutions",&UArchive::get_solutions);

    py::class_<GArchive>(m,"InfiniteGridArchive")
        .def(py::init<>())
        .def(py::init<const std::vector<double>&,GArchive::comparator_t, GArchive::descriptor_t,
                      selector_t>())
        .def("get_solutions",&UArchive::get_solutions);

    m.def("load_unstructured_archive",&ea_rg::load_unstructured_archive<RoboGrammarInd>);
    m.def("load_infinite_grid_archive",&ea_rg::load_infinite_grid_archive<RoboGrammarInd>);

    py::class_<ea_rg::selectors>(m,"selectors")
        .def_readonly_static("random",&ea_rg::selectors::random)
        .def_readonly_static("sparsity",&ea_rg::selectors::sparsity)
        .def_readonly_static("pareto_tournament",&ea_rg::selectors::pareto_tournament)
        .def_readonly_static("pareto",&ea_rg::selectors::pareto<RoboGrammarInd::Ptr>);
    py::class_<ea_rg::RoboGrammarDescriptors>(m,"RoboGrammarDescriptors")
        .def(py::init<>())
        .def("use_all_descriptors",&ea_rg::RoboGrammarDescriptors::use_all_descriptors)
        .def("compute_descriptor_vector",&ea_rg::RoboGrammarDescriptors::compute_descriptor_vector)
        .def_readonly("descriptor_names",&ea_rg::RoboGrammarDescriptors::descriptor_names)
        .def_readonly_static("number_of_parts",&ea_rg::RoboGrammarDescriptors::number_of_parts)
        .def_readonly_static("number_of_limbs",&ea_rg::RoboGrammarDescriptors::number_of_limbs)
        .def_readonly_static("median_limbs_length",&ea_rg::RoboGrammarDescriptors::median_limbs_length)
        .def_readonly_static("max_limbs_length",&ea_rg::RoboGrammarDescriptors::max_limbs_length)
        .def_readonly_static("min_limbs_length",&ea_rg::RoboGrammarDescriptors::min_limbs_length)
        .def_readonly_static("length_of_body",&ea_rg::RoboGrammarDescriptors::length_of_body)
        .def_readonly_static("number_of_body_swing_joints",&ea_rg::RoboGrammarDescriptors::number_of_body_swing_joints)
        .def_readonly_static("number_of_body_roll_joints",&ea_rg::RoboGrammarDescriptors::number_of_body_roll_joints)
        .def_readonly_static("number_of_body_lift_joints",&ea_rg::RoboGrammarDescriptors::number_of_body_lift_joints)
        .def_readonly_static("number_of_body_fixed_joints",&ea_rg::RoboGrammarDescriptors::number_of_body_fixed_joints)
        .def_readonly_static("number_of_limb_swing_joints",&ea_rg::RoboGrammarDescriptors::number_of_limb_swing_joints)
        .def_readonly_static("number_of_limb_lift_joints",&ea_rg::RoboGrammarDescriptors::number_of_limb_lift_joints)
        .def_readonly_static("number_of_limb_roll_joints",&ea_rg::RoboGrammarDescriptors::number_of_limb_roll_joints);

}
