#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>

#include "apear/algorithms/archives.hpp"
#include "ea_rg/rg_genome.hpp"
#include "mehk_qd/mehk_qd.hpp"

namespace py = pybind11;
using namespace apear;
using namespace ea_rg;

using UArchive = UnstructuredArchive<RoboGrammarInd::Ptr,int>;
using selector_t = apear::selectors::selector_t<RoboGrammarInd::Ptr>;

PYBIND11_MODULE(py_async_ea, m, py::mod_gil_not_used()){
    py::class_<RoboGrammarGenome,std::shared_ptr<RoboGrammarGenome>>(m,"RoboGrammarGenome")
        .def(py::init<>())
        .def("get_graph",&RoboGrammarGenome::get_graph);

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
    m.def("load_archive",&ea_rg::load_archive<RoboGrammarInd>);

    py::class_<ea_rg::selectors>(m,"selectors")
        .def_readonly_static("random",&ea_rg::selectors::random)
        .def_readonly_static("sparsity",&ea_rg::selectors::sparsity)
        .def_readonly_static("pareto_tournament",&ea_rg::selectors::pareto_tournament);
}
