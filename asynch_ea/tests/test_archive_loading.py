from py_async_ea import *
from pyrobotdesign import tree_edit_distance
import sys

def comparator(a, b):
    return a.get_objectives()[0] < b.get_objectives()[0]

def distance(a, b):
    return tree_edit_distance(a.get_morph_genome().get_graph(), b.get_morph_genome().get_graph())

archive = UnstructuredArchive(1,distance,comparator,selectors.pareto_tournament,5)

load_archive(archive, sys.argv[2], sys.argv[1])

archive.compute_sparsity_score(5)

solutions = archive.get_solutions()


for solution in solutions:
    print("Solution ID: {} Sparsity Score: {} Fitness Score {}".format(solution.id(), solution.get_sparsity_score(), solution.get_objectives()[0]))

print("Number of Solutions in the archive : {}".format(len(solutions)))

