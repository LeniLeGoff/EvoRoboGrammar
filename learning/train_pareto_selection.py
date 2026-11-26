import sys
import os
import pyrobotdesign as rd
import py_async_ea as aea
import arguments as rg_args
import torch
from sac import SAC
import multiprocessing

def comparator(a, b):
    return a.get_objectives()[0] < b.get_objectives()[0]

def distance(a, b):
    return rd.tree_edit_distance(a.get_morph_genome().get_graph(), b.get_morph_genome().get_graph())

def descriptors_fct(a):
    rgdesc = aea.RoboGrammarDescriptors()
    rgdesc.use_all_descriptors()
    return rgdesc.compute_descriptor_vector(a.get_morph_genome().get_graph())

if __name__ == "__main__":
    args = rg_args.parse_args(sys.argv)
    parameters = {
        "env_name": "RobotLocomotion-v0",
        "task": "FlatTerrainTask",
        "rule_sequence": args.rule_sequence,
        "grammar_file": args.grammar_file,
        "exp_name": "sac_forward_walking",
        "log_dir": "sac_logs",
        "hidden_dim": [512, 256, 128],
        "log_interval": 1,
        "batch_size": 100,
        "frames_per_batch": 1000,
        "num_steps":1024,
        "num_processes": 24,
        "lr": 3e-4,
        "eps": 0.995,
        "parallel": True,
        }
    archive_type = args.archive_type
    
    is_fork = multiprocessing.get_start_method() == "fork"
    device = (
    torch.device(0)
    if torch.cuda.is_available() and not is_fork
    else torch.device("cpu"))

    #load archive
    archive = None
    if(archive_type == "grid"):
        archive = aea.InfiniteGridArchive([1]*13,comparator,descriptors_fct,aea.selectors.random)
        aea.load_infinite_grid_archive(archive,args.log_folder,args.grammar_file)
    elif(archive_type == "unstructured"):
        archive = aea.UnstructuredArchive(1,distance,comparator,aea.selectors.random,5)
        aea.load_unstructured_archive(archive,args.log_folder,args.grammar_file)

    archive.compute_sparsity_score(5)
    selection = aea.selectors.pareto(archive.get_solutions())

    for robot in selection:
        print("Selected solution: ", robot.id(), " fitness: ", robot.get_objectives()[0], " sparsity: ", robot.get_sparsity_score(), " genome: ", "".join("{},{} ".format(gene[0], gene[1]) for gene in robot.get_morph_genome().get_rule_seq()))

    print("{} robots selected from an archive with {} robots".format(len(selection),len(archive.get_solutions())))


    # for robot in selection:
    #     sac = SAC(parameters, device,robot.get_morph_genome().get_graph())
    #     sac.train()
