from pytest import Collector
from torchrl.envs.libs.gym import GymEnv
from torchrl.envs import (
    Compose,
    DoubleToFloat,
    ObservationNorm,
    StepCounter,
    TransformedEnv,
)
from torchrl.render import make_render_env
from torchrl.record import VideoRecorder
from torchrl.record.loggers.csv import CSVLogger
from tensordict import TensorDict
import arguments as rg_args
from utils import convert_rule_pairs_to_list,string_to_rule_pairs
import multiprocessing
import torch
import sys
import environments as rg_env

is_fork = multiprocessing.get_start_method() == "fork"
device = (torch.device(0) 
          if torch.cuda.is_available() and not is_fork
          else torch.device("cpu"))

args_list = ['--env-name', 'RobotLocomotion-v0',
                '--task', 'FlatTerrainTask',
                '--num-env-steps', '30000000']
parser = rg_args.get_parser()
args = parser.parse_args(args_list + sys.argv[1:])
rule_list = convert_rule_pairs_to_list(args.grammar_file,string_to_rule_pairs(args.rule_sequence))
args.rule_sequence = []
for rule in rule_list:
    args.rule_sequence.append(str(rule) + ",")


base_env = GymEnv("RobotLocomotion-v0",device=device,args=args)

base_env.reset()
for i in range(300):
    act = torch.rand(base_env.action_spec.shape, device=device)
    action = TensorDict({"action": act})
    step_td = base_env.step(action)
    print(f"Step {i}): reward: {step_td['next']['reward']}")
    base_env.render()

