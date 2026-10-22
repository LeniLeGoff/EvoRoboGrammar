from torchrl.envs.libs.gym import GymEnv
from torchrl.envs.utils import set_exploration_type, ExplorationType
from torchrl.checkpoint import Checkpoint
from torchrl.modules import Actor
from tensordict import TensorDict
import arguments as rg_args
from utils import convert_rule_pairs_to_list,string_to_rule_pairs
import multiprocessing
import torch
import sys
import time
import environments as rg_env
import sac
from tensordict.nn import TensorDictModule

is_fork = multiprocessing.get_start_method() == "fork"
device = (torch.device(0) 
          if torch.cuda.is_available() and not is_fork
          else torch.device("cpu"))

args = rg_args.parse_args(sys.argv)
parameters = {
    "env_name": "RobotLocomotion-v0",
    "task": "FlatTerrainTask",
    "rule_sequence": args.rule_sequence,
    "grammar_file": args.grammar_file,
    "hidden_dim": [512, 256, 128]
    }
rule_list = convert_rule_pairs_to_list(args.grammar_file,string_to_rule_pairs(args.rule_sequence))
parameters["rule_sequence"] = []
for rule in rule_list:
    parameters["rule_sequence"].append(str(rule) + ",")

env = sac.make_env(parameters,device)

actor_network = sac.make_actor_network(env, parameters["hidden_dim"] ,device)
policy_module = sac.make_policy_module(env, actor_network)
# policy_module = TensorDictModule(actor_network,in_keys=["observation"],out_keys=["action"])
# policy_module = Actor(
#     module=policy_module,
#     spec=env.action_spec,
# )

checkpoint = Checkpoint(policy=policy_module)
checkpoint.load("sac_forward_walking/checkpoints/checkpoint-000001000944",
                components=["policy"])

policy_module.eval()

set_exploration_type(ExplorationType.DETERMINISTIC)
torch.no_grad()
action =policy_module(env.reset())
def callback(a,b):
    env.render()
    time.sleep(0.05)
rollout = env.rollout(10000,policy_module,callback)
print(rollout["next","reward"].max().item())
print(rollout["step_count"].sum().item())
# for i in range(1000):
#     step_td = env.step(action)
#     action = policy_module(step_td)
#     print(f"Step {i}): reward: {step_td['next']['reward']}; action: {action['action']}")
#     env.render()

