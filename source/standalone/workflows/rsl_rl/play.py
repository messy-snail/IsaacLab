# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Script to play a checkpoint if an RL agent from RSL-RL."""

"""Launch Isaac Sim Simulator first."""

import argparse

from omni.isaac.lab.app import AppLauncher

# local imports
import cli_args  # isort: skip

# add argparse arguments
parser = argparse.ArgumentParser(description="Train an RL agent with RSL-RL.")
parser.add_argument("--cpu", action="store_true", default=False, help="Use CPU pipeline.")
parser.add_argument(
    "--disable_fabric", action="store_true", default=False, help="Disable fabric and use USD I/O operations."
)
parser.add_argument("--num_envs", type=int, default=None, help="Number of environments to simulate.")
parser.add_argument("--task", type=str, default=None, help="Name of the task.")
parser.add_argument("--seed", type=int, default=None, help="Seed used for the environment")
# append RSL-RL cli arguments
cli_args.add_rsl_rl_args(parser)
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import gymnasium as gym
import os
import torch
import numpy as np

from rsl_rl.runners import OnPolicyRunner

import omni.isaac.lab_tasks  # noqa: F401
from omni.isaac.lab_tasks.utils import get_checkpoint_path, parse_env_cfg
from omni.isaac.lab_tasks.utils.wrappers.rsl_rl import (
    RslRlOnPolicyRunnerCfg,
    RslRlVecEnvWrapper,
    export_policy_as_jit,
    export_policy_as_onnx,
)

import pandas as pd
def main():
    """Play with RSL-RL agent."""
    # parse configuration
    env_cfg = parse_env_cfg(
        args_cli.task, use_gpu=not args_cli.cpu, num_envs=args_cli.num_envs, use_fabric=not args_cli.disable_fabric
    )
    agent_cfg: RslRlOnPolicyRunnerCfg = cli_args.parse_rsl_rl_cfg(args_cli.task, args_cli)

    # create isaac environment
    env = gym.make(args_cli.task, cfg=env_cfg)
    # wrap around environment for rsl-rl
    env = RslRlVecEnvWrapper(env)

    # specify directory for logging experiments
    log_root_path = os.path.join("logs", "rsl_rl", agent_cfg.experiment_name)
    log_root_path = os.path.abspath(log_root_path)
    print(f"[INFO] Loading experiment from directory: {log_root_path}")
    resume_path = get_checkpoint_path(log_root_path, agent_cfg.load_run, agent_cfg.load_checkpoint)
    print(f"[INFO]: Loading model checkpoint from: {resume_path}")

    # load previously trained model
    ppo_runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
    ppo_runner.load(resume_path)
    print(f"[INFO]: Loading model checkpoint from: {resume_path}")

    # obtain the trained policy for inference
    policy = ppo_runner.get_inference_policy(device=env.unwrapped.device)

    # export policy to onnx/jit
    export_model_dir = os.path.join(os.path.dirname(resume_path), "exported")
    export_policy_as_jit(
        ppo_runner.alg.actor_critic, ppo_runner.obs_normalizer, path=export_model_dir, filename="policy.pt"
    )
    export_policy_as_onnx(
        ppo_runner.alg.actor_critic, normalizer=ppo_runner.obs_normalizer, path=export_model_dir, filename="policy.onnx"
    )

    # reset environment
    obs, _ = env.get_observations()
    # simulate environment
    csv_file = 'observations.csv'
    csv_file2 = 'actions2.csv'
    # policy = torch.jit.load('flat_policy.pt').to('cuda').eval()
    while simulation_app.is_running():
        # run everything in inference mode
        with torch.inference_mode():
            # agent stepping
            # obs = torch.zeros(env.num_envs, 30).to('cuda')
            # obs[:,9:12] = 0
            actions = policy(obs)
            # actions[:] = torch.tensor([0, 0.697, -1.394, 0, 0.697, -1.394]).to('cuda')
            # actions[:] = torch.tensor([0, 0, -1.394, 0, 0, -1.394]).to('cuda')
            # actions[:] = torch.tensor([0, 0, 0, 0, 0, 0]).to('cuda')
            # env stepping
            obs, _, _, extras = env.step(actions)
            print(f'actions= {np.rad2deg(actions.cpu().numpy())}')
            obs_np = obs.cpu().numpy()
            left_forces = env.unwrapped.scene.sensors['contact_forces'].data.net_forces_w[:,5,2].cpu().numpy().reshape(-1, 1)
            right_forces = env.unwrapped.scene.sensors['contact_forces'].data.net_forces_w[:,9,2].cpu().numpy().reshape(-1, 1)
            obs_np = np.hstack((obs_np, left_forces, right_forces))
            print(f'obs= {np.rad2deg(obs_np[:,12:18])}')
            print(f'err= {np.rad2deg(actions.cpu().numpy()-obs_np[:,12:18])}')
            print(f'err2= {np.rad2deg(obs_np[:,24:30]-obs_np[:,12:18])}')
            print(f'gravity= {obs_np[:,6:9]}')
            # obs_np = np.hstack((obs_np, right_forces))
            # obs_np.hstack(actions)
            
            # df = pd.DataFrame(obs_np)
            # df.to_csv(csv_file, mode='a', header=False, index=False) 
            df = pd.DataFrame(actions.cpu().numpy())
            df.to_csv(csv_file2, mode='a', header=False, index=False) 
            # print(obs_np.shape)
            

    # close the simulator
    env.close()


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()
