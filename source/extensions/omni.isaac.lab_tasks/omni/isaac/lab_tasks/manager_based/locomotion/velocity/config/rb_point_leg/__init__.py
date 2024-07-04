# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import gymnasium as gym

from . import agents, flat_env_cfg, rough_env_cfg

##
# Register Gym environments.
##



##
# Rough Env
##

gym.register(
    id="Rb-Point-Leg-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": rough_env_cfg.RbPointLegRoughEnvCfg,
        "rsl_rl_cfg_entry_point": agents.rsl_rl_cfg.RbPointLegRoughPPORunnerCfg,
        # "skrl_cfg_entry_point": f"{agents.__name__}:skrl_rough_ppo_cfg.yaml",
    },
)

gym.register(
    id="Rb-Point-Leg-Play-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": rough_env_cfg.RbPointLegRoughEnvCfg_PLAY,
        "rsl_rl_cfg_entry_point": agents.rsl_rl_cfg.RbPointLegRoughPPORunnerCfg,
        # "skrl_cfg_entry_point": f"{agents.__name__}:skrl_rough_ppo_cfg.yaml",
    },
)


gym.register(
    id="Rb-Point-Leg-Play-Env-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": rough_env_cfg.RbPointLegRoughEnvCfg_PLAY_Env,
        "rsl_rl_cfg_entry_point": agents.rsl_rl_cfg.RbPointLegRoughPPORunnerCfg,
        # "skrl_cfg_entry_point": f"{agents.__name__}:skrl_rough_ppo_cfg.yaml",
    },
)

 
##
# Flat Env
##

gym.register(
    id="Rb-Point-Leg-Flat-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": flat_env_cfg.RbPointLegFlatEnvCfg,
        "rsl_rl_cfg_entry_point": agents.rsl_rl_cfg.RbPointLegFlatPPORunnerCfg,
        # "skrl_cfg_entry_point": f"{agents.__name__}:skrl_rough_ppo_cfg.yaml",
    },
)

gym.register(
    id="Rb-Point-Leg-Flat-Play-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": flat_env_cfg.RbPointLegFlatEnvCfg_PLAY,
        "rsl_rl_cfg_entry_point": agents.rsl_rl_cfg.RbPointLegFlatPPORunnerCfg,
        # "skrl_cfg_entry_point": f"{agents.__name__}:skrl_rough_ppo_cfg.yaml",
    },
)

gym.register(
    id="Rb-Point-Leg-Flat-Play-Env-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": flat_env_cfg.RbPointLegFlatEnvCfg_PLAY_Env,
        "rsl_rl_cfg_entry_point": agents.rsl_rl_cfg.RbPointLegFlatPPORunnerCfg,
        # "skrl_cfg_entry_point": f"{agents.__name__}:skrl_rough_ppo_cfg.yaml",
    },
)