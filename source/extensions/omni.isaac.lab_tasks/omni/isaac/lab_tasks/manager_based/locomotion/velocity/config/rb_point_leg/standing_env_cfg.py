# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from omni.isaac.lab.utils import configclass

from .flat_env_cfg import RbPointLegFlatEnvCfg


@configclass
class RbPointLegStandingEnvCfg(RbPointLegFlatEnvCfg):
    def __post_init__(self):
        
        # post init of parent
        super().__post_init__()
        self.rewards.feet_air_time = None
        # self.rewards.joint_deviation_roll = None
        # # rewards
        # self.rewards.flat_orientation_l2.weight = -2.5
        # self.rewards.feet_air_time.weight = 5.0
        # self.rewards.joint_deviation_hip.params["asset_cfg"].joint_names = ["hip_rotation_.*"]
        # # change terrain to flat
        # self.scene.terrain.terrain_type = "plane"
        # self.scene.terrain.terrain_generator = None
        # # no height scan
        # self.scene.height_scanner = None
        # self.observations.policy.height_scan = None
        # # no terrain curriculum
        # self.curriculum.terrain_levels = Nones


class RbPointLegStaingEnvCfg_PLAY(RbPointLegStandingEnvCfg):
    def __post_init__(self) -> None:
        # post init of parent
        super().__post_init__()

        # make a smaller scene for play
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        # disable randomization for play
        self.observations.policy.enable_corruption = False