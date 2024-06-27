# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from omni.isaac.lab.utils import configclass

from .rough_env_cfg import RbPointLegRoughEnvCfg


@configclass
class RbPointLegFlatEnvCfg(RbPointLegRoughEnvCfg):
    def __post_init__(self):
        assert NotImplementedError
        # post init of parent
        super().__post_init__()
        # rewards
        # self.rewards.flat_orientation_l2.weight = -2.5
        # self.rewards.feet_air_time.weight = 2.0
        # self.rewards.joint_deviation_roll.params["asset_cfg"].joint_names = [".*_ROLL_joint"]
        # change terrain to flat
        self.scene.terrain.terrain_type = "plane"
        self.scene.terrain.terrain_generator = None
        # no height scan
        self.scene.height_scanner = None
        self.observations.policy.height_scan = None
        # no terrain curriculum
        self.curriculum.terrain_levels = None


class RbPointLegFlatEnvCfg_PLAY(RbPointLegRoughEnvCfg):
    def __post_init__(self) -> None:
        # post init of parent
        super().__post_init__()

        # make a smaller scene for play
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        # disable randomization for play
        self.observations.policy.enable_corruption = False
