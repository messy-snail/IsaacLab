# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from omni.isaac.lab.managers import RewardTermCfg as RewTerm
from omni.isaac.lab.managers import SceneEntityCfg
from omni.isaac.lab.utils import configclass

import omni.isaac.lab_tasks.manager_based.locomotion.velocity.mdp as mdp
from omni.isaac.lab_tasks.manager_based.locomotion.velocity.velocity_env_cfg import (
    LocomotionVelocityRoughEnvCfg,
    MySceneCfg,
    RewardsCfg,
)

##
# Pre-defined configs
##
from omni.isaac.lab_assets.rbq10 import RBQ10_CFG


@configclass
class RbQ10RewardsCfg(RewardsCfg):
    termination_penalty = RewTerm(func=mdp.is_terminated, weight=-200.0)
    # track_lin_vel_xy_exp = RewTerm(
    #     func=mdp.track_lin_vel_xy_yaw_frame_exp,
    #     weight=1.0,
    #     params={"command_name": "base_velocity", "std": 0.5},
    # )
    # track_ang_vel_z_exp = RewTerm(
    #     func=mdp.track_ang_vel_z_world_exp, weight=2.0, params={"command_name": "base_velocity", "std": 0.5}
    # )
    
    # joint_deviation_roll = RewTerm(
    #     func=mdp.joint_deviation_l1,
    #     weight=-0.2,
    #     params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*_ROLL_joint"])},
    # )
    feet_air_time = RewTerm(
        func=mdp.feet_air_time_positive_biped,
        weight=0.125,
        params={
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=[".*_foot"]),
            "command_name": "base_velocity",
            "threshold": 0.5,
        },
    )
    # rewards
    undesired_contacts = RewTerm(
            func=mdp.undesired_contacts,
            weight=-1.0,
            params={"sensor_cfg": SceneEntityCfg("contact_forces", body_names=[".*thigh"]), "threshold": 1.0},
    )


@configclass
class RbQ10RoughEnvCfg(LocomotionVelocityRoughEnvCfg):
    """RbPointLeg rough environment configuration."""

    rewards:RbQ10RewardsCfg = RbQ10RewardsCfg()

    def __post_init__(self):
        super().__post_init__()
        # scene
        self.scene.robot = RBQ10_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        self.scene.height_scanner.prim_path = "{ENV_REGEX_NS}/Robot/fuselage"

        # actions
        self.actions.joint_pos.scale = 0.5

        # events
        self.events.push_robot = None
        self.events.add_base_mass = None
        self.events.reset_robot_joints.params["position_range"] = (1.0, 1.0)
        self.events.base_external_force_torque.params["asset_cfg"].body_names = ["fuselage", ".*_hip"]
        self.events.reset_base.params = {
            "pose_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5), "yaw": (-3.14, 3.14)},
            "velocity_range": {
                "x": (0.0, 0.0),
                "y": (0.0, 0.0),
                "z": (0.0, 0.0),
                "roll": (0.0, 0.0),
                "pitch": (0.0, 0.0),
                "yaw": (0.0, 0.0),
            },
        }

        # terminations
        self.terminations.base_contact.params["sensor_cfg"].body_names = ["fuselage"]

        self.rewards.feet_air_time.weight = 0.01
        self.rewards.dof_torques_l2.weight = -0.0002
        self.rewards.track_lin_vel_xy_exp.weight = 1.5
        self.rewards.track_ang_vel_z_exp.weight = 0.75
        self.rewards.dof_acc_l2.weight = -2.5e-7
        # self.rewards.dof_torques_l2.weight = -5.0e-6
        # self.rewards.track_lin_vel_xy_exp.weight = 2.0
        # self.rewards.track_ang_vel_z_exp.weight = 1.0
        # self.rewards.action_rate_l2.weight *= 1.5
        # self.rewards.dof_acc_l2.weight *= 1.5
        # self.commands.base_velocity.ranges.lin_vel_x = (-2.0, 2.0)
        # self.commands.base_velocity.ranges.lin_vel_y = (0.0, 0.0)
        # self.commands.base_velocity.ranges.ang_vel_z = (-1.0, 1.0)


@configclass
class RbQ10RoughEnvCfg_PLAY(RbQ10RoughEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # make a smaller scene for play
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        # spawn the robot randomly in the grid (instead of their terrain levels)
        self.scene.terrain.max_init_terrain_level = None
        # reduce the number of terrains to save memory
        if self.scene.terrain.terrain_generator is not None:
            self.scene.terrain.terrain_generator.num_rows = 5
            self.scene.terrain.terrain_generator.num_cols = 5
            self.scene.terrain.terrain_generator.curriculum = False

        self.commands.base_velocity.ranges.lin_vel_x = (0.7, 1.0)
        self.commands.base_velocity.ranges.lin_vel_y = (0.0, 0.0)
        self.commands.base_velocity.ranges.heading = (0.0, 0.0)
        # disable randomization for play
        self.observations.policy.enable_corruption = False

from omni.isaac.lab.utils.assets import ISAAC_NUCLEUS_DIR, ISAACLAB_NUCLEUS_DIR
@configclass
class RbQ10RoughEnvCfg_PLAY_Env(RbQ10RoughEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()
         # make a smaller scene for play
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        # spawn the robot randomly in the grid (instead of their terrain levels)
        # self.scene.terrain.max_init_terrain_level = None
        # self.curriculum.terrain_levels = None
        
        self.commands.base_velocity.ranges.lin_vel_x = (0.7, 1.0)
        self.commands.base_velocity.ranges.lin_vel_y = (0.0, 0.0)
        self.commands.base_velocity.ranges.heading = (0.0, 0.0)
        # disable randomization for play
        self.observations.policy.enable_corruption = False
        # max_init_terrain_level
        # self.scene.terrain.terrain_generator = None
        self.scene.terrain.terrain_type = "usd"
        # C:\Users\snail\Desktop\env_assets
        self.scene.terrain.usd_path = f"C:/Users/snail/Desktop/env_assets/Simple_Warehouse/warehouse.usd"