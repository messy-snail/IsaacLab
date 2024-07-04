# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause


import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.actuators import ImplicitActuatorCfg
from omni.isaac.lab.assets.articulation import ArticulationCfg
from omni.isaac.lab.utils.assets import ISAACLAB_NUCLEUS_DIR

##
# Configuration
##

RBQ10_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        # usd_path=r"C:\Users\snail\Desktop\rbq10\urdf\rbq10\rbq10.usd",
        usd_path=r"C:\Users\snail\Desktop\rbq10\urdf\rbq10\rbq10_test.usd",
        # usd_path=f"C:/Users/snail/Desktop/legged_gym/resources/robots/rbl1c/urdf/rbl1c/rbl1c.usd",
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True, solver_position_iteration_count=4, solver_velocity_iteration_count=0
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, .6),
        joint_pos={
            ".*_hip_joint": 0.,
            ".*_thigh_joint": 0,
            ".*_calf_joint": -0.46,
        },


        # joint_pos={
        #     "RR_hip_joint": -0.0033773 ,
        #     "RR_thigh_joint": -0.00385357 ,
        #     "RR_calf_joint": -0.313652 ,
            
        #     "RL_hip_joint": 0.00337856 ,
        #     "RL_thigh_joint": -0.00385292 ,
        #     "RL_calf_joint": -0.313648 ,
            
        #     "FR_hip_joint": -0.00142369 ,
        #     "FR_thigh_joint": -0.00194263,
        #     "FR_calf_joint": -0.299336 ,
            
        #     "FL_hip_joint": 0.00142331 ,
        #     "FL_thigh_joint": -0.00194229 ,
        #     "FL_calf_joint": -0.299338,
        # },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,
    actuators={
        "legs": ImplicitActuatorCfg(
            joint_names_expr=[".*_hip_joint", ".*_thigh_joint", ".*_calf_joint"],
            # joint_names_expr=[".*"],
            # effort_limit=200.0,
            # velocity_limit=10.0,
            # stiffness = {   
            #     '.*_hip_joint': 100., 
            #     '.*_thigh_joint': 100, 
            #     '.*_calf_joint': 100,
            # },
            # damping={
            #     '.*_hip_joint': 3., 
            #     '.*_thigh_joint': 3., 
            #     '.*_calf_joint': 3.,
            # },
            effort_limit=45.0,
            velocity_limit=7.5,
            stiffness=60.0,
            damping=1.5,
            
            # effort_limit=100.0,
            # velocity_limit=10.0,
            # stiffness=300.0,
            # damping=3.0,
        ),
    },
)
