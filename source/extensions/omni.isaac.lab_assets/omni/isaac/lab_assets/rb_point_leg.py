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

RB_POINT_LEG_CFG = ArticulationCfg(

#C:\Users\snail\Desktop\rb-point-2legs\urdf\RB_POINT_2_LEGS_FINAL
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"C:/Users/snail/Desktop/rb-point-2legs/urdf/RB_POINT_2_LEGS_FINAL/RB_POINT_2_LEGS_FINAL.usd",
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
        pos=(0.0, 0.0, 1.),
        joint_pos={
            '.*L_ROLL_joint': 0.,
            '.*L_HIP_joint': 0.697, # +40 deg
            '.*L_KNEE_joint': -1.394, # -80 deg
            
            '.*R_ROLL_joint': 0.,
            '.*R_HIP_joint': 0.697, 
            '.*R_KNEE_joint': -1.394            
        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,
    actuators={
        "legs": ImplicitActuatorCfg(
            joint_names_expr=[".*_ROLL_joint", ".*_HIP_joint", ".*_KNEE_joint"],
            # joint_names_expr=[".*"],
            effort_limit=200.0,
            velocity_limit=10.0,
            stiffness = {   
                '.*_ROLL_joint': 100., 
                '.*_HIP_joint': 100, 
                '.*_KNEE_joint': 100,
            },
            damping={
                '.*_ROLL_joint': 3., 
                '.*_HIP_joint': 3., 
                '.*_KNEE_joint': 3.,
            },
        ),
    },
)

RB_POINT_LEG_STANDING_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"C:/Users/snail/Desktop/rb-point-2legs/urdf/RB_POINT_2_LEGS_FINAL/RB_POINT_2_LEGS_FINAL.usd",
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
        pos=(0.0, 0.0, 0.1),
        joint_pos={
            '.*ROLL_joint': 0.524,
            '.*HIP_joint': 1.31,
            '.*KNEE_joint': -2.76,
        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,
    actuators={
        "legs": ImplicitActuatorCfg(
            joint_names_expr=[".*_ROLL_joint", ".*_HIP_joint", ".*_KNEE_joint"],
            # joint_names_expr=[".*"],
            effort_limit=200.0,
            velocity_limit=10.0,
            stiffness = {   
                '.*_ROLL_joint': 100., 
                '.*_HIP_joint': 100, 
                '.*_KNEE_joint': 100,
            },
            damping={
                '.*_ROLL_joint': 3., 
                '.*_HIP_joint': 3., 
                '.*_KNEE_joint': 3.,
            },
        ),
    },
)
