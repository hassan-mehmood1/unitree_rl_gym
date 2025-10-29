from legged_gym.envs.base.legged_robot_config import LeggedRobotCfg, LeggedRobotCfgPPO

class JAMALRoughCfg(LeggedRobotCfg):
    class init_state(LeggedRobotCfg.init_state):
        pos = [0.0, 0.0, 0.35]
        default_joint_angles = {
            'FL_hip_joint':   0.10,
            'RL_hip_joint':   0.10,
            'FR_hip_joint': -0.10,
            'RR_hip_joint': -0.10,
            
            'FL_thigh_joint': 0.87,  
            'RL_thigh_joint': -0.87,
            'FR_thigh_joint': 0.87,  
            'RR_thigh_joint': -0.87,
            
            'FL_calf_joint':  -1.57, 
            'RL_calf_joint':   1.57,
            'FR_calf_joint': -1.57,  
            'RR_calf_joint': 1.57,
        }

    class control(LeggedRobotCfg.control):
        control_type = 'P'
        stiffness = {'joint': 80.0}
        damping   = {'joint': 2.0}     # ↑ damping to kill oscillations
        action_scale = 0.25            # gentler actions for quiet stance
        decimation = 4

    class asset(LeggedRobotCfg.asset):
        file = '{LEGGED_GYM_ROOT_DIR}/resources/robots/jamal/urdf/jamal_inverted.urdf'
        name = "jamal"
        foot_name = "foot"
        penalize_contacts_on = ["thigh","calf"]
        terminate_after_contacts_on = ["base"]
        self_collisions = 1
        # default_dof_drive_mode = 0 # see GymDofDriveModeFlags (0 is none, 1 is pos tgt, 2 is vel tgt, 3 effort)
        
    # Freeze commands to zero so the task is "do not move"
    # class commands(LeggedRobotCfg.commands):
        # Set all command ranges to zero
        # lin_vel_x = [0.0, 0.0]
        # lin_vel_y = [0.0, 0.0]
        # ang_vel_yaw = [0.0, 0.0]
        # heading_command = False
        # resampling_time = 1000.0  # very infrequent resampling (effectively constant)

    class rewards(LeggedRobotCfg.rewards):
        soft_dof_pos_limit = 0.9
        base_height_target = 0.3

        class scales(LeggedRobotCfg.rewards.scales):
            torques = -0.0002
            dof_pos_limits = -15.0


class JAMALRoughCfgPPO(LeggedRobotCfgPPO):
    class algorithm(LeggedRobotCfgPPO.algorithm):
        entropy_coef = 0.01  # small exploration; stance is simple

    class runner(LeggedRobotCfgPPO.runner):
        run_name = ''
        experiment_name = 'rough_jamal'
    # class policy:
    #     actor_hidden_dims = [128, 64, 32]
    #     critic_hidden_dims = [128, 64, 32]
    #     activation = 'elu' # can be elu, relu, selu, crelu, lrelu, tanh, sigmoid
