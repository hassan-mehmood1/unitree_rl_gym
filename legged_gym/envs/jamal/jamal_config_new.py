from legged_gym.envs.base.legged_robot_config import LeggedRobotCfg, LeggedRobotCfgPPO

class JAMALRoughCfg(LeggedRobotCfg):
    class init_state(LeggedRobotCfg.init_state):
        pos = [0.0, 0.0, 0.36]
        default_joint_angles = {
            'FL_hip_joint':   0.10,
            'RL_hip_joint':   0.10,
            'FR_hip_joint': -0.10,
            'RR_hip_joint': -0.10,
            
            'FL_thigh_joint': 0.793,  
            'RL_thigh_joint': -0.793,
            'FR_thigh_joint': 0.793,  
            'RR_thigh_joint': -0.793,
            
            'FL_calf_joint':  -1.509, 
            'RL_calf_joint':   1.509,
            'FR_calf_joint': -1.508,  
            'RR_calf_joint': 1.509,
        }

    class control(LeggedRobotCfg.control):
        control_type = 'P'
        stiffness = {'joint': 60.0}
        damping   = {'joint': 1.0}     # ↑ damping to kill oscillations
        action_scale = 0.15            # gentler actions for quiet stance
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
        # ang_vel_yaw = [0, 0]
        # heading_command = False
        # resampling_time = 1000.0  # very infrequent resampling (effectively constant)

    class rewards(LeggedRobotCfg.rewards):
        soft_dof_pos_limit = 0.9
        # set the base height target
        base_height_target = 0.35
        max_contact_force = 110
        # /////////////////////////////Added for foot height clearence /////////////////////////////
        swing_clearance_target = 0.07   # 6 cm target swing height (flat ground)
        swing_clearance_max    = 0.10   # start penalizing lift above 10 cm
        # /////////////////////////////////////////////////////////////////////////////////////////

        class scales(LeggedRobotCfg.rewards.scales):
            torques = -0.00003
            dof_pos_limits = -10.0

            # //////////////////new rewards////////////////
            # Height / posture stabilizers
            # to track base height we need to add negative reward to base_height
            # base_height = -10.0         # was -0.; enable it
            # orientation = -1.0         # was -0.; penalize bad orientation (pitch/roll/yaw err)
            # action_rate = -0.02        # was -0.01; slow action changes
            # termination = -0.05         # was -0.0; make falling clearly bad
            # Gait shaping
            feet_air_time = 1.5      # was +1.0; reduce a lot to avoid pogo
            # to enable movement in z axis set lin_vel_z to 0 otherwise negative
            # lin_vel_z = -0.005
            # stand_still = -0.01
            # tracking_ang_vel = 0.0
            stumble = -1.0

            # /////////////////////////////Added for foot height clearence /////////////////////////////
            swing_clearance = 0.07   # small; remember scales are multiplied by dt
            # /////////////////////////////////////////////////////////////////////




class JAMALRoughCfgPPO(LeggedRobotCfgPPO):
    class algorithm(LeggedRobotCfgPPO.algorithm):
        entropy_coef = 0.01  # small exploration; stance is simple

    class runner(LeggedRobotCfgPPO.runner):
        run_name = ''
        experiment_name = 'jamal_inverted_no_hip_movement'
    class policy:
        actor_hidden_dims = [128, 64, 32]
        critic_hidden_dims = [128, 64, 32]
        activation = 'elu' # can be elu, relu, selu, crelu, lrelu, tanh, sigmoid
