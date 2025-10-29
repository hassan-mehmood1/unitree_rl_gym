from legged_gym.envs.base.legged_robot_config import LeggedRobotCfg, LeggedRobotCfgPPO

class JAMALRoughCfg(LeggedRobotCfg):
    class init_state(LeggedRobotCfg.init_state):
        pos = [0.0, 0.0, 0.35]
        default_joint_angles = {
            'FL_hip_joint':  0.10, 'FR_hip_joint': -0.10,
            'RL_hip_joint':  0.10, 'RR_hip_joint': -0.10,
            'FL_thigh_joint': 0.785, 'FR_thigh_joint': 0.785,
            'RL_thigh_joint': 0.90,  'RR_thigh_joint': 0.90,
            'FL_calf_joint': -1.35,  'FR_calf_joint': -1.35,
            'RL_calf_joint': -1.35,  'RR_calf_joint': -1.35,
        }

    class control(LeggedRobotCfg.control):
        control_type = 'P'
        stiffness = {'joint': 80.0}
        damping   = {'joint': 2.0}     # ↑ damping to kill oscillations
        action_scale = 0.4            # gentler actions for quiet stance
        decimation = 4

    class asset(LeggedRobotCfg.asset):
        file = '{LEGGED_GYM_ROOT_DIR}/resources/robots/jamal/urdf/jamal2_v5.urdf'
        name = "jamal"
        foot_name = "foot"
        penalize_contacts_on = ["thigh","calf"]
        terminate_after_contacts_on = ["base"]
        self_collisions = 1

    # Freeze commands to zero so the task is "do not move"
    class commands(LeggedRobotCfg.commands):
        # Set all command ranges to zero
        lin_vel_x = [0.0, 0.0]
        lin_vel_y = [0.0, 0.0]
        ang_vel_yaw = [0.0, 0.0]
        heading_command = False
        resampling_time = 1000.0  # very infrequent resampling (effectively constant)

    class rewards(LeggedRobotCfg.rewards):
        base_height_target = 0.33
        soft_dof_pos_limit = 0.9

        class scales(LeggedRobotCfg.rewards.scales):
            # Posture & stability
            torques = -0.0002
            dof_pos_limits = -0.2

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
