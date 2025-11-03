from legged_gym.envs.base.legged_robot_config import LeggedRobotCfg, LeggedRobotCfgPPO

class JAMALRoughCfg( LeggedRobotCfg ):
    class init_state( LeggedRobotCfg.init_state ):
        pos = [0.0, 0.0, 0.36] # x,y,z [m]
        default_joint_angles = { # = target angles [rad] when action = 0.0
            'FL_hip_joint': 0.1,   # [rad]
            'RL_hip_joint': 0.1,   # [rad]
            'FR_hip_joint': -0.1 ,  # [rad]
            'RR_hip_joint': -0.1,   # [rad]

            'FL_thigh_joint': 0.793,     # [rad]
            'RL_thigh_joint': 0.793,   # [rad]
            'FR_thigh_joint': 0.793,     # [rad]
            'RR_thigh_joint': 0.793,   # [rad]

            'FL_calf_joint': -1.509,   # [rad]
            'RL_calf_joint': -1.509,    # [rad]
            'FR_calf_joint': -1.509,  # [rad]
            'RR_calf_joint': -1.509,    # [rad]
        }

    class control( LeggedRobotCfg.control ):
        # PD Drive parameters:
        control_type = 'P'
        # stiffness = {'joint': 20.}  # [N*m/rad]
        # damping = {'joint': 0.5}     # [N*m*s/rad]
        stiffness = {'joint': 60.0}  # [N*m/rad]
        damping = {'joint': 1.0}     # [N*m*s/rad]
        # action scale: target angle = actionScale * action + defaultAngle
        action_scale = 0.15
        # decimation: Number of control action updates @ sim DT per policy DT
        decimation = 4

    class asset( LeggedRobotCfg.asset ):
        # file = '{LEGGED_GYM_ROOT_DIR}/resources/robots/go2/urdf/go2.urdf'
        file = '{LEGGED_GYM_ROOT_DIR}/resources/robots/jamal/urdf/jamal.urdf'
        name = "jamal"
        foot_name = "foot"
        penalize_contacts_on = ["thigh", "calf"]
        terminate_after_contacts_on = ["base"]
        self_collisions = 1 # 1 to disable, 0 to enable...bitwise filter
  
    class rewards( LeggedRobotCfg.rewards ):
        # /////////////////working rewards////////////////
        soft_dof_pos_limit = 0.9
        base_height_target = 0.35
        # //////////////////new rewards////////////////
        # base_height_target = 0.35      
        # only_positive_rewards = False   # let penalties teach!
        class scales( LeggedRobotCfg.rewards.scales ):
            # /////////////////working rewards////////////////
            torques = -0.0002
            dof_pos_limits = -12.0

            # //////////////////new rewards////////////////
            # Height / posture stabilizers
            # base_height = -5.0         # was -0.; enable it
            # orientation = -0.5         # was -0.; penalize bad orientation (pitch/roll/yaw err)
            # # Effort
            # torques = -0.0002          # was -1e-5; give effort some weight
            # termination = -0.05         # was -0.0; make falling clearly bad
            # # Gait shaping
            # feet_air_time = 1.2      # was +1.0; reduce a lot to avoid pogo
            # # Optional (if your env supports it):
            # dof_pos_limits = -14.0
            
        

class JAMALRoughCfgPPO( LeggedRobotCfgPPO ):
    class algorithm( LeggedRobotCfgPPO.algorithm ):
        entropy_coef = 0.01
    class runner( LeggedRobotCfgPPO.runner ):
        run_name = ''
        experiment_name = 'rough_jamal'

    # class policy:
    #     actor_hidden_dims = [128, 64, 32]
    #     critic_hidden_dims = [128, 64, 32]
    #     activation = 'elu' # can be elu, relu, selu, crelu, lrelu, tanh, sigmoid

  
