from legged_gym.envs.base.legged_robot_config import LeggedRobotCfg, LeggedRobotCfgPPO

class JAMALRoughCfg( LeggedRobotCfg ):
    class init_state( LeggedRobotCfg.init_state ):
        pos = [0.0, 0.0, 0.35] # x,y,z [m]
        default_joint_angles = { # = target angles [rad] when action = 0.0
            'FL_hip_joint': 0.1,   # [rad]
            'RL_hip_joint': 0.1,   # [rad]
            'FR_hip_joint': -0.1 ,  # [rad]
            'RR_hip_joint': -0.1,   # [rad]

            'FL_thigh_joint': 0.585,     # [rad]
            'RL_thigh_joint': 0.9,   # [rad]
            'FR_thigh_joint': 0.585,     # [rad]
            'RR_thigh_joint': 0.9,   # [rad]

            'FL_calf_joint': -1.35,   # [rad]
            'RL_calf_joint': -1.35,    # [rad]
            'FR_calf_joint': -1.35,  # [rad]
            'RR_calf_joint': -1.35,    # [rad]
        }

    class control( LeggedRobotCfg.control ):
        # PD Drive parameters:
        # torque_limits = {'joint': 60.0}
        control_type = 'P'
        stiffness = {'joint': 60.0}  # [N*m/rad]
        damping = {'joint': 3.5}     # [N*m*s/rad]
        # action scale: target angle = actionScale * action + defaultAngle
        # action_scale = 0.25
        action_scale = 0.15
        # decimation: Number of control action updates @ sim DT per policy DT
        decimation = 4

    class asset( LeggedRobotCfg.asset ):
        file = '{LEGGED_GYM_ROOT_DIR}/resources/robots/jamal/urdf/jamal2_v5.urdf'
        name = "jamal"
        # foot_name = ['FL_foot', 'FR_foot', 'RL_foot', 'RR_foot']
        foot_name = "foot"
        penalize_contacts_on = ["thigh", "calf"]
        terminate_after_contacts_on = ["base"]
        # terminate_after_contacts_on = ["trunk"]
        self_collisions = 1 # 1 to disable, 0 to enable...bitwise filter
        # flip_visual_attachments = False
  
    class rewards( LeggedRobotCfg.rewards ):
        soft_dof_pos_limit = 0.9
        # base_height_target = 0.25
        base_height_target = 0.4
        class scales( LeggedRobotCfg.rewards.scales ):
            torques = -0.0010
            # dof_pos_limits = -10.0
            # termination = -0.0
            # tracking_lin_vel = 1.0
            # tracking_ang_vel = 0.5
            # lin_vel_z = -2.0
            # ang_vel_xy = -0.05
            # orientation = -0.
            # torques = -0.00001
            # dof_vel = -0.
            # dof_acc = -2.5e-7
            # base_height = -0. 
            # feet_air_time =  1.0
            # collision = -1.
            # feet_stumble = -0.0 
            # action_rate = -0.01
            # # stand_still = -0.
            # feet_stumble = -0.0 

            base_height   =  2.0          # reward staying near target height
            orientation   = -0.0          # penalize roll/pitch away from upright
            ang_vel_xy    = -0.2          # reduce rocking

            # Motion shaping
            tracking_lin_vel = 1.0
            tracking_ang_vel = 0.5
            lin_vel_z     = -2.0          # stronger push against vertical motion
            feet_air_time =  0.0          # disable airtime farming (was 1.0)

            # Smoothness / effort
            action_rate   = -0.02         # slightly stronger smoothing
            dof_vel       = -0.001        # add velocity damping
            dof_acc       = -1e-6         # slightly stronger accel cost
            torques       = -0.0002       # 20x larger than your current tiny cost

            # Safety / constraints
            collision     = -1.0
            dof_pos_limits= -0.2
            # feet_stumble  = -0.1
            termination   = -5.0          # make falling clearly bad
            stand_still   =  0.0          # keep disabled unless you train standing


class JAMALRoughCfgPPO( LeggedRobotCfgPPO ):
    class algorithm( LeggedRobotCfgPPO.algorithm ):
        entropy_coef = 0.01
    class runner( LeggedRobotCfgPPO.runner ):
        run_name = ''
        experiment_name = 'rough_jamal'

  
