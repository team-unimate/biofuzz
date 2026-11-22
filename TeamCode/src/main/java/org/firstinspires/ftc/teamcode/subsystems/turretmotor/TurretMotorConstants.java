package org.firstinspires.ftc.teamcode.subsystems.turretmotor;

import com.acmerobotics.dashboard.config.Config;

@Config
public class TurretMotorConstants {
    public static String MOTOR_NAME = "turretMotor";
    public static boolean MOTOR_REVERSED = true;
    public static double MOTOR_TICKS_PER_REV = 28 * 76.0 / 21.0;
    public static double MOTOR_GEAR_RATIO = 72.0 / 14.0;

    public static double MOTOR_TURRET_FWD = -1;
    public static double MOTOR_TURRET_LEFT = 0;

    public static double MOTOR_SANITY_MARGIN_DEG = 5;
    public static double MOTOR_MIN_ANGLE_DEG = -175;
    public static double MOTOR_MAX_ANGLE_DEG = 175;

    public static double MOTOR_kP = 1.5;
    public static double MOTOR_kI = 5;
    public static double MOTOR_kD = 0.05;
    public static double MOTOR_kS = 0.05;
    public static double MOTOR_kV = 0.03;

    public static double MOTOR_I_ZONE_DEG = 5;
    public static double MOTOR_I_MAX_POWER = 0.15;
    public static double MOTOR_FF_FILTER_ALPHA = 0.5;
    public static double MOTOR_FF_MAX_RATE_DEG_S = 720;

    public static double MOTOR_DEADBAND_DEG = 0.1;
    public static double MOTOR_MAX_POWER = 0.6;

    public static double MOTOR_VELOCITY_FILTER_ALPHA = 0.5;
    public static double MOTOR_AIM_TOLERANCE_DEG = 2;
    public static double MOTOR_SETTLED_VEL_DEG_S = 15;

    public static double MOTOR_STALL_MIN_POWER = 0.3;
    public static double MOTOR_STALL_VEL_DEG_S = 5;
    public static double MOTOR_STALL_TIME_S = 0.5;
    public static double MOTOR_STALL_COOLDOWN_S = 1.0;

    public static boolean MOTOR_USE_MANUAL_ANGLE = false;
    public static double MOTOR_MANUAL_ANGLE_DEG = 0;
    public static double MOTOR_MANUAL_STEP_DEG = 15;

    public static double AIM_OFFSET_DEG = 0;
    public static double IDLE_ANGLE_DEG = 0;

    public static boolean USE_VISION = false;
    public static double VISION_MAX_DISAGREE_DEG = 10;
    public static double BIAS_DECAY_PER_S = 0.5;
    public static double VISION_BIAS_ALPHA = 0.2;
    public static double VISION_LOCK_VALID_MS = 500;
    public static boolean ALLOW_ODOMETRY_ONLY_SHOTS = true;

    public static boolean SHOOT_ON_THE_MOVE = false;
    public static double TRANSFER_DELAY_S = 0.3;
    public static double[] TOF_DIST_IN = {24, 48, 72, 96, 120};
    public static double[] TOF_S = {0.35, 0.45, 0.65, 0.65, 0.75};
    public static double MAX_SHOOT_SPEED_IN_S = 30;
    public static double POSE_VELOCITY_ALPHA = 0.3;
    public static double MAX_ROBOT_SPEED_IN_S = 120;

    public static double MIN_SHOT_DIST = 24;
    public static double MAX_SHOT_DIST = 120;

    public static boolean SHOW_TAGS = false;
}
