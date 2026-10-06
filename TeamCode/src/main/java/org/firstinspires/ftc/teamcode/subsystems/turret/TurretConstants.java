package org.firstinspires.ftc.teamcode.subsystems.turret;

import com.acmerobotics.dashboard.config.Config;

@Config
public class TurretConstants {
    public static String SERVO_NAME = "turret";
    public static String ENCODER_NAME = "encoder";

    public static boolean SERVO_REVERSED = false;

    public static boolean ENCODER_REVERSED = false;

    public static double GEAR_RATIO = 25.0 / 20.0;

    public static double RAW_RANGE_DEG = 315;

    public static double ZERO_RAW_DEG = 157.5;

    public static double RAW_SANITY_MARGIN_DEG = 125;

    public static double TURRET_FWD = 0;
    public static double TURRET_LEFT = 0;

    public static double MIN_ANGLE_DEG = -90;
    public static double MAX_ANGLE_DEG = 90;

    public static double kP = 0.8;
    public static double kI = 0;
    public static double kD = 0.02;
    public static double kF = 0;

    public static double kS = 0.04;
    public static double DEADBAND_DEG = 0.75;

    public static double kTurnFF = 0.0;
    public static double MAX_POWER = 1.0;

    public static double VELOCITY_FILTER_ALPHA = 0.3;

    public static double AIM_TOLERANCE_DEG = 2;
    public static double SETTLED_VEL_DEG_S = 15;
}
