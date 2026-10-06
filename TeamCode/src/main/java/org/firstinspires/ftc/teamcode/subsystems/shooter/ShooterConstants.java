package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterConstants {
    public static String MOTOR_NAME = "flywheel";
    public static boolean MOTOR_REVERSED = true;
    public static double TICKS_PER_REV = 28;

    public static double HIVE_RPM = 1850;

    public static double MAX_RPM = 6000;

    public static double kP = 0.1;
    public static double kI = 0;
    public static double kD = 0.02;
    public static double kF = 0.00071;

    public static double VELOCITY_TOLERANCE = 20;

    // Anti-windup bounds on the PIDF controller's accumulated integral (ticks/sec-equivalent).
    // The controller library defaults to +-1, which saturates almost instantly against
    // velocity errors in the hundreds/thousands of ticks/sec - these widen it so kI has a
    // meaningful effect instead of acting like an on/off bias.
    public static double INTEGRAL_MIN = -3000;
    public static double INTEGRAL_MAX = 3000;
}
