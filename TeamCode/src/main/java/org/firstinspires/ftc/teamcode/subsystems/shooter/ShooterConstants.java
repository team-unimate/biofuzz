package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class ShooterConstants {
    public static String MOTOR_NAME = "shooter";
    public static boolean MOTOR_REVERSED = false;
    public static double TICKS_PER_REV = 28;
    public static double MAX_RPM = 6000;

    public static PIDFCoefficients FLYWHEEL_PIDF_COEFFICIENTS = new PIDFCoefficients(0.01, 0, 0, 0.00054);
    public static double SHOOTER_TOLERANCE_RPM = 40;

    public static double[] SHOOTER_DIST_IN = {37.4, 47.2, 59.6, 84.4};
    public static double[] SHOOTER_RPM = {1600, 1660, 1720, 1920};
    public static double TUNING_RPM =  1950;
}
