package org.firstinspires.ftc.teamcode.vision;

public class AimConstants {
    public static double kP = 1.2;
    public static double kI = 0.0;
    public static double kD = 0.08;

    // Anti-windup clamp on the accumulated integral term (power units).
    public static double INTEGRAL_MAX = 0.3;

    // Max commanded rotational power, +-.
    public static double MAX_TURN_POWER = 0.6;

    // Heading error (degrees) inside which no correction is applied, so the robot doesn't
    // oscillate once it's already aimed.
    public static double TOLERANCE_DEGREES = 1.5;

    // How long (ms) to keep correcting toward the last known tag heading after the tag drops
    // out of view, before giving up and stopping the correction.
    public static long LOST_TAG_HOLD_MS = 300;
}
