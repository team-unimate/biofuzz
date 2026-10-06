package org.firstinspires.ftc.teamcode.subsystems.turret;

import com.acmerobotics.dashboard.config.Config;

/**
 * Turret tunables (see TURRET.md). Angles in this class are degrees (names end in _DEG); the
 * turret code works in radians internally. Turret angle 0 = facing robot forward, positive = CCW.
 */
@Config
public class TurretConstants {
    // Hardware names (must match the Driver Station configuration).
    public static String SERVO_NAME = "turret";
    public static String ENCODER_NAME = "encoder";

    // Flip if positive power turns the turret clockwise (seen from above).
    public static boolean SERVO_REVERSED = false;
    // Flip if turning the turret CCW makes the measured angle go down.
    public static boolean ENCODER_REVERSED = false;

    // Taura on the 20T gear, turret on the 25T: servo degrees per turret degree.
    public static double GEAR_RATIO = 25.0 / 20.0;
    // Degrees of servo rotation covered by the analog feedback's 0..max voltage.
    public static double RAW_RANGE_DEG = 315;
    // Raw reading (degrees) with the turret facing forward. Calibrate: read "Raw deg" telemetry.
    // Should be near the middle of the sensor range so the 315 -> 0 jump is never reached.
    public static double ZERO_RAW_DEG = 157.5;
    // A raw reading farther than this from ZERO_RAW_DEG is impossible: encoder fault, turret stops.
    // Must stay larger than (largest soft limit * GEAR_RATIO).
    public static double RAW_SANITY_MARGIN_DEG = 125;

    // Soft limits (turret degrees). Can be asymmetric. Keep inside the cable range.
    public static double MIN_ANGLE_DEG = -90;
    public static double MAX_ANGLE_DEG = 90;

    // Position controller. Error is in radians, output is servo power.
    public static double kP = 0.8;
    public static double kI = 0;
    public static double kD = 0.02;
    public static double kF = 0;
    // Static friction power, added in the direction of the error outside the deadband.
    public static double kS = 0.04;
    public static double DEADBAND_DEG = 0.75;
    // Power per rad/s of robot turn rate, so the turret holds its field direction while the robot spins.
    public static double kTurnFF = 0.0;
    public static double MAX_POWER = 1.0;

    // Low-pass filter for turret and robot turn rates (0..1, higher = less filtering).
    public static double VELOCITY_FILTER_ALPHA = 0.3;

    // "Settled" = aim error and turret speed both small.
    public static double AIM_TOLERANCE_DEG = 2;
    public static double SETTLED_VEL_DEG_S = 15;
}
