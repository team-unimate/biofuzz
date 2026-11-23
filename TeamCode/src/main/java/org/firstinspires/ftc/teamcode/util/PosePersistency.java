package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.math.Pose;

public final class PosePersistency {
    private static Pose pose = null;
    private static double turretZeroTicks = Double.NaN;
    private static double turretMotorZeroTicks = Double.NaN;

    public static void savePose(Pose pose) {
        PosePersistency.pose = pose;
    }

    public static void saveTurretZero(double ticks) {
        turretZeroTicks = ticks;
    }

    public static double getTurretZero(double fallback) {
        return Double.isNaN(turretZeroTicks) ? fallback : turretZeroTicks;
    }

    public static void saveTurretMotorZero(double ticks) {
        turretMotorZeroTicks = ticks;
    }

    public static double getTurretMotorZero(double fallback) {
        return Double.isNaN(turretMotorZeroTicks) ? fallback : turretMotorZeroTicks;
    }

    public static Pose getPose(Pose fallback) {
        return pose != null ? pose : fallback;
    }

    public static boolean hasPose() {
        return pose != null;
    }

    public static void clear() {
        pose = null;
        turretZeroTicks = Double.NaN;
        turretMotorZeroTicks = Double.NaN;
    }

    private PosePersistency() {
    }
}
