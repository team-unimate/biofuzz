package org.firstinspires.ftc.teamcode.subsystems.turret;

public final class AimMath {
    public enum Status {
        CELL_UNKNOWN,
        HIVE_JUST_FLIPPED,
        TOO_CLOSE,
        TOO_FAR,
        OUT_OF_RANGE,
        NOT_SETTLED,
        SHOOTER_NOT_READY,
        NO_RECENT_VISION,
        OK
    }

    public static double[] pivot(double x, double y, double heading, double pivotForward, double pivotLeft) {
        double c = Math.cos(heading), s = Math.sin(heading);
        return new double[]{x + pivotForward * c - pivotLeft * s, y + pivotForward * s + pivotLeft * c};
    }

    public static double odomAngle(double pivotX, double pivotY, double heading, double aimX, double aimY) {
        return wrap(Math.atan2(aimY - pivotY, aimX - pivotX) - heading);
    }

    public static double visionAngle(double turretAtCapture, double bearing, double headingNow, double headingAtCapture) {
        return wrap(turretAtCapture + bearing - wrap(headingNow - headingAtCapture));
    }

    public static double updateBias(double bias, double error, double alpha) {
        return bias + alpha * (error - bias);
    }

    public static double decayBias(double bias, double ratePerSecond, double dtSeconds) {
        return bias * Math.exp(-ratePerSecond * dtSeconds);
    }

    public static Status status(boolean cellKnown, boolean justFlipped, double distance, double minDistance,
                                double maxDistance, boolean reachable, boolean settled, boolean shooterReady,
                                boolean recentVision, boolean allowOdometryOnly) {
        if (!cellKnown) return Status.CELL_UNKNOWN;
        if (justFlipped) return Status.HIVE_JUST_FLIPPED;
        if (Double.isNaN(distance) || distance < minDistance) return Status.TOO_CLOSE;
        if (distance > maxDistance) return Status.TOO_FAR;
        if (!reachable) return Status.OUT_OF_RANGE;
        if (!settled) return Status.NOT_SETTLED;
        if (!shooterReady) return Status.SHOOTER_NOT_READY;
        if (!recentVision && !allowOdometryOnly) return Status.NO_RECENT_VISION;
        return Status.OK;
    }

    public static double wrap(double radians) {
        return Math.atan2(Math.sin(radians), Math.cos(radians));
    }

    private AimMath() {
    }
}
