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

    private AimMath() {
    }
}
