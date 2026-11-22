package org.firstinspires.ftc.teamcode.subsystems.turretmotor;

import java.util.function.DoubleUnaryOperator;

public final class TurretMath {
    private TurretMath() {
    }

    public static double wrap(double radians) {
        return Math.atan2(Math.sin(radians), Math.cos(radians));
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

    public static double[] leadPivot(double x, double y, double velX, double velY, double aimX, double aimY,
                                     double transferDelay, DoubleUnaryOperator timeOfFlight) {
        double t = transferDelay + timeOfFlight.applyAsDouble(Math.hypot(aimX - x, aimY - y));
        for (int i = 0; i < 3; i++) {
            t = transferDelay + timeOfFlight.applyAsDouble(Math.hypot(aimX - (x + velX * t), aimY - (y + velY * t)));
        }
        return new double[]{x + velX * t, y + velY * t, t};
    }
}
