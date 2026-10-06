package org.firstinspires.ftc.teamcode.vision;

import static org.firstinspires.ftc.teamcode.vision.AimConstants.*;

import com.pedropathing.utils.Angle;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.util.Clock;


public class AprilTagAimController {
    private static final double MIN_DT_S = 0.001;

    private double integral = 0;
    private double previousError = 0;
    private double previousTime = 0;
    private boolean firstUpdate = true;

    private long lastSeenTimeMillis = -1;
    private double lastTargetHeading = 0;

    public double calculateAimHeading(double robotX, double robotY, double tagX, double tagY) {
        return Math.atan2(tagY - robotY, tagX - robotX);
    }

    public static double wrapHeadingError(double targetHeadingRadians, double currentHeadingRadians) {
        return Angle.error(currentHeadingRadians, targetHeadingRadians);
    }

    public double getAimTurnPower(double currentHeadingRadians, double targetHeadingRadians) {
        double error = wrapHeadingError(targetHeadingRadians, currentHeadingRadians);

        if (Math.abs(error) < Math.toRadians(TOLERANCE_DEGREES)) {
            integral = 0;
            previousError = error;
            firstUpdate = false;
            return 0;
        }

        double now = Clock.seconds();
        double dt = firstUpdate ? 0 : now - previousTime;
        previousTime = now;

        if (dt > MIN_DT_S) {
            integral = Range.clip(integral + error * dt, -INTEGRAL_MAX, INTEGRAL_MAX);
        }

        double derivative = (!firstUpdate && dt > MIN_DT_S) ? (error - previousError) / dt : 0;
        previousError = error;
        firstUpdate = false;

        double power = kP * error + kI * integral + kD * derivative;
        return Range.clip(power, -MAX_TURN_POWER, MAX_TURN_POWER);
    }

    public double aimAtAprilTag(double robotX, double robotY, double currentHeadingRadians,
                                 double tagX, double tagY, boolean tagVisible) {
        long now = System.currentTimeMillis();

        if (tagVisible) {
            lastSeenTimeMillis = now;
            lastTargetHeading = calculateAimHeading(robotX, robotY, tagX, tagY);
        } else if (lastSeenTimeMillis < 0 || now - lastSeenTimeMillis > LOST_TAG_HOLD_MS) {
            reset();
            return 0;
        }

        return getAimTurnPower(currentHeadingRadians, lastTargetHeading);
    }

    public double aimAtAprilTag(double robotX, double robotY, double currentHeadingRadians,
                                 int tagId, boolean tagVisible) {
        AprilTagTarget target = AprilTagField.get(tagId);
        if (target == null) {
            reset();
            return 0;
        }
        return aimAtAprilTag(robotX, robotY, currentHeadingRadians, target.x, target.y, tagVisible);
    }

    public double getLastTargetHeading() {
        return lastTargetHeading;
    }

    public void reset() {
        integral = 0;
        previousError = 0;
        firstUpdate = true;
        lastSeenTimeMillis = -1;
        lastTargetHeading = 0;
    }
}
