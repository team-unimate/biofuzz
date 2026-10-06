package org.firstinspires.ftc.teamcode.vision;

import static org.firstinspires.ftc.teamcode.vision.AimConstants.*;

import com.pedropathing.utils.Angle;
import com.qualcomm.robotcore.util.Range;

/**
 * Computes the rotational power needed to keep the robot's heading pointed at a fixed field
 * position (an AprilTag's known (x, y)), given the robot's current Pedro Pathing pose.
 * <p>
 * This class does not touch hardware or the Follower directly - it only produces a turn power
 * (-1..1) that the caller feeds into follower.manual(...)/DrivePowers alongside the driver's
 * translational input, the same way Pedro's own ManualDrive.headingLock(...) is meant to be used.
 * All angles are in radians unless the method name says degrees. It has no notion of alliances or
 * tag ID sets - it just aims at whatever (x, y) it's given.
 */
public class AprilTagAimController {
    private double integral = 0;
    private double previousError = 0;
    private long previousTimeNanos = 0;
    private boolean firstUpdate = true;

    private long lastSeenTimeMillis = -1;
    private double lastTargetHeading = 0;

    /** desiredHeading = atan2(tagY - robotY, tagX - robotX) */
    public double calculateAimHeading(double robotX, double robotY, double tagX, double tagY) {
        return Math.atan2(tagY - robotY, tagX - robotX);
    }

    /** Wrapped heading error (target - current), normalized to -PI..PI. */
    public static double wrapHeadingError(double targetHeadingRadians, double currentHeadingRadians) {
        return Angle.error(currentHeadingRadians, targetHeadingRadians);
    }

    /**
     * PD(+I) controller from a current heading to a target heading. Clamps to +-MAX_TURN_POWER
     * and zeroes out (with the integral reset) inside the TOLERANCE_DEGREES deadband.
     */
    public double getAimTurnPower(double currentHeadingRadians, double targetHeadingRadians) {
        double error = wrapHeadingError(targetHeadingRadians, currentHeadingRadians);

        if (Math.abs(error) < Math.toRadians(TOLERANCE_DEGREES)) {
            integral = 0;
            previousError = error;
            firstUpdate = false;
            return 0;
        }

        long now = System.nanoTime();
        double dt = firstUpdate ? 0 : (now - previousTimeNanos) * 1e-9;
        previousTimeNanos = now;

        if (dt > 1e-3) {
            integral = Range.clip(integral + error * dt, -INTEGRAL_MAX, INTEGRAL_MAX);
        }

        double derivative = (!firstUpdate && dt > 1e-3) ? (error - previousError) / dt : 0;
        previousError = error;
        firstUpdate = false;

        double power = kP * error + kI * integral + kD * derivative;
        return Range.clip(power, -MAX_TURN_POWER, MAX_TURN_POWER);
    }

    /**
     * Full pipeline for a known tag field position: updates/holds the target heading based on
     * visibility and returns a turn power. Once the tag has been out of view for longer than
     * AimConstants.LOST_TAG_HOLD_MS, resets and returns 0 instead of continuing to chase a stale
     * heading.
     */
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

    /** Same as above, looking the tag's field position up in AprilTagField by id. */
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
