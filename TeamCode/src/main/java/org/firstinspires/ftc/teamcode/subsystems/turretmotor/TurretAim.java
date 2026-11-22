package org.firstinspires.ftc.teamcode.subsystems.turretmotor;

import static org.firstinspires.ftc.teamcode.subsystems.turretmotor.TurretMath.leadPivot;
import static org.firstinspires.ftc.teamcode.subsystems.turretmotor.TurretMath.odomAngle;
import static org.firstinspires.ftc.teamcode.subsystems.turretmotor.TurretMath.pivot;
import static org.firstinspires.ftc.teamcode.subsystems.turretmotor.TurretMath.visionAngle;
import static org.firstinspires.ftc.teamcode.subsystems.turretmotor.TurretMath.wrap;
import static org.firstinspires.ftc.teamcode.subsystems.turretmotor.TurretMotorConstants.*;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.field.HiveCells;
import org.firstinspires.ftc.teamcode.vision.CellObservation;
import org.firstinspires.ftc.teamcode.vision.Limelight;

public class TurretAim {
    private final Limelight vision;
    private final InterpLUT timeOfFlightTable = new InterpLUT();

    private Field.Alliance alliance = null;
    private HiveCells.Cell cell = null;

    private double poseTime = Double.NaN;
    private double lastX = 0;
    private double lastY = 0;
    private double velocityX = 0;
    private double velocityY = 0;

    private double bias = 0;
    private double visionDistance = Double.NaN;
    private double lastSeenTime = Double.NaN;
    private boolean seen = false;

    private double target = 0;
    private double previousTarget = Double.NaN;
    private double targetRate = 0;
    private double distance = Double.NaN;
    private double leadTime = 0;

    public TurretAim(Limelight vision) {
        this.vision = vision;
        for (int i = 0; i < Math.min(TOF_DIST_IN.length, TOF_S.length); i++) {
            timeOfFlightTable.add(TOF_DIST_IN[i], TOF_S[i]);
        }
        timeOfFlightTable.createLUT();
    }

    public void setAlliance(Field.Alliance alliance) {
        if (alliance != this.alliance) forgetVision();
        this.alliance = alliance;
    }

    public void update(Pose robot, double now, double dt) {
        updateRobotVelocity(robot, now);
        updateCell(robot);
        if (cell == null) {
            clearTarget();
            return;
        }

        double cellX = cell.pose.x();
        double cellY = cell.pose.y();
        double[] pivot = pivot(robot.x(), robot.y(), robot.heading(), MOTOR_TURRET_FWD, MOTOR_TURRET_LEFT);
        double odomAngle = odomAngle(pivot[0], pivot[1], robot.heading(), cellX, cellY);
        double odomDistance = Math.hypot(cellX - pivot[0], cellY - pivot[1]);

        double vx = SHOOT_ON_THE_MOVE ? velocityX : 0;
        double vy = SHOOT_ON_THE_MOVE ? velocityY : 0;
        double[] lead = leadPivot(pivot[0], pivot[1], vx, vy, cellX, cellY, TRANSFER_DELAY_S, timeOfFlightTable::get);
        leadTime = lead[2];
        double leadAngle = odomAngle(lead[0], lead[1], robot.heading(), cellX, cellY);
        double leadDistance = Math.hypot(cellX - lead[0], cellY - lead[1]);

        updateVisionBias(odomAngle, robot.heading(), now, dt);

        target = wrap(leadAngle + bias + Math.toRadians(AIM_OFFSET_DEG));
        updateTargetRate(dt);
        distance = (seen ? visionDistance : odomDistance) + leadDistance - odomDistance;
    }

    private void updateRobotVelocity(Pose robot, double now) {
        if (!Double.isNaN(poseTime) && now > poseTime) {
            double vx = (robot.x() - lastX) / (now - poseTime);
            double vy = (robot.y() - lastY) / (now - poseTime);
            if (Math.hypot(vx, vy) <= MAX_ROBOT_SPEED_IN_S) {
                velocityX += POSE_VELOCITY_ALPHA * (vx - velocityX);
                velocityY += POSE_VELOCITY_ALPHA * (vy - velocityY);
            }
        }
        poseTime = now;
        lastX = robot.x();
        lastY = robot.y();
    }

    private void updateCell(Pose robot) {
        HiveCells.Cell nextCell = alliance == null ? null : HiveCells.forRobot(alliance, robot);
        if (nextCell == cell) return;
        cell = nextCell;
        forgetVision();
        previousTarget = Double.NaN;
        targetRate = 0;
    }

    private void updateVisionBias(double odomAngle, double heading, double now, double dt) {
        CellObservation obs = USE_VISION && vision != null && vision.hasNewFrame() ? vision.getObservation(cell) : null;
        if (obs != null) {
            double error = wrap(visionAngle(obs.turretAngleAtCapture, obs.bearing, heading, obs.headingAtCapture) - odomAngle);
            if (Math.abs(error) <= Math.toRadians(VISION_MAX_DISAGREE_DEG)) {
                bias += VISION_BIAS_ALPHA * (error - bias);
                visionDistance = obs.distance;
                lastSeenTime = now;
            }
        }
        seen = !Double.isNaN(lastSeenTime) && (now - lastSeenTime) * 1000 <= VISION_LOCK_VALID_MS;
        if (!seen) bias *= Math.exp(-BIAS_DECAY_PER_S * dt);
    }

    private void updateTargetRate(double dt) {
        if (!Double.isNaN(previousTarget) && dt > 0) {
            double maxRate = Math.toRadians(MOTOR_FF_MAX_RATE_DEG_S);
            double rate = Range.clip(wrap(target - previousTarget) / dt, -maxRate, maxRate);
            targetRate += MOTOR_FF_FILTER_ALPHA * (rate - targetRate);
        }
        previousTarget = target;
    }

    private void forgetVision() {
        bias = 0;
        lastSeenTime = Double.NaN;
    }

    private void clearTarget() {
        previousTarget = Double.NaN;
        targetRate = 0;
        target = 0;
        distance = Double.NaN;
        seen = false;
        leadTime = 0;
    }

    public boolean hasCell() {
        return cell != null;
    }

    public boolean hasGoodShot() {
        return cell != null
                && !isTooClose() && distance <= MAX_SHOT_DIST && !isTooFast()
                && (seen || ALLOW_ODOMETRY_ONLY_SHOTS);
    }

    public boolean isTooClose() {
        return seen && visionDistance < MIN_SHOT_DIST;
    }

    public boolean isTooFast() {
        return getRobotSpeed() > MAX_SHOOT_SPEED_IN_S;
    }

    public double getRobotSpeed() {
        return Math.hypot(velocityX, velocityY);
    }

    public HiveCells.Cell getCell() {
        return cell;
    }

    public double getTarget() {
        return target;
    }

    public double getTargetRate() {
        return targetRate;
    }

    public double getDistance() {
        return distance;
    }

    public double getBias() {
        return bias;
    }

    public boolean isSeen() {
        return seen;
    }

    public double getLeadTime() {
        return leadTime;
    }
}
