package org.firstinspires.ftc.teamcode.subsystems.turretmotor;

import static org.firstinspires.ftc.teamcode.subsystems.turretmotor.TurretMotorConstants.*;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.field.HiveCells;
import org.junit.Test;

public class TurretAimTest {
    private static final double EPS = 0.0001;
    private static final Pose START = new Pose(Field.CENTER_X, Field.CENTER_Y + 1, 0);

    private static TurretAim blueAim() {
        TurretAim aim = new TurretAim(null);
        aim.setAlliance(Field.Alliance.BLUE);
        return aim;
    }

    private static double odomTarget(Pose robot, HiveCells.Cell cell) {
        double[] p = TurretMath.pivot(robot.x(), robot.y(), robot.heading(), MOTOR_TURRET_FWD, MOTOR_TURRET_LEFT);
        return TurretMath.odomAngle(p[0], p[1], robot.heading(), cell.pose.x(), cell.pose.y());
    }

    private static double odomDistance(Pose robot, HiveCells.Cell cell) {
        double[] p = TurretMath.pivot(robot.x(), robot.y(), robot.heading(), MOTOR_TURRET_FWD, MOTOR_TURRET_LEFT);
        return Math.hypot(cell.pose.x() - p[0], cell.pose.y() - p[1]);
    }

    @Test
    public void noAllianceMeansNoTarget() {
        TurretAim aim = new TurretAim(null);
        aim.update(START, 0, 0);
        assertNull(aim.getCell());
        assertEquals(0, aim.getTarget(), EPS);
        assertTrue(Double.isNaN(aim.getDistance()));
        assertFalse(aim.hasGoodShot());
    }

    @Test
    public void stationaryRobotAimsStraightAtTheCell() {
        TurretAim aim = blueAim();
        aim.update(START, 0, 0);
        HiveCells.Cell cell = HiveCells.Cell.BLUE_AUDIENCE;
        assertEquals(cell, aim.getCell());
        assertEquals(odomTarget(START, cell), aim.getTarget(), EPS);
        assertEquals(odomDistance(START, cell), aim.getDistance(), EPS);
        assertTrue(aim.hasGoodShot());
    }

    @Test
    public void targetRateFollowsRobotRotation() {
        TurretAim aim = blueAim();
        Pose turned = new Pose(START.x(), START.y(), Math.toRadians(10));
        aim.update(START, 0, 0);
        aim.update(turned, 0.1, 0.1);
        double rate = TurretMath.wrap(odomTarget(turned, aim.getCell()) - odomTarget(START, aim.getCell())) / 0.1;
        assertEquals(MOTOR_FF_FILTER_ALPHA * rate, aim.getTargetRate(), EPS);
    }

    @Test
    public void drivingFastBlocksTheShot() {
        TurretAim aim = blueAim();
        aim.update(START, 0, 0);
        aim.update(new Pose(START.x() + 11, START.y(), 0), 0.1, 0.1);
        assertTrue(aim.isTooFast());
        assertFalse(aim.hasGoodShot());
    }

    @Test
    public void impossibleSpeedJumpsAreIgnored() {
        TurretAim aim = blueAim();
        aim.update(START, 0, 0);
        aim.update(new Pose(START.x() + 50, START.y(), 0), 0.1, 0.1);
        assertEquals(0, aim.getRobotSpeed(), EPS);
    }
}
