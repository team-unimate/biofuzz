package org.firstinspires.ftc.teamcode.subsystems.turret;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import java.util.function.DoubleUnaryOperator;

public class VirtualPoseTest {
    private static final double EPS = 0.0001;
    private static final DoubleUnaryOperator TOF = d -> 0.25 + d / 240;
    private static final double AIM_X = 100;
    private static final double AIM_Y = 0;

    private static double[] solve(double velX, double velY) {
        return Turret.virtualPose(0, 0, velX, velY, AIM_X, AIM_Y, 0.15, TOF);
    }

    private static double leadAngle(double[] virtual) {
        return Turret.wrap(Turret.odomAngle(virtual[0], virtual[1], 0, AIM_X, AIM_Y)
                - Turret.odomAngle(0, 0, 0, AIM_X, AIM_Y));
    }

    private static double distance(double[] virtual) {
        return Math.hypot(AIM_X - virtual[0], AIM_Y - virtual[1]);
    }

    @Test
    public void stationaryRobotHasNoLead() {
        double[] virtual = solve(0, 0);
        assertEquals(0, leadAngle(virtual), EPS);
        assertEquals(100, distance(virtual), EPS);
    }

    @Test
    public void sidewaysMotionLeadsOppositeAndGrowsWithSpeed() {
        double slow = leadAngle(solve(0, 10));
        double fast = leadAngle(solve(0, 20));
        assertTrue(slow < 0);
        assertTrue(fast < slow);
    }

    @Test
    public void drivingTowardCellShortensDistance() {
        assertTrue(distance(solve(20, 0)) < 100);
    }
}
