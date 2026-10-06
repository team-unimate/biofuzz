package org.firstinspires.ftc.teamcode.subsystems.turret;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class TurretTest {
    private static final double EPS = 1e-9;
    private static final double MIN = Math.toRadians(-90);
    private static final double MAX = Math.toRadians(90);

    @Test
    public void rawToAngleUsesGearRatioAndDirection() {
        assertEquals(0, Turret.rawToAngle(157.5, 157.5, 1.25, false), EPS);
        assertEquals(Math.toRadians(90), Turret.rawToAngle(157.5 + 112.5, 157.5, 1.25, false), EPS);
        assertEquals(Math.toRadians(-90), Turret.rawToAngle(157.5 + 112.5, 157.5, 1.25, true), EPS);
    }

    @Test
    public void targetInsideLimitsIsKept() {
        assertEquals(Math.toRadians(45), Turret.reachableEquivalent(Math.toRadians(45), 0, MIN, MAX), EPS);
    }

    @Test
    public void wrappedTargetIsBroughtBackInsideLimits() {
        assertEquals(Math.toRadians(10), Turret.reachableEquivalent(Math.toRadians(370), 0, MIN, MAX), EPS);
    }

    @Test
    public void targetBehindRobotIsUnreachable() {
        assertTrue(Double.isNaN(Turret.reachableEquivalent(Math.toRadians(180), 0, MIN, MAX)));
    }

    @Test
    public void closestEquivalentWinsWhenRangeIsWide() {
        double min = Math.toRadians(-200), max = Math.toRadians(200);

        assertEquals(Math.toRadians(190), Turret.reachableEquivalent(Math.toRadians(-170), Math.toRadians(150), min, max), 1e-6);
    }

    @Test
    public void unreachableTargetClampsToNearestLimit() {
        assertEquals(MAX, Turret.nearestLimit(Math.toRadians(120), MIN, MAX), EPS);
        assertEquals(MIN, Turret.nearestLimit(Math.toRadians(-120), MIN, MAX), EPS);

        assertEquals(Math.toRadians(60), Turret.nearestLimit(Math.toRadians(150), Math.toRadians(-30), Math.toRadians(60)), EPS);
    }
}
