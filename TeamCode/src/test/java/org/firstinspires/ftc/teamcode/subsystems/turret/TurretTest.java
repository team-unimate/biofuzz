package org.firstinspires.ftc.teamcode.subsystems.turret;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class TurretTest {
    private static final double EPS = 0.000001;

    @Test
    public void rawToAngleUsesGearRatioAndDirection() {
        assertEquals(0, Turret.rawToAngle(157.5, 157.5, 1.25, false), EPS);
        assertEquals(Math.toRadians(90), Turret.rawToAngle(157.5 + 112.5, 157.5, 1.25, false), EPS);
        assertEquals(Math.toRadians(-90), Turret.rawToAngle(157.5 + 112.5, 157.5, 1.25, true), EPS);
    }

    @Test
    public void normalizeAngleKeepsAnglesBetweenMinusPiAndPi() {
        assertEquals(Math.toRadians(45), Turret.normalizeAngle(Math.toRadians(45)), EPS);
        assertEquals(Math.toRadians(-10), Turret.normalizeAngle(Math.toRadians(350)), EPS);
        assertEquals(Math.toRadians(10), Turret.normalizeAngle(Math.toRadians(-350)), EPS);
        assertEquals(Math.toRadians(90), Turret.normalizeAngle(Math.toRadians(90 + 720)), EPS);
    }
}
