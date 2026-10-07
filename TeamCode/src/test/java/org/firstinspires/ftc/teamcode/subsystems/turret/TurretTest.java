package org.firstinspires.ftc.teamcode.subsystems.turret;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class TurretTest {
    private static final double EPS = 0.0001;

    @Test
    public void rawToAngleUsesGearRatioAndDirection() {
        assertEquals(0, Turret.rawToAngle(157.5, 157.5, 1.25, false), EPS);
        assertEquals(Math.toRadians(90), Turret.rawToAngle(157.5 + 112.5, 157.5, 1.25, false), EPS);
        assertEquals(Math.toRadians(-90), Turret.rawToAngle(157.5 + 112.5, 157.5, 1.25, true), EPS);
    }

    @Test
    public void pivotIsOffsetInRobotFrame() {
        assertArrayEquals(new double[]{10, 23}, Turret.pivot(10, 20, Math.toRadians(90), 3, 0), EPS);
        assertArrayEquals(new double[]{8, 20}, Turret.pivot(10, 20, Math.toRadians(90), 0, 2), EPS);
    }

    @Test
    public void odomAngleIsRobotRelative() {
        assertEquals(0, Turret.odomAngle(0, 0, Math.toRadians(90), 0, 50), EPS);
        assertEquals(Math.toRadians(-90), Turret.odomAngle(0, 0, 0, 0, -50), EPS);
        assertEquals(Math.toRadians(45), Turret.odomAngle(0, 0, 0, 10, 10), EPS);
    }

    @Test
    public void visionAngleRemovesRobotTurnSinceCapture() {
        assertEquals(Math.toRadians(15), Turret.visionAngle(Math.toRadians(10), Math.toRadians(5), 0, 0), EPS);
        assertEquals(Math.toRadians(5), Turret.visionAngle(Math.toRadians(10), Math.toRadians(5),
                Math.toRadians(30), Math.toRadians(20)), EPS);
    }
}
