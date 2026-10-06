package org.firstinspires.ftc.teamcode.subsystems.turret;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class AimMathTest {
    private static final double EPS = 1e-9;

    @Test
    public void pivotIsOffsetInRobotFrame() {
        assertArrayEquals(new double[]{10, 23}, AimMath.pivot(10, 20, Math.toRadians(90), 3, 0), EPS);
        assertArrayEquals(new double[]{8, 20}, AimMath.pivot(10, 20, Math.toRadians(90), 0, 2), EPS);
    }

    @Test
    public void odomAngleIsRobotRelative() {
        assertEquals(0, AimMath.odomAngle(0, 0, Math.toRadians(90), 0, 50), EPS);
        assertEquals(Math.toRadians(-90), AimMath.odomAngle(0, 0, 0, 0, -50), EPS);
        assertEquals(Math.toRadians(45), AimMath.odomAngle(0, 0, 0, 10, 10), EPS);
    }

    @Test
    public void visionAngleRemovesRobotTurnSinceCapture() {
        assertEquals(Math.toRadians(15), AimMath.visionAngle(Math.toRadians(10), Math.toRadians(5), 0, 0), EPS);
        assertEquals(Math.toRadians(5), AimMath.visionAngle(Math.toRadians(10), Math.toRadians(5),
                Math.toRadians(30), Math.toRadians(20)), EPS);
    }

    @Test
    public void biasMovesTowardErrorAndDecays() {
        assertEquals(0.2, AimMath.updateBias(0, 1, 0.2), EPS);
        assertEquals(Math.exp(-1), AimMath.decayBias(1, 0.5, 2), EPS);
    }

    @Test
    public void statusReportsFirstFailureInOrder() {
        assertEquals(AimMath.Status.CELL_UNKNOWN, AimMath.status(false, true, 5, 24, 120, false, false, false, false, false));
        assertEquals(AimMath.Status.HIVE_JUST_FLIPPED, AimMath.status(true, true, 5, 24, 120, false, false, false, false, false));
        assertEquals(AimMath.Status.TOO_CLOSE, AimMath.status(true, false, 5, 24, 120, false, false, false, false, false));
        assertEquals(AimMath.Status.TOO_CLOSE, AimMath.status(true, false, Double.NaN, 24, 120, true, true, true, true, false));
        assertEquals(AimMath.Status.TOO_FAR, AimMath.status(true, false, 200, 24, 120, false, false, false, false, false));
        assertEquals(AimMath.Status.OUT_OF_RANGE, AimMath.status(true, false, 60, 24, 120, false, false, false, false, false));
        assertEquals(AimMath.Status.NOT_SETTLED, AimMath.status(true, false, 60, 24, 120, true, false, false, false, false));
        assertEquals(AimMath.Status.SHOOTER_NOT_READY, AimMath.status(true, false, 60, 24, 120, true, true, false, false, false));
        assertEquals(AimMath.Status.NO_RECENT_VISION, AimMath.status(true, false, 60, 24, 120, true, true, true, false, false));
        assertEquals(AimMath.Status.OK, AimMath.status(true, false, 60, 24, 120, true, true, true, false, true));
        assertEquals(AimMath.Status.OK, AimMath.status(true, false, 60, 24, 120, true, true, true, true, false));
    }
}
