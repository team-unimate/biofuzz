package org.firstinspires.ftc.teamcode.opmodes.Autonomous;

import static org.junit.Assert.assertEquals;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.junit.Test;

public class AutonomousPathsTest {
    private static final double EPSILON = 1e-9;

    @Test
    public void autoPathSegmentsShareEndpointsAndHeadings() {
        autoPath auto = new autoPath();
        auto.buildPoses();
        Path[] paths = {
                auto.initGarden(), auto.gardenToShoot1(), auto.shoot1ToHive1(),
                auto.hive1ToShoot2(), auto.shoot2ToHive2(), auto.hive2ToParking()
        };

        for (int i = 1; i < paths.length; i++) {
            assertSamePose(paths[i - 1].endPose(), paths[i].get(0));
        }
    }

    private static void assertSamePose(Pose expected, Pose actual) {
        assertEquals(expected.x(), actual.x(), EPSILON);
        assertEquals(expected.y(), actual.y(), EPSILON);
        double headingDelta = actual.heading() - expected.heading();
        assertEquals(0, Math.IEEEremainder(headingDelta, 2 * Math.PI), EPSILON);
    }
}
