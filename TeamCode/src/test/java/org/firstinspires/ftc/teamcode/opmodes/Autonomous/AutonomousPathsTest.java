package org.firstinspires.ftc.teamcode.opmodes.Autonomous;

import static org.junit.Assert.assertEquals;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.field.Field;
import org.junit.Test;

public class AutonomousPathsTest {
    private static final double EPSILON = 1e-9;

    @Test
    public void autoPathSegmentsShareEndpointsAndHeadingsForBothAlliances() {
        for (Field.Alliance alliance : Field.Alliance.values()) {
            autoPath auto = new autoPath();
            auto.buildPoses(alliance);
            Path[] paths = {
                    auto.initGarden(), auto.gardenToShoot1(), auto.shoot1ToHive1(),
                    auto.hive1ToShoot2(), auto.shoot2ToHive2(), auto.hive2ToParking()
            };

            for (int i = 1; i < paths.length; i++) {
                assertSamePose(paths[i - 1].endPose(), paths[i].get(0));
            }
        }
    }

    @Test
    public void autoPathRedGardenAndParkingAreOnOppositeEndsFromBlue() {
        autoPath auto = new autoPath();
        auto.buildPoses(Field.Alliance.RED);

        assertSamePose(new Pose(7.0501, 7.3216, Math.PI), auto.initGarden().endPose());
        assertSamePose(new Pose(12.512, 89.5398, Math.PI / 2), auto.hive2ToParking().endPose());
    }

    @Test
    public void testMatchRotatesTheWholeParkingCurveForRed() {
        testMatch auto = new testMatch();
        auto.buildPoses(Field.Alliance.BLUE);
        Path blue = auto.startToPark();
        auto.buildPoses(Field.Alliance.RED);
        Path red = auto.startToPark();

        for (double t : new double[] {0, 0.5, 1}) {
            Pose bluePose = blue.get(t);
            Pose expectedRed = new Pose(141.5 - bluePose.x(), 141.5 - bluePose.y(),
                    bluePose.heading() + Math.PI);
            assertSamePose(expectedRed, red.get(t));
        }
    }

    private static void assertSamePose(Pose expected, Pose actual) {
        assertEquals(expected.x(), actual.x(), EPSILON);
        assertEquals(expected.y(), actual.y(), EPSILON);
        double headingDelta = actual.heading() - expected.heading();
        assertEquals(0, Math.IEEEremainder(headingDelta, 2 * Math.PI), EPSILON);
    }
}
