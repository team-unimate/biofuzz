package org.firstinspires.ftc.teamcode.field;

import static org.junit.Assert.assertEquals;

import com.pedropathing.math.Pose;

import org.junit.Test;

public class FieldTest {
    private static final double EPSILON = 1e-9;

    @Test
    public void blueRouteCoordinatesAndHeadingArePreserved() {
        Pose pose = Field.blueRoutePoses(Field.Alliance.BLUE).of(130, 120, 30);

        assertEquals(130, pose.x(), EPSILON);
        assertEquals(120, pose.y(), EPSILON);
        assertEquals(Math.toRadians(30), pose.heading(), EPSILON);
    }

    @Test
    public void redRouteRotatesAcrossTheCenterOfThePlayableField() {
        Pose pose = Field.blueRoutePoses(Field.Alliance.RED).of(130, 120, 30);

        assertEquals(11.5, pose.x(), EPSILON);
        assertEquals(21.5, pose.y(), EPSILON);
        assertEquals(Math.toRadians(210), pose.heading(), EPSILON);
    }
}
