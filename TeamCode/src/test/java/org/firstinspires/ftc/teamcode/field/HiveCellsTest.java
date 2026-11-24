package org.firstinspires.ftc.teamcode.field;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertSame;

import com.pedropathing.math.Pose;

import org.junit.Test;

public class HiveCellsTest {
    @Test
    public void cellFollowsFieldHalf() {
        assertEquals(HiveCells.Cell.TOP, HiveCells.forRobot(new Pose(100, Field.CENTER_Y, 0)));
        assertEquals(HiveCells.Cell.BOTTOM, HiveCells.forRobot(new Pose(100, Field.CENTER_Y - 0.1, 0)));
        assertEquals(HiveCells.Cell.TOP, HiveCells.forRobot(new Pose(20, 130, 0)));
        assertEquals(HiveCells.Cell.BOTTOM, HiveCells.forRobot(new Pose(20, 10, 0)));
    }

    @Test
    public void cellMatchesFieldGetCell() {
        Pose top = new Pose(20, 130, 0);
        Pose bottom = new Pose(20, 10, 0);
        assertSame(Field.getCell(top), HiveCells.forRobot(top).pose);
        assertSame(Field.getCell(bottom), HiveCells.forRobot(bottom).pose);
    }

    @Test
    public void tagsMapToTheirCell() {
        assertEquals(HiveCells.Cell.TOP, HiveCells.forTag(38));
        assertEquals(HiveCells.Cell.BOTTOM, HiveCells.forTag(45));
        assertNull(HiveCells.forTag(37));
        assertNull(HiveCells.forTag(46));
    }
}
