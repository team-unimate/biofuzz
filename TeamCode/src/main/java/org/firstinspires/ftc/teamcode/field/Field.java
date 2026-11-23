package org.firstinspires.ftc.teamcode.field;

import com.pedropathing.math.Pose;

public class Field {
    public static final double WIDTH = 141.5;
    public static final double CENTER_X = WIDTH / 2;
    public static final double CENTER_Y = WIDTH / 2;

    public static final Pose CELL_TOP = new Pose(84.2, 85.0, 0);
    public static final Pose CELL_BOTTOM = new Pose(84.2, 55.7, 0);

    public static Pose getCell(Pose robotPose) {
        return robotPose.y() >= CENTER_Y ? CELL_TOP : CELL_BOTTOM;
    }
}
