package org.firstinspires.ftc.teamcode.field;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

public class Field {
    public static final double WIDTH = 141.5;
    public static final double CENTER_X = WIDTH / 2;
    public static final double CENTER_Y = WIDTH / 2;

    // Placeholder coordinates - update with measured field positions.
    public static final Pose RED_CELL_TOP = new Pose(58, 83, 0);
    public static final Pose RED_CELL_BOTTOM = new Pose(58, 59, 0);
    public static final Pose BLUE_CELL_TOP = new Pose(84.5, 75.5, 0);
    public static final Pose BLUE_CELL_BOTTOM = new Pose(84.5, 60.1, 0);


    public enum Alliance {
        RED, BLUE
    }

    /**
     * Creates poses from blue route coordinates and headings in degrees. The BIOBUZZ gardens
     * and loading zones are on opposite ends of the field, so red routes use a half-turn.
     */
    public static PoseFactory blueRoutePoses(Alliance alliance) {
        PoseFactory poses = PoseFactory.degrees();
        return alliance == Alliance.RED ? poses.mirrorAroundPoint(CENTER_X, CENTER_Y) : poses;
    }

    /**
     * The given alliance's goal closest to the pose, picking top/bottom by which half of the field
     * it's on (y vs {@link #CENTER_Y}).
     */
    public static Pose getCell(Alliance alliance, Pose robotPose) {
        boolean top = robotPose.y() >= CENTER_Y;
        if (alliance == Alliance.RED) {
            return top ? RED_CELL_TOP : RED_CELL_BOTTOM;
        }
        return top ? BLUE_CELL_TOP : BLUE_CELL_BOTTOM;
    }

    /**
     * The nearest goal to the pose, guessing the alliance from which half of the field it's on (x vs
     * {@link #CENTER_X}). Useful when no alliance has been selected ahead of time.
     */
    public static Pose getCell(Pose robotPose) {
        return getCell(robotPose.x() < CENTER_X ? Alliance.RED : Alliance.BLUE, robotPose);
    }
}
