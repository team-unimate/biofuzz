package org.firstinspires.ftc.teamcode.vision;

/** A single AprilTag's known field position, in the same frame/units as Pedro Pathing's Follower pose (inches). */
public class AprilTagTarget {
    public final int id;
    public final double x;
    public final double y;

    public AprilTagTarget(int id, double x, double y) {
        this.id = id;
        this.x = x;
        this.y = y;
    }
}
