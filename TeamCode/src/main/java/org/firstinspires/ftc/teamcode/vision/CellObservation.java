package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.teamcode.field.HiveCells;

public class CellObservation {
    public final HiveCells.Cell cell;
    public final int[] tagIds;

    public final double forward;
    public final double left;

    public final double bearing;

    public final double distance;

    public final double rowYaw;
    public final HiveCells.State state;

    public final double fieldX;
    public final double fieldY;
    public final long captureNanos;
    public final double turretAngleAtCapture;
    public final double headingAtCapture;

    public CellObservation(HiveCells.Cell cell, int[] tagIds, double forward, double left,
                           double rowYaw, HiveCells.State state, double fieldX, double fieldY,
                           long captureNanos, double turretAngleAtCapture, double headingAtCapture) {
        this.cell = cell;
        this.tagIds = tagIds;
        this.forward = forward;
        this.left = left;
        this.bearing = Math.atan2(left, forward);
        this.distance = Math.hypot(forward, left);
        this.rowYaw = rowYaw;
        this.state = state;
        this.fieldX = fieldX;
        this.fieldY = fieldY;
        this.captureNanos = captureNanos;
        this.turretAngleAtCapture = turretAngleAtCapture;
        this.headingAtCapture = headingAtCapture;
    }

    public int tagCount() {
        return tagIds.length;
    }
}
