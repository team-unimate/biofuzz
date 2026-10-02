package org.firstinspires.ftc.teamcode.commands.field;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.field.Field;

import java.util.function.Supplier;

/**
 * Locks the robot's pose so it faces the given alliance's cell
 * ({@link Field#getCell(Field.Alliance, Pose)}), holding that fixed pose via {@link Follower#hold(Pose)}
 * so the follower's translational/heading controllers fight back if something pushes the robot off it.
 * The aim pose is computed once, when tracking starts - it does not re-aim if the robot is pushed to a
 * spot where a different heading would face the cell, since that would cancel out the push correction.
 * Never finishes on its own - bind it with {@code whileHeld}/{@code whileActiveContinuous}.
 */
public class TrackCellCommand extends CommandBase {
    private final Follower follower;
    private final Field.Alliance alliance;
    private final Supplier<Pose> robotPoseSupplier;
    private Pose lockedPose;

    public TrackCellCommand(Follower follower, Field.Alliance alliance) {
        this(follower, alliance, follower::pose);
    }

    public TrackCellCommand(Follower follower, Field.Alliance alliance, Supplier<Pose> robotPoseSupplier) {
        this.follower = follower;
        this.alliance = alliance;
        this.robotPoseSupplier = robotPoseSupplier;
    }

    @Override
    public void initialize() {
        Pose current = robotPoseSupplier.get();
        Pose target = Field.getCell(alliance, current);
        double headingToTarget = Math.atan2(target.y() - current.y(), target.x() - current.x());
        lockedPose = current.withHeading(headingToTarget);

        follower.algorithm().reset();
        follower.hold(lockedPose);
    }

    @Override
    public void execute() {
        follower.hold(lockedPose);
    }
}
