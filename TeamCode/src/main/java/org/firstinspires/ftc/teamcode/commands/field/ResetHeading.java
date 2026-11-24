package org.firstinspires.ftc.teamcode.commands.field;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.util.PosePersistency;

import java.util.function.DoubleSupplier;

public class ResetHeading extends InstantCommand {
    public ResetHeading(Follower follower, DoubleSupplier headingRadians) {
        super(() -> {
            Pose current = follower.pose();
            Pose pose = new Pose(current.x(), current.y(), headingRadians.getAsDouble());
            follower.setPose(pose);
            PosePersistency.savePose(pose);
        });
    }
}
