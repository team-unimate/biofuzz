package org.firstinspires.ftc.teamcode.commands.field;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.util.PosePersistency;

@Config
public class ResetPose extends InstantCommand {
    public static double X_IN = 83.4;
    public static double Y_IN = 8;
    public static double HEADING_DEG = 90;

    public ResetPose(Follower follower) {
        super(() -> {
            Pose pose = pose();
            follower.setPose(pose);
            PosePersistency.savePose(pose);
        });
    }

    public static Pose pose() {
        return PoseFactory.degrees().of(X_IN, Y_IN, HEADING_DEG);
    }
}
