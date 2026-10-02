package org.firstinspires.ftc.teamcode.opmodes.Autonomous;

import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "autoPath", group = "Autonomous")
public class autoPath extends CommandOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(86.7, 133.8, 0);
    private final Pose initGardenStart = poseFactory.of(86.7    , 133.8594, 0);
    private final Pose initGarden = poseFactory.of(134.4499, 134.1784, 0);
    private final Pose gardenToShoot1 = poseFactory.of(97.9289, 23.6729, -100);
    private final Pose shoot1ToHive1 = poseFactory.of(96.9, 13.0, -100.0781);
    private final Pose hive1ToShoot2Start = poseFactory.of(95.9072, 12.2983, -100);
    private final Pose hive1ToShoot2 = poseFactory.of(100.3939, 31.8151, 30);
    private final Pose shoot2ToHive2 = poseFactory.of(127 , 92.6, 10);
    private final Pose hive2ToParkingStart = poseFactory.of(128.4241, 92.6001, 0);
    private final Pose hive2ToParking = poseFactory.of(128.988, 51.9602, -90);

    // Autonomous routine
    public Command autoRoutine() {
        return new SequentialCommandGroup(
                new FollowPathCommand(follower, initGarden()),
                new FollowPathCommand(follower, gardenToShoot1()),
                new FollowPathCommand(follower, shoot1ToHive1()),
                new FollowPathCommand(follower, hive1ToShoot2()),
                new FollowPathCommand(follower, shoot2ToHive2()),
                new FollowPathCommand(follower, hive2ToParking())
        );
    }

    @Override
    public void initialize() {
        super.reset();

        follower = Constants.createFollower(hardwareMap);
        follower.setPose(start);
        follower.update();
    }

    @Override
    public void preRun() {
        schedule(autoRoutine());
    }

    @Override
    public void run() {
        super.run();
        follower.update();

        telemetry.addData("x", follower.pose().x());
        telemetry.addData("y", follower.pose().y());
        telemetry.addData("heading", follower.pose().heading());

        if (follower.currentPath() != null) {
            telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
            telemetry.addData("Path number", follower.pathIndex());
        }

        telemetry.update();
    }
    public Path initGarden() {
        return Paths.line(initGardenStart, initGarden).linear(initGardenStart, initGarden);
    }

    public Path gardenToShoot1() {
        return Paths.line(initGarden, gardenToShoot1).linear(initGarden, gardenToShoot1);
    }

    public Path shoot1ToHive1() {
        return Paths.line(gardenToShoot1, shoot1ToHive1).tangent();
    }

    public Path hive1ToShoot2() {
        return Paths.line(hive1ToShoot2Start, hive1ToShoot2).linear(hive1ToShoot2Start, hive1ToShoot2);
    }

    public Path shoot2ToHive2() {
        return Paths.line(hive1ToShoot2, shoot2ToHive2).linear(hive1ToShoot2, shoot2ToHive2);
    }

    public Path hive2ToParking() {
        return Paths.line(hive2ToParkingStart, hive2ToParking).linear(hive2ToParkingStart, hive2ToParking);
    }
}