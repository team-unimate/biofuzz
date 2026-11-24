package org.firstinspires.ftc.teamcode.opmodes.TeleOP;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretHistory;
import org.firstinspires.ftc.teamcode.subsystems.vision.VisionSubsystem;

@TeleOp(name = "Vision Aim Test")
public class VisionAimTestTeleOp extends CommandOpMode {
    private static final Pose START_POSE = new Pose(133, 133.8594, Math.toRadians(-90));

    private Field.Alliance alliance = Field.Alliance.BLUE;
    private Follower follower;
    private Turret turret;
    private VisionSubsystem vision;

    @Override
    public void initialize() {
        super.reset();

        follower = Constants.createFollower(hardwareMap);
        follower.setPose(START_POSE);

        TurretHistory history = new TurretHistory();
        vision = new VisionSubsystem(hardwareMap, () -> alliance);
        turret = new Turret(hardwareMap, history);
        turret.setAlliance(alliance);
        turret.setState(Turret.State.AIM);

        register(vision, turret);
    }

    @Override
    public void initialize_loop() {
        updateAlliance();
        telemetry.addData("Controls", "X = BLUE, B = RED");
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Hive tipped", vision.isHiveTipped());
        telemetry.update();
    }

    @Override
    public void run() {
        super.run();
        updateAlliance();

        follower.manual(ManualDrive.fieldCentric(
                gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                follower.pose().heading() + (alliance == Field.Alliance.RED ? Math.PI : 0)
        ));
        follower.update();

        telemetry.addData("Alliance", alliance);
        telemetry.addData("Hive tipped", vision.isHiveTipped());
        telemetry.addData("Turret state", turret.getState());
        telemetry.addData("Turret target deg", Math.toDegrees(turret.getTarget()));
        telemetry.addData("Turret angle deg", Math.toDegrees(turret.getAngle()));
        telemetry.addData("Robot X", follower.pose().x());
        telemetry.addData("Robot Y", follower.pose().y());
        telemetry.update();
    }

    private void updateAlliance() {
        if (gamepad1.x) alliance = Field.Alliance.BLUE;
        else if (gamepad1.b) alliance = Field.Alliance.RED;
        turret.setAlliance(alliance);
    }
}
