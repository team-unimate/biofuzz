package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;

public class ShootWhenReady extends SequentialCommandGroup {
    public static long TIMEOUT_MS = 1500;

    public ShootWhenReady(Turret turret) {
        addCommands(
                new WaitUntilCommand(turret::okToShoot).withTimeout(TIMEOUT_MS),
                new TransferSequence()
        );
    }
}
