package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.gate.GateClose;
import org.firstinspires.ftc.teamcode.commands.gate.GateOpen;
import org.firstinspires.ftc.teamcode.commands.intake.intakeOff;
import org.firstinspires.ftc.teamcode.commands.intake.intakeOn;
import org.firstinspires.ftc.teamcode.subsystems.gate.Gate;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;

public class TransferSequence extends SequentialCommandGroup {
    public TransferSequence(Intake intake, Gate gate) {
        addCommands(
                new GateClose(gate),
                new WaitCommand(200),
                new intakeOn(intake),
                new GateOpen(gate),
                new WaitCommand(1500),
                new intakeOff(intake),
                new GateClose(gate)

        );
        addRequirements(intake, gate);
    }
}
