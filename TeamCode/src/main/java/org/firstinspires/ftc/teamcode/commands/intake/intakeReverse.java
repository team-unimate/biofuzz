package org.firstinspires.ftc.teamcode.commands.intake;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;

public class intakeReverse extends CommandBase {
    private final Intake intake;
    public intakeReverse(Intake intake) {
        this.intake = intake;
    }

    @Override
    public void initialize(){
        intake.intakeReverse();
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
