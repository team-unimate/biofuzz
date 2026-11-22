package org.firstinspires.ftc.teamcode.subsystems.intakeFlower;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.TauraServo;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;

public class IntakeFlower extends SubsystemBase {
    public enum State {
        UP, DOWN, TUNE
    }

    private final Intake intake;
    TauraServo IntakeFlower1;
    private State state = State.UP;

    public IntakeFlower(HardwareMap hardwareMap){
        this(hardwareMap, null);
    }

    public IntakeFlower(HardwareMap hardwareMap, Intake intake){
        this.intake = intake;
        IntakeFlower1 = new TauraServo(hardwareMap.get(Servo.class, IntakeFlowerConstants.HMIntakeFlower1));
    }

    public void setState(State state){
        this.state = state;
        apply();
        if (intake == null) return;
        if (state == State.DOWN) intake.setState(Intake.State.INTAKE);
        else if (state == State.UP) intake.setState(Intake.State.OFF);
    }

    public State getState(){
        return state;
    }

    private void apply(){
        if (state == State.DOWN) {
            IntakeFlower1.setPosition(IntakeFlowerConstants.IntakeFlowerOn1);

        } else if (state == State.TUNE) {
            IntakeFlower1.setPosition(IntakeFlowerConstants.IntakeFlowerTune);
        } else {
            IntakeFlower1.setPosition(IntakeFlowerConstants.IntakeFlowerOff);
        }
    }

    @Override
    public void periodic(){
        apply();
    }

    public double getPosition(){
        return IntakeFlower1.getPosition();
    }
}
