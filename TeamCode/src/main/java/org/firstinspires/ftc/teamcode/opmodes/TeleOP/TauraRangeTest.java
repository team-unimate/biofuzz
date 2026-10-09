package org.firstinspires.ftc.teamcode.opmodes.TeleOP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.TauraServo;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretConstants;

@Config
@TeleOp(name = "Taura Range Test", group = "Tuning")
public class TauraRangeTest extends LinearOpMode {
    public static String SERVO_NAME = "turret";
    public static String ENCODER_NAME = "encoder";
    public static double PWM_LOWER_US = 500;
    public static double PWM_UPPER_US = 2500;
    public static double POSITION = 0.5;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        ServoImplEx servo = hardwareMap.get(ServoImplEx.class, SERVO_NAME);
        servo.setPwmRange(new PwmControl.PwmRange(PWM_LOWER_US, PWM_UPPER_US));
        AnalogInput encoder = hardwareMap.get(AnalogInput.class, ENCODER_NAME);
        TauraServo taura = new TauraServo(servo);
        taura.setAnalogFeedbackSensor(encoder);

        double posA = Double.NaN, uniA = Double.NaN, posB = Double.NaN, uniB = Double.NaN;

        waitForStart();
        while (opModeIsActive()) {
            servo.setPosition(POSITION);
            double universal = taura.getUniversalPosition();
            if (gamepad1.a) {
                posA = POSITION;
                uniA = universal;
            }
            if (gamepad1.b) {
                posB = POSITION;
                uniB = universal;
            }
            double deltaPosition = posB - posA;
            double analogSlope = (uniB - uniA) / deltaPosition;

            telemetry.addData("Controls", "set POSITION on Dashboard, A = save point A, B = save point B");
            telemetry.addData("PWM us", PWM_LOWER_US + " to " + PWM_UPPER_US);
            telemetry.addData("Position", POSITION);
            telemetry.addData("Voltage", encoder.getVoltage());
            telemetry.addData("Analog universal 0..1", universal);
            telemetry.addData("Point A position / universal", posA + " / " + uniA);
            telemetry.addData("Point B position / universal", posB + " / " + uniB);
            telemetry.addData("Analog slope (1 = analog matches PWM range)", analogSlope);
            telemetry.addData("Position change B - A", deltaPosition);
            telemetry.addData("SERVO_RANGE_DEG", "turret deg moved (protractor) * " + TurretConstants.GEAR_RATIO
                    + " / " + deltaPosition);
            telemetry.update();
        }
    }
}
