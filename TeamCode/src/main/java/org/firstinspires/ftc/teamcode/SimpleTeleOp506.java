package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Simple teleop for testing Drivetrain with mecanum/tank drive. Toggle with start button on gamepad 1
 * @author Oscar Kosar-Kosarewicz
 * @version 9/12/16
 */
@TeleOp(name="Simple TeleOp", group="Iterative Opmode")
public class SimpleTeleOp506 extends OpMode{

    Hardware506 robot;
    boolean startPreviousState;
    boolean isMecanumDriveActive;

    @Override
    public void init() {
        robot = new Hardware506(hardwareMap);
        startPreviousState = false;
        isMecanumDriveActive = true;
        telemetry.addData("State", "Initialized");
    }

    @Override
    public void loop() {
        boolean startButtonState = gamepad1.start;
        if (startButtonState != startPreviousState){
            if (startButtonState){
                isMecanumDriveActive = !isMecanumDriveActive;
            }
        }
        startPreviousState =startButtonState;

        if (isMecanumDriveActive){
            robot.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            telemetry.addData("Drive Mode", "Mecanum");
        }
        else {
            double leftPower = gamepad1.left_stick_y;
            double rightPower = gamepad1.right_stick_y;
            robot.leftFrontMotor.setPower(leftPower);
            robot.leftRearMotor.setPower(leftPower);
            robot.rightFrontMotor.setPower(rightPower);
            robot.rightRearMotor.setPower(rightPower);
            telemetry.addData("Drive Mode", "Tank Drive");
        }
        telemetry.addData("State", "Running");
    }
}
