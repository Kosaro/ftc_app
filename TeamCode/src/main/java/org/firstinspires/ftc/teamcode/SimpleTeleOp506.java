package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Simple teleop for testing Drivetrain with mecanum/tank drive. Toggle with start button on gamepad 1
 *
 * @author Oscar Kosar-Kosarewicz
 * @version 9/21/16
 */
@TeleOp(name = "Simple TeleOp", group = "Iterative Opmode")
public class SimpleTeleOp506 extends OpMode {

    Hardware506 robot;
    boolean backPreviousState;

    enum DriveMode {
        TANK_DRIVE,
        MECANUM,
        MECANUM_RELATIVE_TO_DRIVER
    }

    DriveMode currentDriveMode;

    @Override
    public void init() {
        robot = new Hardware506(hardwareMap);
        backPreviousState = false;
        currentDriveMode = DriveMode.MECANUM;
        switchDriveMode();
        robot.gyro.calibrate();
        telemetry.addData("State", "Calibrating Gyro");
    }

    @Override
    public void init_loop() {
        if (!robot.gyro.isCalibrating()) {
            telemetry.addData("State", "Initialized");
        }
    }

    @Override
    public void loop() {
        boolean backButtonState = gamepad1.back;
        if (backButtonState != backPreviousState) {
            if (backButtonState) {
                switchDriveMode();
            }
        }
        backPreviousState = backButtonState;

        switch (currentDriveMode) {
            case MECANUM:
                telemetry.addData("Drive Mode", "Mecanum");
                robot.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
                break;
            case MECANUM_RELATIVE_TO_DRIVER:
                telemetry.addData("Drive Mode", "Mecanum relative to driver");
                double directionRelativeToRobot;
                if (gamepad1.left_stick_y == 0) {
                    if (gamepad1.left_stick_x > 0)
                        directionRelativeToRobot = Math.PI / 2;
                    else
                        directionRelativeToRobot = -Math.PI / 2;
                } else
                    directionRelativeToRobot = Math.atan(gamepad1.left_stick_x / -gamepad1.left_stick_y);
                double velocity = Math.sqrt(Math.pow(gamepad1.left_stick_y, 2) + Math.pow(gamepad1.left_stick_x, 2));
                if (-gamepad1.left_stick_y < 0) {
                    directionRelativeToRobot += Math.PI;
                }
                double adjustedDirection = directionRelativeToRobot - robot.gyro.getHeading() * Math.PI / 180;
                double forwardPower = velocity * Math.cos(adjustedDirection);
                double sidePower = velocity * Math.sin(adjustedDirection);
                robot.drive(forwardPower, sidePower, gamepad1.right_stick_x);
                break;
            case TANK_DRIVE:
                telemetry.addData("Drive Mode", "Tank Drive");
                double leftPower = gamepad1.left_stick_y;
                double rightPower = gamepad1.right_stick_y;
                robot.leftFrontMotor.setPower(leftPower);
                robot.leftRearMotor.setPower(leftPower);
                robot.rightFrontMotor.setPower(rightPower);
                robot.rightRearMotor.setPower(rightPower);
                break;


        }
        telemetry.addData("State", "Running");
    }

    @Override
    public void stop() {
        telemetry.addData("State", "Stopped");
    }

    void switchDriveMode() {
        switch (currentDriveMode) {
            case TANK_DRIVE:
                currentDriveMode = DriveMode.MECANUM;
                break;
            case MECANUM:
                currentDriveMode = DriveMode.MECANUM_RELATIVE_TO_DRIVER;
                break;
            case MECANUM_RELATIVE_TO_DRIVER:
                currentDriveMode = DriveMode.TANK_DRIVE;
                break;
            default:
                currentDriveMode = DriveMode.MECANUM;
                break;
        }
    }
}
