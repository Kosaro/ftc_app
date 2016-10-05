package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.util.Range;

/**
 * Simple teleop for testing Drivetrain with mecanum/tank drive. Toggle with start button on gamepad 1
 *
 * @author Oscar Kosar-Kosarewicz
 * @version 9/21/16
 */
@TeleOp(name = "Simple TeleOp", group = "Iterative Opmode")
public class SimpleTeleOp506 extends OpMode {

    Hardware506 robot;
    boolean startPreviousState;

    enum DriveMode {
        TANK_DRIVE,
        MECANUM,
        MECANUM_RELATIVE_TO_DRIVER
    }

    DriveMode currentDriveMode;

    @Override
    public void init() {
        robot = new Hardware506(hardwareMap);
        startPreviousState = false;
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
        boolean startButtonState = gamepad1.start;
        if (startButtonState != startPreviousState) {
            if (startButtonState) {
                switchDriveMode();
            }
        }
        startPreviousState = startButtonState;

        switch (currentDriveMode) {
            case MECANUM:
                telemetry.addData("Drive Mode", "Mecanum");
                robot.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
                break;
            case MECANUM_RELATIVE_TO_DRIVER:
                telemetry.addData("Drive Mode", "Mecanum relative to driver");
                double directionRelativeToRobot;
                if (gamepad1.left_stick_y == 0){
                    directionRelativeToRobot = Math.PI / 2;
                }else
                directionRelativeToRobot = Math.atan(gamepad1.left_stick_x / gamepad1.left_stick_y);
                double velocity = Math.sqrt(Math.pow(gamepad1.left_stick_y, 2) + Math.pow(gamepad1.left_stick_x, 2));
                double adjustedDirection = directionRelativeToRobot - robot.gyro.getHeading() * Math.PI / 180;
                robot.drive(velocity * Math.cos(adjustedDirection), velocity * Math.sin(adjustedDirection), gamepad1.right_stick_x);
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
