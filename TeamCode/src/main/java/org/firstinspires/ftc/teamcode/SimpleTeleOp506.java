package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Simple teleop for testing Drivetrain with mecanum/tank drive. Toggle with start button on gamepad 1
 *
 * @author Oscar Kosar-Kosarewicz
 * @version 10/22/16
 */
@TeleOp(name = "Simple TeleOp", group = "Iterative Opmode")
public class SimpleTeleOp506 extends OpMode {

    Hardware506 robot;
    boolean backPreviousState;
    boolean aPreviousState;
    boolean servoIsDown;

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
        aPreviousState = false;
        servoIsDown = false;
        currentDriveMode = DriveMode.MECANUM_RELATIVE_TO_DRIVER;
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
                robot.setDriveMode(Hardware506.DriveMode.MECANUM);
                robot.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
                break;
            case MECANUM_RELATIVE_TO_DRIVER:
                telemetry.addData("Drive Mode", "Mecanum relative to driver");
                robot.setDriveMode(Hardware506.DriveMode.MECANUM_WITH_GYRO);
                robot.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
                break;
            case TANK_DRIVE:
                telemetry.addData("Drive Mode", "Tank Drive");
                double leftPower = -gamepad1.left_stick_y;
                double rightPower = -gamepad1.right_stick_y;
                robot.leftFrontMotor.setPower(leftPower);
                robot.leftRearMotor.setPower(leftPower);
                robot.rightFrontMotor.setPower(rightPower);
                robot.rightRearMotor.setPower(rightPower);
                break;
        }

        boolean aButtonState = gamepad1.a;
        if (aButtonState != aPreviousState) {
            if (aButtonState) {
                servoIsDown = !servoIsDown;
            }
        }
        aPreviousState = aButtonState;
        robot.setArmPositionDown(servoIsDown);

        telemetry.addData("Gyro Heading", robot.gyro.getHeading());
        telemetry.addData("Color Detected", robot.getBeaconColor());
        telemetry.addData("Color Detected RGB", String.format("%4d, %4d, %4d", robot.beaconColorSensor.red(), robot.beaconColorSensor.green(), robot.beaconColorSensor.blue()));
        telemetry.addData("Line Detected", robot.isLineDetected());
        telemetry.addData("Ultrasonic left, right", String.format("%4.2f, %4.2f", robot.leftUltrasonic.getUltrasonicLevel(), robot.rightUltrasonic.getUltrasonicLevel()));
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
