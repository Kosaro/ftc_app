package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import static org.firstinspires.ftc.teamcode.Hardware506.ColorDetected.NONE;
import static org.firstinspires.ftc.teamcode.Hardware506.ColorDetected.RED;

/**
 * Autonomous without vuforia, using two ultrasonic sensors and two color sensors(work in progress)
 *
 * @author Oscar Kosar-Kosarewicz
 * @version 10/12/16
 */
@Autonomous(name = "Ultrasonic Autonomous", group = "Autonomous")
//@Disabled
public class UltrasonicAutonomous extends LinearOpMode {
    Hardware506 robot;
    double sidePower;
    double forwardPower;
    double rotationPower;


    enum State {
        SEARCH_FOR_BEACON,
        FOLLOW_LINE,
        DETECT_COLOR,
        TURN,
        PUSH_BUTTON,
        MOVE_LEFT,
        STOP
    }

    State currentState;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Hardware506(hardwareMap);
        robot.setDriveMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.setDriveMode(Hardware506.DriveMode.MECANUM_WITH_GYRO);
        currentState = State.SEARCH_FOR_BEACON;
        int beaconsPushed = 0;
        Hardware506.ColorDetected colorDetected = NONE;
        double time = getRuntime();
        robot.gyro.calibrate();
        telemetry.addData("State", "Calibrating Gyro");
        telemetry.update();
        while (robot.gyro.isCalibrating()) {
            idle();
        }
        telemetry.addData("State", "Initialized");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            double distance = robot.getUltrasonicAverageDistance();
            switch (currentState) {
                case SEARCH_FOR_BEACON:
                    rotationPower = 0;
                    if (distance > 50) {
                        forwardPower = .3;
                        if (robot.gyro.getHeading() > 5)
                            rotationPower = .3;
                        else if (robot.gyro.getHeading() < 5)
                            rotationPower = -.3;
                        else
                            rotationPower = 0;
                    } else if (distance > 25) {
                        forwardPower = .3;
                        if (robot.gyro.getHeading() > 5)
                            rotationPower = .3;
                        else if (robot.gyro.getHeading() < 5)
                            rotationPower = -.3;
                        else
                            rotationPower = 0;
                    } else if (distance > 15) {
                        robot.setDriveMode(Hardware506.DriveMode.MECANUM);
                        forwardPower = .1;
                        rotationPower = alignToWall();
                        if (robot.isLineDetected()) {
                            currentState = State.FOLLOW_LINE;
                        } else sidePower = -.2;
                    } else if (distance < 15) {
                        forwardPower = -.2;
                        rotationPower = alignToWall();
                        if (robot.isLineDetected()) {
                            currentState = State.FOLLOW_LINE;
                        } else sidePower = -.2;
                    }
                    telemetry.addData("State", "Searching White Line");
                    break;
                case FOLLOW_LINE:
                    rotationPower = alignToWall();
                    if (robot.getUltrasonicAverageDistance() < 5)
                        forwardPower = -.1;
                    else if (robot.getUltrasonicAverageDistance() > 7)
                        forwardPower = .1;
                    else if (rotationPower == 0) {
                        forwardPower = 0;
                        sidePower = 0;
                        currentState = State.DETECT_COLOR;
                        robot.gyro.centerOffset();
                        robot.setDriveMode(Hardware506.DriveMode.MECANUM_WITH_GYRO);
                    }
                    if (robot.isLineDetected()) {
                        sidePower = .2;
                    } else
                        sidePower = -.2;
                    telemetry.addData("State", "Following Line");
                    break;
                case DETECT_COLOR:
                    colorDetected = Hardware506.ColorDetected.NONE;
                    if (robot.getBeaconColor() == Hardware506.ColorDetected.BLUE) {
                        colorDetected = Hardware506.ColorDetected.BLUE;
                    } else if (robot.getBeaconColor() == RED) {
                        colorDetected = RED;
                    } else
                        colorDetected = NONE;
                    currentState = State.TURN;
                    telemetry.addData("State", "Detect Color");
                    break;
                case TURN:
                    switch (colorDetected) {
                        case RED:
                            if (robot.gyro.getHeading() < 20)
                                rotationPower = -.3;
                            else
                                currentState = State.PUSH_BUTTON;
                            break;
                        case BLUE:
                            if (robot.gyro.getHeading() > -20)
                                rotationPower = .3;
                            else
                                currentState = State.PUSH_BUTTON;
                            break;
                        case NONE:
                            currentState = State.PUSH_BUTTON;
                            break;
                    }
                    time = getRuntime();
                    telemetry.addData("State", "Turn");
                    break;
                case PUSH_BUTTON:
                    if (getRuntime() < time + 2) {
                        forwardPower = 0;
                        robot.setDriveMode(Hardware506.DriveMode.MECANUM);
                        time = getRuntime();
                        beaconsPushed++;
                        if (beaconsPushed < 2)
                            currentState = State.MOVE_LEFT;
                        else
                            currentState = State.STOP;
                    } else {
                        forwardPower = .3;
                    }
                    telemetry.addData("State", "Push Button");
                    break;
                case MOVE_LEFT:
                    if (getRuntime() < time + .5) {
                        if (distance > 15) {
                            forwardPower = .3;
                            rotationPower = alignToWall();
                        } else if (distance < 15) {
                            forwardPower = -.2;
                        }
                        sidePower = -1;
                    } else {
                        currentState = State.SEARCH_FOR_BEACON;
                    }
                    break;
                case STOP:
                    telemetry.addData("State", "Stop");
                    sidePower = 0;
                    rotationPower = 0;
                    forwardPower = 0;
                    break;
            }


            telemetry.addData("Ultrasonic Left", robot.leftUltrasonic.getUltrasonicLevel());
            telemetry.addData("Ultrasonic Right", robot.rightUltrasonic.getUltrasonicLevel());
            telemetry.addData("Ultrasonic Average", distance);
            telemetry.addData("Gyro Heading", robot.gyro.getHeading());
            telemetry.addData("Line Detected", robot.isLineDetected());
            telemetry.addData("Beacon Color Detected", robot.getBeaconColor());

            robot.drive(forwardPower, sidePower, rotationPower);

            telemetry.update();
            idle();
        }
    }


    /**
     * Returns necessary turn power to align to the wall
     *
     * @return turn power
     */

    public double alignToWall() {
        double left = robot.leftUltrasonic.getUltrasonicLevel();
        double right = robot.rightUltrasonic.getUltrasonicLevel();

        if (Math.abs(left - right) > 2) {
            if (left > right)
                return .1;
            else
                return -.1;
        }
        return 0;
    }


}
