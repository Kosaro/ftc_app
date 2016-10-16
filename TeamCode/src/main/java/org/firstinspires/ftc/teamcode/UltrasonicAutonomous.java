package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import static org.firstinspires.ftc.teamcode.Hardware506.ColorDetected.NONE;

/**
 * Autonomous without vuforia, using two ultrasonic sensors and two color sensors(work in progress)
 *
 * @author Oscar Kosar-Kosarewicz
 * @version 10/16/16
 */
@Autonomous(name = "Ultrasonic Autonomous", group = "Autonomous")
//@Disabled
public class UltrasonicAutonomous extends LinearOpMode {
    Hardware506 robot;
    double sidePower;
    double forwardPower;
    double rotationPower;


    enum State {
        DRIVE_AWAY_FROM_WALL("Drive Away from Wall"),
        TURN_TOWARD_BEACON("Turn Toward Beacon"),
        DRIVE_TO_WALL("Drive to Wall"),
        SEARCH_FOR_WHITE_LINE("Search for White Line"),
        FOLLOW_LINE("Follow Line"),
        DETECT_COLOR("Detect Beacon Color"),
        TURN_BUTTON_ALIGNMENT("Turn to Push Button"),
        PUSH_BUTTON("Push Button"),
        MOVE_LEFT("Move Left"),
        STOP("Stop");

        String description;

        State(String description) {
            this.description = description;
        }

        public String toString() {
            return description;
        }
    }

    State currentState;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Hardware506(hardwareMap);
        robot.setDriveMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.setDriveMode(Hardware506.DriveMode.MECANUM_WITH_GYRO);
        currentState = State.DRIVE_AWAY_FROM_WALL;
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
                case DRIVE_AWAY_FROM_WALL:
                    forwardPower = .5;
                    sidePower = 0;
                    rotationPower = gyroAngleCorrection();
                    if (robot.leftFrontMotor.getCurrentPosition() > 1000){
                        currentState = State.TURN_TOWARD_BEACON;
                    }
                    break;
                case TURN_TOWARD_BEACON:
                    forwardPower = 0;
                    sidePower = 0;
                    rotationPower = turn(45);
                    if (rotationPower == 0){
                        robot.gyro.centerOffset();
                        currentState = State.DRIVE_TO_WALL;
                    }
                    break;
                case DRIVE_TO_WALL:
                    if (distance > 100) {
                        forwardPower = 1;
                    } else if (distance > 70) {
                        forwardPower = .5;
                    } else if (distance > 40) {
                        forwardPower = .3;
                    } else if (distance < 25) {
                        currentState = State.SEARCH_FOR_WHITE_LINE;
                        robot.setDriveMode(Hardware506.DriveMode.MECANUM);
                    }
                    sidePower = 0;
                    rotationPower = gyroAngleCorrection();
                    break;
                case SEARCH_FOR_WHITE_LINE:
                    if (distance > 25) {
                        forwardPower = .3;
                    } else if (distance > 20) {
                        forwardPower = .1;
                    } else if (distance < 15) {
                        forwardPower = -.1;
                    } else
                        forwardPower = 0;
                    if (robot.isLineDetected()) {
                        currentState = State.FOLLOW_LINE;
                    }
                    sidePower = -.2;
                    rotationPower = alignToWall();
                    break;
                case FOLLOW_LINE:
                    rotationPower = alignToWall();
                    if (distance > 10) {
                        forwardPower = .2;
                    }
                    else if (distance < 5)
                        forwardPower = -.1;
                    else if (distance > 7)
                        forwardPower = .1;
                    else
                        forwardPower = 0;

                    if (robot.isLineDetected()) {
                        sidePower = .1;
                    } else
                        sidePower = -.1;

                    if (rotationPower == 0 && forwardPower == 0) {
                        sidePower = 0;
                        currentState = State.DETECT_COLOR;
                    }
                    break;
                case DETECT_COLOR:
                    colorDetected = robot.getBeaconColor();
                    currentState = State.TURN_BUTTON_ALIGNMENT;
                    robot.gyro.centerOffset();
                    robot.setDriveMode(Hardware506.DriveMode.MECANUM_WITH_GYRO);
                    break;
                case TURN_BUTTON_ALIGNMENT:
                    forwardPower = 0;
                    sidePower = 0;
                    rotationPower = 0;
                    switch (colorDetected) {
                        case RED:
                            rotationPower = turn(20);
                            break;
                        case BLUE:
                            rotationPower = turn(-20);
                            break;
                    }
                    if (rotationPower == 0) {
                        currentState = State.PUSH_BUTTON;
                        time = getRuntime();
                    }
                    break;
                case PUSH_BUTTON:
                    if (getRuntime() > time + 1) {
                        forwardPower = 0;
                        robot.setDriveMode(Hardware506.DriveMode.MECANUM);
                        time = getRuntime();
                        beaconsPushed++;
                        if (beaconsPushed < 2)
                            currentState = State.MOVE_LEFT;
                        else
                            currentState = State.STOP;
                    } else {
                        forwardPower = .2;
                    }
                    break;
                case MOVE_LEFT:
                    if (getRuntime() < time + .75) {
                        if (distance > 25) {
                            forwardPower = .3;
                        } else if (distance > 20) {
                            forwardPower = .1;
                        } else if (distance < 15) {
                            forwardPower = -.1;
                        } else
                            forwardPower = 0;
                        sidePower = -1;
                        rotationPower = alignToWall();
                    } else {
                        currentState = State.SEARCH_FOR_WHITE_LINE;
                    }
                    break;
                case STOP:
                    sidePower = 0;
                    rotationPower = 0;
                    forwardPower = 0;
                    break;
            }


            robot.drive(forwardPower, sidePower, rotationPower);

            telemetry.addData("State", currentState);
            telemetry.addData("Ultrasonic Left", robot.leftUltrasonic.getUltrasonicLevel());
            telemetry.addData("Ultrasonic Right", robot.rightUltrasonic.getUltrasonicLevel());
            telemetry.addData("Ultrasonic Average", distance);
            telemetry.addData("Gyro Heading", robot.gyro.getHeading());
            telemetry.addData("Line Detected", robot.isLineDetected());
            telemetry.addData("Beacon Color Detected", robot.getBeaconColor());

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

    public double gyroAngleCorrection() {
        int heading = robot.gyro.getHeading();
        if (heading > 180) { // convert 0 - 360 range of heading to -180 - 180
            heading += 180;
            heading %= 360;
            heading -= 180;
        }
        heading = Range.clip(heading, -90, 90);
         return Range.scale(heading, -90, 90, -1, 1);
    }

    public double turn(double finalHeading){
        int heading = robot.gyro.getHeading();
        if (heading > 180) { // convert 0 - 360 range of heading to -180 - 180
            heading += 180;
            heading %= 360;
            heading -= 180;
        }

        if (heading > finalHeading)
            return  .3;
        else if (heading < finalHeading)
             return  -.3;
        else
        return 0;
    }
}
