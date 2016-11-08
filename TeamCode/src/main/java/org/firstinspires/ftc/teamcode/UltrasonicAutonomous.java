package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import static org.firstinspires.ftc.teamcode.Hardware506.ColorDetected.BLUE;
import static org.firstinspires.ftc.teamcode.Hardware506.ColorDetected.NONE;
import static org.firstinspires.ftc.teamcode.Hardware506.ColorDetected.RED;

/**
 * Autonomous without vuforia, using two ultrasonic sensors and two color sensors
 *
 * @author Oscar Kosar-Kosarewicz
 * @version 11/2/16
 */
@Autonomous(name = "Ultrasonic Autonomous", group = "Autonomous")
@Disabled
public abstract class UltrasonicAutonomous extends LinearOpMode {
    final static int DISTANCE_FROM_WALL_FAR = 30;
    final static int DISTANCE_FROM_WALL_CLOSE = 22;
    Hardware506 robot;
    double sidePower;
    double forwardPower;
    double rotationPower;
    int directionMultiplier;


    enum State {
        DRIVE_AWAY_FROM_WALL("Drive Away from Wall"),
        TURN_TOWARD_BEACON("Turn Toward Beacon"),
        DRIVE_TO_WALL("Drive to Wall"),
        SEARCH_FOR_WHITE_LINE("Search for White Line"),
        FOLLOW_LINE("Follow Line"),
        DETECT_COLOR("Detect Beacon Color"),
        RETURN_TO_CENTER("Return to Center"),
        TURN_BUTTON_ALIGNMENT("Turn to Push Button"),
        PUSH_BUTTON("Push Button"),
        VERIFY_BEACON_COLOR("Verify Beacon Color"),
        WAIT_FIVE_SECONDS("Wait Five Seconds"),
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
    State nextState;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Hardware506(hardwareMap);
        robot.setDriveMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setDriveMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.setDriveMode(Hardware506.DriveMode.MECANUM_WITH_GYRO);
        if (getDesiredColor() == BLUE) {
            directionMultiplier = 1;
        } else if (getDesiredColor() == RED) {
            directionMultiplier = -1;
        }
        currentState = State.DRIVE_AWAY_FROM_WALL;
        int movedLeft = 0;
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
            sidePower = 0;
            rotationPower = 0;
            forwardPower = 0;
            switch (currentState) {
                case DRIVE_AWAY_FROM_WALL:
                    sidePower = -.3 * directionMultiplier;
                    forwardPower = 0;
                    rotationPower = gyroAngleCorrection();
                    if (Math.abs(robot.leftFrontMotor.getCurrentPosition()) > 750 / Hardware506.GEAR_RATIO) {
                        //currentState = State.TURN_TOWARD_BEACON;
                        robot.gyro.centerOffset();
                        currentState = State.DRIVE_TO_WALL;
                        robot.setArmPositionDown(true);
                    }
                    break;
                case TURN_TOWARD_BEACON:
                    forwardPower = 0;
                    sidePower = 0;
                    rotationPower = turn(-41 * directionMultiplier);
                    if (rotationPower == 0) {
                        robot.gyro.centerOffset();
                        currentState = State.DRIVE_TO_WALL;
                        robot.setArmPositionDown(true);
                    }
                    break;
                case DRIVE_TO_WALL:
                    if (distance > 70) {
                        forwardPower = .25;
                    } else if (distance > 50) {
                        forwardPower = .15;
                    } else if (distance < 40) {
                        currentState = State.SEARCH_FOR_WHITE_LINE;
                        robot.setDriveMode(Hardware506.DriveMode.MECANUM);
                    }
                    sidePower = 0;
                    rotationPower = gyroAngleCorrection();
                    break;
                case SEARCH_FOR_WHITE_LINE:
                    if (distance > DISTANCE_FROM_WALL_FAR + 5) {
                        forwardPower = .075;
                    } else if (distance < DISTANCE_FROM_WALL_FAR) {
                        forwardPower = -.075;
                    } else
                        forwardPower = 0;
                    if (robot.isLineDetected()) {
                        currentState = State.FOLLOW_LINE;
                    }
                    sidePower = -.05 * directionMultiplier;
                    rotationPower = alignToWall();
                    break;
                case FOLLOW_LINE:
                    rotationPower = alignToWall();
                    if (distance > DISTANCE_FROM_WALL_FAR) {
                        forwardPower = .1;
                    } else if (distance < DISTANCE_FROM_WALL_CLOSE)
                        forwardPower = -.05;
                    else if (distance > DISTANCE_FROM_WALL_CLOSE + 4)
                        forwardPower = .05;
                    else
                        forwardPower = 0;

                    if (robot.isLineDetected()) {
                        sidePower = -.025 * directionMultiplier;
                    } else
                        sidePower = .025 * directionMultiplier;

                    if (rotationPower == 0 && forwardPower == 0) {
                        sidePower = 0;
                        currentState = State.DETECT_COLOR;
                        robot.setDriveMode(Hardware506.DriveMode.MECANUM_WITH_GYRO);
                        robot.gyro.centerOffset();
                        time = getRuntime();
                        nextState = State.TURN_BUTTON_ALIGNMENT;
                    }
                    break;
                case DETECT_COLOR:
                    /**
                     * robot.setDriveMode(Hardware506.DriveMode.MECANUM);
                     * colorDetected = robot.getBeaconColor();
                    if ((getRuntime() < time + 2) && (colorDetected == NONE)) {
                        if (distance > 21) {
                            forwardPower = .025;
                        } else if (distance < 19) {
                            forwardPower = -.025;
                        } else
                            forwardPower = 0;
                        sidePower = .015 * directionMultiplier;
                        rotationPower = alignToWall();
                    } else if ((getRuntime() > time + 1)) {
                        time = getRuntime();
                        currentState = State.RETURN_TO_CENTER;
                    }
                     **/
                    rotationPower = turn(7 * directionMultiplier);
                    if (rotationPower == 0){
                        colorDetected = robot.getBeaconColor();
                        if (colorDetected != NONE)
                            currentState = State.TURN_BUTTON_ALIGNMENT;
                        else {
                            currentState = State.FOLLOW_LINE;
                            robot.setDriveMode(Hardware506.DriveMode.MECANUM);
                        }
                    }


                    break;
                case RETURN_TO_CENTER:
                    if (!robot.isLineDetected() && getRuntime() < time + 2) {
                        if (distance > 26) {
                            forwardPower = .025;
                        } else if (distance < 24) {
                            forwardPower = -.025;
                        } else
                            forwardPower = 0;
                        sidePower = -.1 * directionMultiplier;
                        rotationPower = alignToWall();
                    } else {
                        currentState = nextState;
                        if (currentState == State.PUSH_BUTTON) {
                            robot.gyro.centerOffset();
                            robot.setDriveMode(Hardware506.DriveMode.MECANUM_WITH_GYRO);
                        }
                    }
                    break;
                case TURN_BUTTON_ALIGNMENT:
                    switch (colorDetected) {
                        case RED:
                            rotationPower = turn(-7 * directionMultiplier);
                            break;
                        case BLUE:
                            rotationPower = turn(7 * directionMultiplier);
                            break;
                        case NONE:
                            rotationPower = turn(7 * directionMultiplier);
                            break;
                    }
                    if (rotationPower == 0) {
                        currentState = State.PUSH_BUTTON;
                        time = getRuntime();
                    }
                    break;
                case PUSH_BUTTON:
                    if (getRuntime() > time + 1.5 || distance < 15) {
                        forwardPower = 0;
                        //robot.setDriveMode(Hardware506.DriveMode.MECANUM);

                        currentState = State.VERIFY_BEACON_COLOR;
                        time = getRuntime();
                        nextState = State.VERIFY_BEACON_COLOR;

                    } else if (getRuntime() < time + .5) {
                        forwardPower = -.1;
                    } else forwardPower = .15;
                    break;
                case VERIFY_BEACON_COLOR:
                    if (colorDetected == getDesiredColor() || colorDetected == NONE) {
                        if (movedLeft == 1) {
                            currentState = State.STOP;
                        } else
                            currentState = State.MOVE_LEFT;
                    } else {
                        currentState = State.WAIT_FIVE_SECONDS;
                        time = getRuntime();
                    }
                    break;
                case WAIT_FIVE_SECONDS:
                    if (getRuntime() > time + 5) {
                        currentState = State.PUSH_BUTTON;
                    }
                    break;
                case MOVE_LEFT:
                    if (getRuntime() < time + .75) {
                        if (distance > 30) {
                            forwardPower = .15;
                        } else if (distance > DISTANCE_FROM_WALL_FAR + 5) {
                            forwardPower = .05;
                        } else if (distance < DISTANCE_FROM_WALL_FAR) {
                            forwardPower = -.05;
                        } else
                            forwardPower = 0;
                        sidePower = -1 * directionMultiplier;
                        rotationPower = alignToWall();
                    } else {
                        currentState = State.SEARCH_FOR_WHITE_LINE;
                        movedLeft++;
                    }
                    break;
                case STOP:
                    sidePower = 0;
                    rotationPower = 0;
                    forwardPower = 0;
                    robot.setArmPositionDown(false);
                    break;
            }

            sidePower *= Hardware506.GEAR_RATIO;
            forwardPower *= Hardware506.GEAR_RATIO;
            rotationPower *= Hardware506.GEAR_RATIO;

            robot.drive(-sidePower, forwardPower, rotationPower);

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
        if (left + right / 2.0 < 40) {
            if (Math.abs(left - right) > 2) {
                if (left > right)
                    return .2;
                else
                    return -.2;
            }
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

    public double turn(double finalHeading) {
        int heading = robot.gyro.getHeading();

        if (heading > 180) { // convert 0 - 360 range of heading to -180 - 180
            heading += 180;
            heading %= 360;
            heading -= 180;
        }

        if (Math.abs(heading - finalHeading) < 3) {
            return 0;
        }
        if (heading < finalHeading)
            return .05;
        else if (heading > finalHeading)
            return -.05;
        else
            return 0;
    }

    public abstract Hardware506.ColorDetected getDesiredColor();
}
