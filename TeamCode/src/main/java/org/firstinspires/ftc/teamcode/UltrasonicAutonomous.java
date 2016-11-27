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
 * Autonomous without vuforia
 *
 * @author Oscar Kosar-Kosarewicz
 * @version 11/27/16
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
    double sweeperPower;
    double launcherPower;
    int directionMultiplier;


    enum State {
        DRIVE_AWAY_FROM_WALL("Drive Away from Wall"),
        TURN_TOWARD_VORTEX("Turn Toward Center Vortex"),
        SHOOT_PARTICLE("Shoot Particle"),
        TURN_TOWARD_BEACON("Turn Toward Beacon"),
        DRIVE_TO_WALL("Drive to Wall"),
        SEARCH_FOR_WHITE_LINE("Search for White Line"),
        FOLLOW_LINE("Follow Line"),
        DETECT_COLOR("Detect Beacon Color"),
        PUSH_BUTTON("Push Button"),
        BACK_UP("Back Up"),
        VERIFY_WRONG_COLOR_WAS_NOT_PRESSED("Verify Wrong Color Was Not Pressed"),
        VERIFY_BEACON_WAS_PRESSED("Verify Beacon Was Pressed"),
        WAIT("Wait"),
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
        double fiveSecondDelayTime = 0;
        double waitTime = 0;
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
            sweeperPower = 0;
            launcherPower = 0;
            switch (currentState) {
                case DRIVE_AWAY_FROM_WALL:
                    sidePower = -.5;
                    rotationPower = gyroAngleCorrection();
                    if (Math.abs(robot.leftFrontMotor.getCurrentPosition()) > 1400 / Hardware506.GEAR_RATIO) {
                        currentState = State.TURN_TOWARD_VORTEX;
                    }
                    break;
                case TURN_TOWARD_VORTEX:
                    rotationPower = turn(-20 * directionMultiplier);
                    if (rotationPower == 0) {
                        robot.gyro.centerOffset();
                        currentState = State.SHOOT_PARTICLE;
                        time = getRuntime();
                    }
                    break;
                case SHOOT_PARTICLE:
                    if (getRuntime() < time + 5) {
                        if (getRuntime() > time + 1.5 && getRuntime() < time + 3.5) {
                            sweeperPower = 1;
                        }
                        launcherPower = 1;
                    } else {
                        currentState = State.TURN_TOWARD_BEACON;
                    }
                    break;
                case TURN_TOWARD_BEACON:
                    int turnAngle = 10 * directionMultiplier;
                    if (getDesiredColor() == RED) {
                        turnAngle += 180;
                    }
                    rotationPower = turn(turnAngle);

                    if (rotationPower == 0) {
                        robot.gyro.centerOffset();
                        currentState = State.DRIVE_TO_WALL;
                        robot.setSlidePosition(Hardware506.SLIDE_SERVO_POSITION_LEFT);
                    }
                    break;
                case DRIVE_TO_WALL:
                    if (distance > 70) {
                        forwardPower = .5;
                    } else if (distance > 50) {
                        forwardPower = .3;
                    } else if (distance < 40) {
                        currentState = State.SEARCH_FOR_WHITE_LINE;
                        robot.setDriveMode(Hardware506.DriveMode.MECANUM);
                        time = getRuntime();
                    }
                    rotationPower = gyroAngleCorrection();
                    break;
                case SEARCH_FOR_WHITE_LINE:
                    if (distance > DISTANCE_FROM_WALL_FAR + 5) {
                        forwardPower = .2;
                    } else if (distance < DISTANCE_FROM_WALL_FAR) {
                        forwardPower = -.2;
                    } else
                        forwardPower = 0;
                    if (robot.isLineDetected()) {
                        currentState = State.FOLLOW_LINE;
                        time = getRuntime();
                    }
                    sidePower = -.15 * directionMultiplier;
                    if (getRuntime() > time + 8) {
                        currentState = State.MOVE_LEFT;
                        time = getRuntime();
                    } else if (getRuntime() > time + 5) {
                    } else if (getRuntime() > time + 3) {
                        sidePower = -sidePower;
                    }
                    rotationPower = alignToWall();
                    break;
                case FOLLOW_LINE:
                    if (distance > DISTANCE_FROM_WALL_FAR) {
                        forwardPower = .3;
                    } else if (distance > DISTANCE_FROM_WALL_CLOSE)
                        forwardPower = .05;
                    else if (distance < DISTANCE_FROM_WALL_CLOSE + 3)
                        forwardPower = -.05;
                    else
                        forwardPower = 0;

                    if (robot.isLineDetected()) {
                        sidePower = -.025 * directionMultiplier;
                        time = getRuntime();
                    } else
                        sidePower = .025 * directionMultiplier;
                    if (getRuntime() > time + 4.5) {
                        currentState = State.MOVE_LEFT;
                        time = getRuntime();
                    } else if (getRuntime() > time + 2.5) {
                    } else if (getRuntime() > time + 1) {
                        sidePower = -sidePower;
                    }
                    if (rotationPower == 0 && forwardPower == 0 && robot.isLineDetected()) {
                        currentState = State.DETECT_COLOR;
                        time = getRuntime();
                    }
                    robot.setSlidePosition(Hardware506.SLIDE_SERVO_POSITION_LEFT);
                    rotationPower = alignToWall();
                    break;
                case DETECT_COLOR:
                    colorDetected = robot.getBeaconColor();
                    if (colorDetected != NONE)
                        if (colorDetected == getDesiredColor())
                            currentState = State.PUSH_BUTTON;
                        else {
                            if (Math.abs(robot.getSlideServoPosition() - robot.SLIDE_SERVO_POSITION_LEFT)
                                    < Math.abs(robot.getSlideServoPosition() - robot.SLIDE_SERVO_POSITION_RIGHT))
                                robot.setSlidePosition(Hardware506.SLIDE_SERVO_POSITION_RIGHT);
                            else
                                robot.setSlidePosition(Hardware506.SLIDE_SERVO_POSITION_LEFT);
                            currentState = State.WAIT;
                            waitTime = 1;
                            nextState = State.PUSH_BUTTON;
                        }
                    else if (getRuntime() > time + .1) {
                        time = getRuntime();
                        robot.setSlidePosition(robot.getSlideServoPosition() + .01);
                    }
                    break;
                case PUSH_BUTTON:
                    if (getRuntime() > time + 1.5 || distance < 8) {
                        currentState = State.BACK_UP;
                        time = getRuntime();
                        fiveSecondDelayTime = getRuntime();
                    } else forwardPower = .15;

                    if (robot.isLineDetected()) {
                        sidePower = -.025 * directionMultiplier;
                        time = getRuntime();
                    } else
                        sidePower = .025 * directionMultiplier;
                    rotationPower = alignToWall();
                    break;
                case BACK_UP:
                    if (distance > DISTANCE_FROM_WALL_CLOSE)
                        forwardPower = .05;
                    else if (distance < DISTANCE_FROM_WALL_CLOSE + 3)
                        forwardPower = -.2;
                    else {
                        currentState = State.VERIFY_WRONG_COLOR_WAS_NOT_PRESSED;
                        time = getRuntime();
                    }
                    break;
                case VERIFY_WRONG_COLOR_WAS_NOT_PRESSED:
                    colorDetected = robot.getBeaconColor();
                    if (colorDetected != getDesiredColor() && colorDetected != NONE) {
                        waitTime = 5;
                        time = fiveSecondDelayTime;
                        currentState = State.WAIT;
                        nextState = State.PUSH_BUTTON;
                    } else if (colorDetected == getDesiredColor() || getRuntime() > time + 1) {
                        if (Math.abs(robot.getSlideServoPosition() - robot.SLIDE_SERVO_POSITION_LEFT)
                                < Math.abs(robot.getSlideServoPosition() - robot.SLIDE_SERVO_POSITION_RIGHT))
                            robot.setSlidePosition(Hardware506.SLIDE_SERVO_POSITION_RIGHT);
                        else
                            robot.setSlidePosition(Hardware506.SLIDE_SERVO_POSITION_LEFT);
                        currentState = State.WAIT;
                        waitTime = 1;
                        nextState = State.VERIFY_BEACON_WAS_PRESSED;
                    }
                    break;
                case VERIFY_BEACON_WAS_PRESSED:
                    colorDetected = robot.getBeaconColor();
                    if (colorDetected == getDesiredColor() || colorDetected == NONE) {
                        currentState = State.MOVE_LEFT;
                        time = getRuntime();
                    } else {
                        currentState = State.PUSH_BUTTON;
                        time = getRuntime();
                    }
                    break;
                case WAIT:
                    if (getRuntime() > time + waitTime) {
                        currentState = nextState;
                    }
                    break;
                case MOVE_LEFT:
                    if (movedLeft == 1) {
                        currentState = State.STOP;
                    } else if (getRuntime() < time + .75) {
                        robot.setSlidePosition(Hardware506.SLIDE_SERVO_POSITION_LEFT);
                        if (distance > DISTANCE_FROM_WALL_FAR + 5) {
                            forwardPower = .1;
                        } else if (distance < DISTANCE_FROM_WALL_FAR) {
                            forwardPower = -.1;
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
                    break;
            }

            sidePower /= Hardware506.GEAR_RATIO;
            forwardPower /= Hardware506.GEAR_RATIO;
            rotationPower /= Hardware506.GEAR_RATIO;

            robot.drive(sidePower, -forwardPower, rotationPower);
            robot.launcherMotor.setPower(launcherPower);
            robot.sweeperMotor.setPower(sweeperPower);

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
