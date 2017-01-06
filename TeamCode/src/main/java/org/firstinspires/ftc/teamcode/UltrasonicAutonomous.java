package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;

import static org.firstinspires.ftc.teamcode.Hardware506.ColorDetected.BLUE;
import static org.firstinspires.ftc.teamcode.Hardware506.ColorDetected.NONE;
import static org.firstinspires.ftc.teamcode.Hardware506.ColorDetected.RED;

/**
 * Autonomous without vuforia
 *
 * @author Oscar Kosar-Kosarewicz
 * @version 12/3/16
 */
@Autonomous(name = "Ultrasonic Autonomous", group = "Autonomous")
@Disabled
public abstract class UltrasonicAutonomous extends LinearOpMode {
    final static int DISTANCE_FROM_WALL_FAR = 35;
    final static int DISTANCE_FROM_WALL_CLOSE = 22;
    Hardware506 robot;
    double sidePower;
    double forwardPower;
    double rotationPower;
    double sweeperPower;
    double launcherPower;
    int directionMultiplier;
    double accelMagnitude;
    boolean lineFound;
    Hardware506.LineDetected lastDetectedPosition;
    Hardware506.LineDetected currentLinePosition;


    enum State {
        DRIVE_AWAY_FROM_WALL("Drive Away from Wall"),
        DRIVE_BEFORE_TURN("Drive Before Turn"),
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
        WAIT_FOR_SERVO("Wait For Servo"),
        MOVE_LEFT("Move Left"),
        CHECK_IF_STOPPED("Check if Stopped"),
        DEFLECT_CAP_BALL("Deflect Cap Ball"),
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
        int slideDirection = 1;
        Hardware506.ColorDetected colorDetected = NONE;
        double time = getRuntime();
        double incrementTime = getRuntime();
        double fiveSecondDelayTime = 0;
        //boolean reverseSidePower = false;
        lastDetectedPosition = Hardware506.LineDetected.NONE;
        lineFound = false;
        double waitTime = 0;
        double previousAccel;
        Hardware506.ColorDetected previousColor = NONE;
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
            checkIfUltrasonicsDead();
            double distance;
            if (!leftDead && !rightDead)
                distance = robot.getUltrasonicAverageDistance();
            else if (!rightDead)
                distance = robot.rightUltrasonic.getUltrasonicLevel();
            else
                distance = robot.leftUltrasonic.getUltrasonicLevel();

            distance = robot.getUltrasonicAverageDistance();

            Acceleration accel = robot.compass.getAcceleration();
            accelMagnitude = Math.sqrt(accel.xAccel * accel.xAccel + accel.yAccel * accel.yAccel + accel.zAccel * accel.zAccel);
            sidePower = 0;
            rotationPower = 0;
            forwardPower = 0;
            sweeperPower = 0;
            launcherPower = 0;
            switch (currentState) {
                case DRIVE_AWAY_FROM_WALL:
                    sidePower = -.7;
                    rotationPower = gyroAngleCorrection();
                    if (Math.abs(robot.leftFrontMotor.getCurrentPosition()) > 1250 / Hardware506.GEAR_RATIO) {
                        currentState = State.SHOOT_PARTICLE;
                        time = getRuntime();
                    }
                    break;
                case DRIVE_BEFORE_TURN:
                    sidePower = -.7;
                    rotationPower = gyroAngleCorrection();
                    if (Math.abs(robot.leftFrontMotor.getCurrentPosition()) > 2150 / Hardware506.GEAR_RATIO) {
                        robot.setDriveMode(Hardware506.DriveMode.MECANUM);
                        currentState = State.TURN_TOWARD_BEACON;
                    }
                    break;
                case TURN_TOWARD_VORTEX:
                    rotationPower = trueTurn(-25 * directionMultiplier);
                    if (getDesiredColor() == getDesiredColor()) {
                        rotationPower = -25;
                    }
                    if (rotationPower == 0) {
                        robot.gyro.centerOffset();
                        currentState = State.SHOOT_PARTICLE;
                        time = getRuntime();
                    }
                    break;
                case SHOOT_PARTICLE:
                    if (getRuntime() < time + 2) {
                        if (getRuntime() > time + 1.5 && getRuntime() < time + 3.5) {
                            //sweeperPower = 1;
                        }
                        launcherPower = 0;
                    } else {
                        currentState = State.DRIVE_BEFORE_TURN;
                    }
                    break;
                case TURN_TOWARD_BEACON:
                    int turnAngle = -14 * directionMultiplier;
                    if (getDesiredColor() == RED) {
                        turnAngle = -5;
                    }
                    rotationPower = trueTurn(turnAngle);

                    if (rotationPower == 0) {
                        robot.gyro.centerOffset();
                        currentState = State.DRIVE_TO_WALL;
                    }
                    break;
                case DRIVE_TO_WALL:
                    robot.setSlidePosition(Hardware506.SLIDE_SERVO_POSITION_RIGHT_LIMIT);
                    rotationPower = gyroAngleCorrection();
                    if (distance > 100) {
                        forwardPower = 1;
                    } else if (distance > 80) {
                        forwardPower = .5;
                    } else if (distance > 60) {
                        forwardPower = .3;
                    } else if (distance > 45) {
                        forwardPower = .15;
                    } else {
                        rotationPower = trueTurn(0);
                        forwardPower = 0;
                        if (true || isParallel()) {
                            currentState = State.SEARCH_FOR_WHITE_LINE;
                            //currentState = State.STOP;
                            robot.setSlidePosition(Hardware506.SLIDE_SERVO_POSITION_LEFT);
                            robot.setDriveMode(Hardware506.DriveMode.MECANUM);
                            time = getRuntime();
                        }
                    }
                    break;
                case SEARCH_FOR_WHITE_LINE:
                    robot.setSlidePosition(Hardware506.SLIDE_SERVO_POSITION_LEFT_LIMIT);
                    if (distance > 50) {
                        forwardPower = 1;
                        sidePower = 0;
                        rotationPower = 0;
                    } else if (distance < DISTANCE_FROM_WALL_CLOSE) {
                        forwardPower = -1;
                        sidePower = 0;
                    } else if (distance < DISTANCE_FROM_WALL_CLOSE + 7) {
                        forwardPower = -.5;
                        sidePower = 0;
                        rotationPower = 0;
                    } else if (distance > DISTANCE_FROM_WALL_FAR + 5) {
                        forwardPower = .15;
                    } else if (distance < DISTANCE_FROM_WALL_FAR) {
                        forwardPower = -.15;
                    } else
                        forwardPower = 0;
                    if (robot.getLineDetected() != Hardware506.LineDetected.NONE) {
                        lastDetectedPosition = robot.getLineDetected();
                        currentState = State.FOLLOW_LINE;
                        robot.setSlidePosition(Hardware506.SLIDE_SERVO_POSITION_LEFT);
                        lineFound = true;
                        time = getRuntime();
                    }
                    sidePower = -.3 * directionMultiplier;
                    if (hitCapBall()) {
                        currentState = State.CHECK_IF_STOPPED;
                        time = getRuntime();
                    }
                    if (getRuntime() > time + 14) {
                        currentState = State.MOVE_LEFT;
                        time = getRuntime();
                    } else if (getRuntime() > time + 9) {
                    } else if (getRuntime() > time + 5) {
                        sidePower = -sidePower;
                    }

                    if (lineFound)
                        rotationPower = alignToWall();
                    else {
                        rotationPower = trueTurn(0);
                    }
                    if (!isParallel() && distance > 25) {
                        sidePower = 0;
                        forwardPower = 0;
                        time = getRuntime();
                    }


                    break;
                case FOLLOW_LINE:
                    if (distance > DISTANCE_FROM_WALL_FAR) {
                        forwardPower = .5;
                    } else if (distance > DISTANCE_FROM_WALL_CLOSE + 1)
                        forwardPower = .15;
                    else if (distance < DISTANCE_FROM_WALL_CLOSE)
                        forwardPower = -.2;
                    else
                        forwardPower = 0;


                    alignToLine();


                    if (isParallel() && forwardPower == 0 && currentLinePosition == Hardware506.LineDetected.CENTER) {
                        currentState = State.DETECT_COLOR;
                        time = getRuntime();
                    }
                    robot.setSlidePosition(Hardware506.SLIDE_SERVO_POSITION_CENTER);
                    rotationPower = alignToWall();
                    if (robot.getLineDetected() != Hardware506.LineDetected.CENTER) {
                        forwardPower /= 2;
                    }
                    if (!isParallel()) {
                        sidePower = 0;
                        forwardPower = 0;
                    }
                    break;
                case DETECT_COLOR:
                    colorDetected = robot.getBeaconColor();
                    currentLinePosition = robot.getLineDetected();
                    if (currentLinePosition != Hardware506.LineDetected.CENTER) {
                        alignToLine();
                    }
                    else if (colorDetected == getDesiredColor())
                        currentState = State.PUSH_BUTTON;
                    else {
                        /**
                         if (Math.abs(robot.getSlideServoPosition() - robot.SLIDE_SERVO_POSITION_LEFT)
                         < Math.abs(robot.getSlideServoPosition() - robot.SLIDE_SERVO_POSITION_RIGHT))
                         robot.setSlidePosition(Hardware506.SLIDE_SERVO_POSITION_RIGHT);
                         else
                         robot.setSlidePosition(Hardware506.SLIDE_SERVO_POSITION_LEFT);
                         */
                        waitTime = 0;
                        robot.setSlidePosition(Hardware506.SLIDE_SERVO_POSITION_CENTER);

                        currentState = State.WAIT_FOR_SERVO;
                        nextState = State.PUSH_BUTTON;
                    }
                    break;
                case PUSH_BUTTON:
                    if (getRuntime() > time + 2 || distance < 8) {
                        currentState = State.BACK_UP;
                        time = getRuntime();
                        fiveSecondDelayTime = getRuntime();
                    } else if (robot.getLineDetected() == Hardware506.LineDetected.CENTER)
                        forwardPower = .3;


                    alignToLine();

                    rotationPower = alignToWall();
                    break;
                case BACK_UP:
                    if (distance > DISTANCE_FROM_WALL_CLOSE + 2)
                        forwardPower = .05;
                    else if (distance < DISTANCE_FROM_WALL_CLOSE)
                        forwardPower = -.2;
                    else {
                        currentState = State.VERIFY_WRONG_COLOR_WAS_NOT_PRESSED;
                        time = getRuntime();
                    }
                    if (currentLinePosition != Hardware506.LineDetected.CENTER) {
                        alignToLine();
                    }
                    rotationPower = alignToWall();
                    break;
                case VERIFY_WRONG_COLOR_WAS_NOT_PRESSED:
                    colorDetected = robot.getBeaconColor();
                    if (currentLinePosition != Hardware506.LineDetected.CENTER) {
                        alignToLine();
                        rotationPower = alignToWall();
                        time = getRuntime();
                    } else if (colorDetected != getDesiredColor() && colorDetected != NONE) {
                        waitTime = 5;
                        time = fiveSecondDelayTime;
                        currentState = State.WAIT;
                        nextState = State.PUSH_BUTTON;
                    } else if (colorDetected == getDesiredColor() || getRuntime() > time + 1) {
                        if (Math.abs(robot.slideServo.getPosition() - robot.SLIDE_SERVO_POSITION_LEFT)
                                < Math.abs(robot.slideServo.getPosition() - robot.SLIDE_SERVO_POSITION_RIGHT))
                            robot.setSlidePosition(Hardware506.SLIDE_SERVO_POSITION_RIGHT);
                        else
                            robot.setSlidePosition(Hardware506.SLIDE_SERVO_POSITION_LEFT);
                        currentState = State.WAIT;
                        waitTime = 2;
                        nextState = State.VERIFY_BEACON_WAS_PRESSED;

                        //skipping 2nd verify
                        currentState = State.MOVE_LEFT;
                        time = getRuntime();
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
                        time = getRuntime();
                    }
                    break;
                case WAIT_FOR_SERVO:
                    if ((robot.getBeaconColor() == getDesiredColor())) {
                        currentState = nextState;
                        time = getRuntime();
                    } else if ((robot.slideServo.getPosition() <= Hardware506.SLIDE_SERVO_POSITION_RIGHT_LIMIT || robot.slideServo.getPosition() >= Hardware506.SLIDE_SERVO_POSITION_LEFT_LIMIT || (colorDetected != getDesiredColor() && colorDetected != NONE)) && colorDetected != previousColor) {
                        slideDirection = -slideDirection;
                        time = getRuntime();
                        colorDetected = previousColor;
                    } else {
                        if (getRuntime() > incrementTime + .1) {
                            incrementTime = getRuntime();
                            robot.setSlidePosition(robot.slideServo.getPosition() - .0075 * slideDirection);
                        }
                    }
                    break;
                case MOVE_LEFT:
                    if (movedLeft == 1) {
                        currentState = State.STOP;
                    } else if (getRuntime() < time + 1) {
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
                case CHECK_IF_STOPPED:
                    if (getRuntime() < time + 1) {
                        final double accelThreshold = 9;
                        if (accelMagnitude > accelThreshold) {
                            time = getRuntime();
                            currentState = State.SEARCH_FOR_WHITE_LINE;
                        }
                    } else {
                        time = getRuntime();
                        currentState = State.DEFLECT_CAP_BALL;
                    }
                    break;
                case DEFLECT_CAP_BALL:
                    int turnTo = robot.gyro.getTrueHeading() - 170 * directionMultiplier;
                    if (getRuntime() > time + 2 &&
                            (((getDesiredColor() == BLUE && robot.gyro.getTrueHeading() < 180) ||
                                    (getDesiredColor() == RED && robot.gyro.getTrueHeading() > 180)) ||
                                    (robot.gyro.getTrueHeading() < 10 || robot.gyro.getTrueHeading() > 350))) {
                        turnTo = 0;
                    }
                    rotationPower = trueTurn(turnTo);
                    if (isParallel()) {
                        time = getRuntime();
                        currentState = State.SEARCH_FOR_WHITE_LINE;
                    }
                    break;
                case STOP:
                    robot.setSlidePosition(Hardware506.SLIDE_SERVO_POSITION_LEFT_LIMIT);
                    break;
            }

            sidePower /= Hardware506.GEAR_RATIO;
            forwardPower /= Hardware506.GEAR_RATIO;
            rotationPower /= Hardware506.GEAR_RATIO;

            robot.drive(-sidePower, forwardPower, rotationPower);
            robot.launcherMotor.setPower(launcherPower);
            robot.sweeperMotor.setPower(sweeperPower);

            telemetry.addData("State", currentState);
            telemetry.addData("Ultrasonic Left", robot.leftUltrasonic.getUltrasonicLevel());
            telemetry.addData("Ultrasonic Right", robot.rightUltrasonic.getUltrasonicLevel());
            telemetry.addData("Ultrasonic Average", distance);
            telemetry.addData("Gyro Heading", robot.gyro.getHeading());
            telemetry.addData("Gyro True Heading", robot.gyro.getTrueHeading());
            telemetry.addData("Line Detected", robot.getLineDetected());
            telemetry.addData("Beacon Color Detected", robot.getBeaconColor());
            telemetry.addData("Forward, Side, Turn", forwardPower + ", " + sidePower + ", " + rotationPower);
            telemetry.addData("lr, lf, rr, rf", robot.leftRearMotor.getPower() + ", " + robot.leftFrontMotor.getPower() + ", " + robot.rightRearMotor.getPower() + ", " + robot.rightFrontMotor.getPower());


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

        return trueTurn(0);/**
         if (!leftDead && !rightDead) {
         double left = robot.leftUltrasonic.getUltrasonicLevel();
         double right = robot.rightUltrasonic.getUltrasonicLevel() - 1;
         if (left + right / 2.0 < 80) {
         if (Math.abs(left - right) > 2) {
         if (left > right)
         return .2;
         else
         return -.2;
         }
         }
         return 0;
         } else {
         return trueTurn(0);
         }
         **/
    }

    public void alignToLine() {
        currentLinePosition = robot.getLineDetected();
        if (currentLinePosition != Hardware506.LineDetected.NONE) {
            lastDetectedPosition = currentLinePosition;
            time = getRuntime();
        }


        if (lastDetectedPosition == Hardware506.LineDetected.CENTER) {
            sidePower = 0;
        } else if (lastDetectedPosition == Hardware506.LineDetected.LEFT) {
            sidePower = -.15;
        } else if (lastDetectedPosition == Hardware506.LineDetected.RIGHT)
            sidePower = .15;

        if (currentLinePosition == Hardware506.LineDetected.NONE) {
            if (sidePower == 0) {
                sidePower = .15;
            }
            if (getRuntime() > time + 4.5) {
                currentState = State.MOVE_LEFT;
                time = getRuntime();
            } else if (getRuntime() > time + 1.5) {
                sidePower = -sidePower;
            }
        }
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

        double turnPower;
        if (Math.abs(heading - finalHeading) > 20) {
            turnPower = .5;
        } else if (Math.abs(heading - finalHeading) > 5) {
            turnPower = .3;
        } else {
            turnPower = .1;
        }
        if (Math.abs(heading - finalHeading) < 3) {
            return 0;
        }
        if (heading < finalHeading)
            return turnPower;
        else if (heading > finalHeading)
            return -turnPower;
        else
            return 0;
    }

    public double trueTurn(double finalHeading) {
        finalHeading %= 360;
        if (finalHeading > 180) {
            finalHeading -= 360;
        } else if (finalHeading < -180) {
            finalHeading += 360;
        }

        int heading = robot.gyro.getTrueHeading();
        if (getDesiredColor() == RED) {
            heading += 180;
            heading %= 360;
        }

        if (heading > 180) { // convert 0 - 360 range of heading to -180 - 180
            heading += 180;
            heading %= 360;
            heading -= 180;
        }

        double turnPower;
        if (Math.abs(heading - finalHeading) > 120) {
            turnPower = 1;
        }
        if (Math.abs(heading - finalHeading) > 90) {
            turnPower = .7;
        } else if (Math.abs(heading - finalHeading) > 40) {
            turnPower = .3;
        } else if (Math.abs(heading - finalHeading) > 20) {
            turnPower = .2;

        } else if (Math.abs(heading - finalHeading) > 10) {
            turnPower = .1;
        } else if (Math.abs(heading - finalHeading) > 5) {
            turnPower = .1;
        } else {
            turnPower = .1;
        }

        if (Math.abs(heading - finalHeading) < 2) {
            return 0;
        }

        if (heading <= 180 && heading >= 0 && finalHeading <= 180 && finalHeading >= 0) {
            if (finalHeading > heading) {
                return turnPower;
            } else {
                return -turnPower;
            }
        } else if (heading >= -180 && heading <= 0 && finalHeading >= -180 && finalHeading <= 0) {
            if (finalHeading > heading) {
                return turnPower;
            } else {
                return -turnPower;
            }
        } else if (heading >= -180 && heading <= 0 && finalHeading <= 180 && finalHeading >= 0) {
            if (finalHeading - heading > (heading + 360) - finalHeading) {
                return turnPower;
            } else {
                return -turnPower;
            }
        } else if (heading <= 180 && heading >= 0 && finalHeading >= -180 && finalHeading <= 0) {
            if (heading - finalHeading > (finalHeading + 360) - heading) {
                return turnPower;
            } else {
                return -turnPower;
            }
        }
        telemetry.addData("Error", finalHeading + ", " + heading);
        return .00005;

    }

    public boolean isParallel() {
        if (lineFound) {
            double left = robot.leftUltrasonic.getUltrasonicLevel();
            double right = robot.rightUltrasonic.getUltrasonicLevel();
            if (Math.abs(left - right) < 2) {
                {
                    return true;
                }
            }
        }
        int heading = robot.gyro.getTrueHeading();
        if (getDesiredColor() == RED) {
            heading += 180;
            heading %= 360;
        }
        if (heading > 180) { // convert 0 - 360 range of heading to -180 - 180
            heading += 180;
            heading %= 360;
            heading -= 180;
        }
        if (Math.abs(heading) > 3) {
            return false;
        } else {
            return true;
        }

    }

    public abstract Hardware506.ColorDetected getDesiredColor();

    private boolean leftDead = false;
    private boolean rightDead = false;
    private Double leftPreviousValue;
    private Double rightPreviousValue;
    private double leftTimer;
    private double rightTimer;

    void checkIfUltrasonicsDead() {
        if (leftPreviousValue == null) {
            leftPreviousValue = robot.leftUltrasonic.getUltrasonicLevel();
            leftTimer = getRuntime();
        }
        if (rightPreviousValue == null) {
            rightPreviousValue = robot.rightUltrasonic.getUltrasonicLevel();
            rightTimer = getRuntime();
        }

        if (robot.leftUltrasonic.getUltrasonicLevel() != leftPreviousValue) {
            leftTimer = getRuntime();
            leftPreviousValue = robot.leftUltrasonic.getUltrasonicLevel();
        }

        if (robot.rightUltrasonic.getUltrasonicLevel() != rightPreviousValue) {
            rightTimer = getRuntime();
            rightPreviousValue = robot.rightUltrasonic.getUltrasonicLevel();
        }

        if (leftTimer + 3 < getRuntime()) {
            leftDead = true;
        } else {
            leftDead = false;
        }

        if (rightTimer + 3 < getRuntime()) {
            rightDead = true;
        } else {
            rightDead = false;
        }
    }


    boolean hitCapBall() {
        final double ACCELERATION_THRESHOLD = 30;
        if (sidePower > Math.abs(.3) && accelMagnitude > ACCELERATION_THRESHOLD) {
            return true;
        }
        return false;
    }

}
