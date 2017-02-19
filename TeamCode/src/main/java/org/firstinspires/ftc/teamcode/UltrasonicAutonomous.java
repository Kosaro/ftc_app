package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

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
    final static int DISTANCE_FROM_WALL_FAR = 26;
    final static int DISTANCE_FROM_WALL_CLOSE = 23;
    Hardware506 robot;
    double sidePower;
    double forwardPower;
    double rotationPower;
    double sweeperPower;
    double launcherPower;
    int encoderOffset = 0;
    int directionMultiplier;
    double accelMagnitude;
    boolean lineFound;
    Hardware506.LineDetected lastDetectedPosition;
    Hardware506.LineDetected currentLinePosition;


    enum State {
        DRIVE_AWAY_FROM_WALL("Drive Away from Wall"),
        DRIVE_BEFORE_TURN("Drive Before Turn"),
        SHOOT_PARTICLE("Shoot Particle"),
        TURN_TOWARD_BEACON("Turn Toward Beacon"),
        DRIVE_TO_WALL("Drive to Wall"),
        TURN_TOWARD_WALL("Turn toward Wall"),
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
        robot.launcherMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        int launcherOffset = 0;
        //boolean reverseSidePower = false;
        lastDetectedPosition = Hardware506.LineDetected.NONE;
        lineFound = false;
        int particlesShot = 0;
        double waitTime = 0;
        Hardware506.ColorDetected previousColor = NONE;
        boolean crossedLine = false;
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

            //distance = robot.getUltrasonicAverageDistance();

            Acceleration accel = robot.compass.getAcceleration();
            accelMagnitude = Math.sqrt(accel.xAccel * accel.xAccel + accel.yAccel * accel.yAccel + accel.zAccel * accel.zAccel);
            sidePower = 0;
            rotationPower = 0;
            forwardPower = 0;
            sweeperPower = 0;
            launcherPower = 0;
            switch (currentState) {
                case DRIVE_AWAY_FROM_WALL:
                    sidePower = -1;
                    rotationPower = gyroAngleCorrection();
                    if (Math.abs(robot.leftFrontMotor.getCurrentPosition()) > 500 / Hardware506.GEAR_RATIO) {
                        currentState = State.SHOOT_PARTICLE;
                        time = getRuntime();
                    }
                    break;
                case DRIVE_BEFORE_TURN:
                    sidePower = -1;
                    rotationPower = gyroAngleCorrection();
                    if (Math.abs(robot.leftFrontMotor.getCurrentPosition()) > 500 / Hardware506.GEAR_RATIO) {
                        robot.setDriveMode(Hardware506.DriveMode.MECANUM);
                        currentState = State.TURN_TOWARD_BEACON;
                    }
                    break;

                case SHOOT_PARTICLE:
                    if (Math.abs(robot.launcherMotor.getCurrentPosition() - launcherOffset) < 1120 * 4.0 / 3) {
                        launcherPower = 1;
                    } else {
                        if (particlesShot < 1) {
                            nextState = State.SHOOT_PARTICLE;
                            currentState = State.WAIT;
                            waitTime = 1.5;
                            time = getRuntime();
                            launcherOffset = robot.launcherMotor.getCurrentPosition();
                            particlesShot++;
                        } else {
                            currentState = State.DRIVE_BEFORE_TURN;
                        }
                    }
                    break;
                case TURN_TOWARD_BEACON:
                    int turnAngle = 54 * directionMultiplier;
                    if (getDesiredColor() == RED) {
                        turnAngle = -52  ;
                    }
                    rotationPower = trueTurn(turnAngle);

                    if (rotationPower == 0) {
                        robot.gyro.centerOffset();
                        currentState = State.DRIVE_TO_WALL;
                        resetEncoder();
                        robot.gyro.centerOffset();
                    }
                    break;
                case DRIVE_TO_WALL:
                    /**
                     robot.setSlidePosition(Hardware506.SLIDE_SERVO_POSITION_RIGHT_LIMIT);
                     rotationPower = preciseTurn(0) * 3;
                     if (distance > 100) {
                     forwardPower = 1;
                     } else if (distance > 80) {
                     forwardPower = .7;
                     } else if (distance > 60) {
                     forwardPower = .4;
                     } else if (distance > 45) {
                     forwardPower = .2;
                     } else if (distance < 25) {
                     forwardPower = -.3;
                     } else {
                     rotationPower = preciseTurn(0) * 1.5;
                     forwardPower = 0;
                     if (isParallel()) {
                     currentState = State.SEARCH_FOR_WHITE_LINE;
                     //currentState = State.STOP;
                     robot.setSlidePosition(Hardware506.SLIDE_SERVO_POSITION_LEFT);
                     robot.setDriveMode(Hardware506.DriveMode.MECANUM);
                     time = getRuntime();
                     }
                     }
                     if (!isParallel()) {
                     forwardPower /= 1.5;
                     }
                     **/
                    robot.setSlidePosition(Hardware506.SLIDE_SERVO_POSITION_RIGHT);

                    rotationPower = gyroAngleCorrection();
                    forwardPower = 0;
                    sidePower = -1 * directionMultiplier;
                    if (Math.abs(getEncoderPosition()) > 2700) {
                        currentState = State.TURN_TOWARD_WALL;
                        robot.setSlidePosition(Hardware506.SLIDE_SERVO_POSITION_LEFT);
                        robot.setDriveMode(Hardware506.DriveMode.MECANUM);
                        time = getRuntime();
                    }
                    break;
                case TURN_TOWARD_WALL:

                    rotationPower = trueTurn(0);

                    if (isParallel()) {
                        //robot.gyro.centerOffset();
                        currentState = State.SEARCH_FOR_WHITE_LINE;
                        resetEncoder();
                        robot.gyro.centerOffset();
                    }
                    break;
                case SEARCH_FOR_WHITE_LINE:
                    robot.setSlidePosition(Hardware506.SLIDE_SERVO_POSITION_LEFT_LIMIT);
                    if (distance > 50) {
                        forwardPower = 1;
                        sidePower = 0;
                        rotationPower = 0;
                    } else if (distance < DISTANCE_FROM_WALL_CLOSE) {
                        forwardPower = -.5;
                        sidePower = 0;
                    } else if (distance < DISTANCE_FROM_WALL_CLOSE + 2) {
                        forwardPower = -.3;
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
                        resetEncoder();
                        time = getRuntime();
                    } else if (getRuntime() > time + 9) {
                    } else if (getRuntime() > time + 5) {
                        sidePower = -sidePower;
                    }

                    if (lineFound)
                        rotationPower = alignToWall();
                    else {
                        rotationPower = preciseTurn(0);
                    }
                    if (!isParallel()) {
                        if (distance < DISTANCE_FROM_WALL_CLOSE) {
                            sidePower = 0;
                            //forwardPower = 0;
                            rotationPower *= 2;
                            time = getRuntime();
                        } else if (distance < 50 && getRobotOffset() > 10) {
                            sidePower = 0;
                            forwardPower = 0;
                            rotationPower *= 2;

                        } else {
                            //sidePower = 0;
                            //forwardPower = 0;
                            rotationPower *= 2;
                            time = getRuntime();
                        }
                    }
                    break;
                case FOLLOW_LINE:
                    if (distance > DISTANCE_FROM_WALL_FAR + 8) {
                        forwardPower = .5;
                    }
                    if (distance > DISTANCE_FROM_WALL_FAR) {
                        forwardPower = .4;
                    } else if (distance > DISTANCE_FROM_WALL_CLOSE + 3)
                        forwardPower = .2;
                    else if (distance > DISTANCE_FROM_WALL_CLOSE + 1)
                        forwardPower = .125 ;
                    else if (distance < DISTANCE_FROM_WALL_CLOSE)
                        forwardPower = -.2;
                    else
                        forwardPower = 0;


                    alignToLine();

                    rotationPower = alignToWall();
                    if (rotationPower == 0 && forwardPower == 0 && currentLinePosition == Hardware506.LineDetected.CENTER) {
                        currentState = State.DETECT_COLOR;
                        time = getRuntime();
                    }
                    robot.setSlidePosition(Hardware506.SLIDE_SERVO_POSITION_CENTER);

                    if (robot.getLineDetected() != Hardware506.LineDetected.CENTER) {
                        //fforwardPower /= 2;
                    }
                    if (!isParallel()) {
                        if (rotationPower > .1){
                            sidePower = 0;
                            forwardPower = 0;
                        }
                        if (getRobotOffset() > 12){
                            sidePower = 0;
                            forwardPower = -.2;
                            rotationPower *= 1.5;
                        }

                        rotationPower *= 1.5;
                    }
                    if (distance < DISTANCE_FROM_WALL_CLOSE - 2) {
                        forwardPower = -.3;
                        rotationPower = 0;
                        sidePower = 0;
                    }

                    break;
                case DETECT_COLOR:
                    colorDetected = robot.getBeaconColor();
                    currentLinePosition = robot.getLineDetected();
                    if (currentLinePosition != Hardware506.LineDetected.CENTER) {
                        alignToLine();
                    } else if (colorDetected == getDesiredColor())
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
                        resetEncoder();
                        time = getRuntime();
                    }
                    break;
                case VERIFY_BEACON_WAS_PRESSED:
                    colorDetected = robot.getBeaconColor();
                    if (colorDetected == getDesiredColor() || colorDetected == NONE) {
                        currentState = State.MOVE_LEFT;
                        resetEncoder();
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
                    if ((robot.slideServo.getPosition() <=
                            Hardware506.SLIDE_SERVO_POSITION_RIGHT -  .1 &&
                            robot.slideServo.getPosition() >= Hardware506.SLIDE_SERVO_POSITION_LEFT_LIMIT)){
                        crossedLine = false;
                    }
                    telemetry.addData("Crossed line", crossedLine);
                    telemetry.addData("Servo Position",robot.slideServo.getPosition());
                    telemetry.addData("slide direction", slideDirection);

                    if ((robot.getBeaconColor() == getDesiredColor())) {
                        currentState = nextState;
                        time = getRuntime();
                    } else if ((robot.slideServo.getPosition() <=
                            Hardware506.SLIDE_SERVO_POSITION_RIGHT -  .03 ||
                            robot.slideServo.getPosition() >= Hardware506.SLIDE_SERVO_POSITION_LEFT_LIMIT
                            || (colorDetected != getDesiredColor() && colorDetected != NONE)) && (colorDetected
                            != previousColor || previousColor == NONE) && !crossedLine && getRuntime() > time + .5) {
                        slideDirection = -slideDirection;
                        time = getRuntime();
                        if (robot.slideServo.getPosition() <=
                                Hardware506.SLIDE_SERVO_POSITION_RIGHT -  .1 ||
                                robot.slideServo.getPosition() >= Hardware506.SLIDE_SERVO_POSITION_LEFT_LIMIT){
                                crossedLine = true;
                        }


                        colorDetected = previousColor;
                    } else {
                        telemetry.addData("servo", "active");
                        if (getRuntime() > incrementTime + .1) {
                            incrementTime = getRuntime();
                            double incrementValue = .008 ;
                            if (time  + 1 > getRuntime()){
                                incrementValue /= 2;
                            }else if (time  + .5 > getRuntime()){
                                incrementValue /= 3;
                            }
                            robot.setSlidePosition(robot.slideServo.getPosition() + incrementValue  * slideDirection);
                            telemetry.addData("servo", robot.slideServo.getPosition() + ", " + previousColor);
                        } else {
                            telemetry.addData("servo", "not active");
                        }
                    }

                    break;
                case MOVE_LEFT:
                    if (movedLeft == 1) {
                        currentState = State.STOP;
                    } else if (Math.abs(getEncoderPosition()) < 3200) {
                        robot.setSlidePosition(Hardware506.SLIDE_SERVO_POSITION_LEFT);
                        if (distance > DISTANCE_FROM_WALL_FAR + 5) {
                            forwardPower = .1;
                        } else if (distance < DISTANCE_FROM_WALL_CLOSE) {
                            forwardPower = -.5;
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
                    rotationPower = preciseTurn(turnTo);
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
            telemetry.addData("launcher", robot.launcherMotor.getCurrentPosition());
            telemetry.addData("Ultrasonic Left", robot.leftUltrasonic.getUltrasonicLevel());
            telemetry.addData("Ultrasonic Right", robot.rightUltrasonic.getUltrasonicLevel());
            telemetry.addData("Ultrasonic Average", distance);
            telemetry.addData("Gyro Heading", robot.gyro.getHeading());
            telemetry.addData("Gyro True Heading", robot.gyro.getTrueHeading());
            telemetry.addData("Line Detected", robot.getLineDetected());
            telemetry.addData("Beacon Color Detected", robot.getBeaconColor());
            telemetry.addData("Color Detected RGB", String.format("%4d, %4d, %4d", robot.beaconColorSensor.red(), robot.beaconColorSensor.green(), robot.beaconColorSensor.blue()));
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

        //return preciseTurn(0);
        double left = robot.leftUltrasonic.getUltrasonicLevel();
        double right = robot.rightUltrasonic.getUltrasonicLevel();
        if (!leftDead && !rightDead && left + right / 2.0 < 80) {
            if (Math.abs(left - right) > 2) {
                if (left > right)
                    return -.2;
                else
                    return .2;
            }
            return 0;
        } else {
            return preciseTurn(0);
        }

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
            sidePower = -.2;
        } else if (lastDetectedPosition == Hardware506.LineDetected.RIGHT)
            sidePower = .2;

        if (currentLinePosition == Hardware506.LineDetected.NONE) {
            if (sidePower == 0) {
                sidePower = .3;
            }
            if (getRuntime() > time + 4.5) {
                currentState = State.MOVE_LEFT;
                resetEncoder();
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
        double turnPower;
//        if (Math.abs(heading) > 90) {
//            turnPower = .7;
//        } else if (Math.abs(heading) > 40) {
//            turnPower = .3;}
        if (Math.abs(heading) > 20) {
            turnPower = .5;
        } else if (Math.abs(heading) > 10) {
            turnPower = .4;
        } else if (Math.abs(heading) > 5) {
            turnPower = .3;
        } else if (Math.abs(heading) > 2) {
            turnPower = .1;
        } else {
            turnPower = .1;
        }

        if (heading > 0) {
            turnPower = -turnPower;
        }
        return turnPower;
//        heading = Range.clip(heading, -60, 60);
//        return -Range.scale(heading, -60, 60, -1, 1);

        //return turn(0) * 2;
    }

    public double turn(double finalHeading) {
        finalHeading %= 360;
        if (finalHeading > 180) {
            finalHeading -= 360;
        } else if (finalHeading < -180) {
            finalHeading += 360;
        }

        int heading = robot.gyro.getHeading();


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
            if (finalHeading - heading < (heading + 360) - finalHeading) {
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

    public double preciseTurn(double finalHeading) {
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
            if (finalHeading - heading < (heading + 360) - finalHeading) {
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
        } else if (Math.abs(heading - finalHeading) > 90) {
            turnPower = .7;
        } else if (Math.abs(heading - finalHeading) > 40) {
            turnPower = .4;
        } else if (Math.abs(heading - finalHeading) > 30) {
            turnPower = .35;
        } else if (Math.abs(heading - finalHeading) > 25) {
            turnPower = .3;
        } else if (Math.abs(heading - finalHeading) > 20) {
            turnPower = .2;

        } else if (Math.abs(heading - finalHeading) > 15) {
            turnPower = .2;

        } else if (Math.abs(heading - finalHeading) > 10) {
            turnPower = .15;
        } else if (Math.abs(heading - finalHeading) > 5) {
            turnPower = .15;
        } else {
            turnPower = .10;
        }

        if (Math.abs(heading - finalHeading) < 3
                ) {
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
            if (finalHeading - heading < (heading + 360) - finalHeading) {
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

        if (getRobotOffset() > 3) {
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

    public int getEncoderPosition() {
        return robot.leftFrontMotor.getCurrentPosition() - encoderOffset;
    }

    public void resetEncoder() {
        encoderOffset = robot.leftFrontMotor.getCurrentPosition();
    }

    int getRobotOffset() {
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
        return Math.abs(heading);

    }
}
