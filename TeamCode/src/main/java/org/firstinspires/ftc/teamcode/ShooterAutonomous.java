package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
//@Disabled
public abstract class ShooterAutonomous extends LinearOpMode {
    final static int DISTANCE_FROM_WALL_FAR = 30;
    final static int DISTANCE_FROM_WALL_CLOSE = 23;
    Hardware506 robot;
    double sidePower;
    double forwardPower;
    double rotationPower;
    double sweeperPower;
    double launcherPower;
    int directionMultiplier;
    boolean lineFound;


    enum State {
        DRIVE_AWAY_FROM_WALL("Drive Away from Wall"),
        SHOOT_PARTICLE("Shoot Particle"),
        WAIT("Wait"),
        STOP("STOP");


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
        boolean reverseSidePower = false;
        boolean previousLineDetectedState = false;
        lineFound = false;
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
        robot.setSlidePosition(Hardware506.SLIDE_SERVO_POSITION_RIGHT_LIMIT);
        currentState = State.WAIT;
        waitTime = 5;
        nextState = State.DRIVE_AWAY_FROM_WALL;

        while (opModeIsActive()) {
            double distance = robot.getUltrasonicAverageDistance();
            sidePower = 0;
            rotationPower = 0;
            forwardPower = 0;
            sweeperPower = 0;
            launcherPower = 0;
            switch (currentState) {
                case DRIVE_AWAY_FROM_WALL:
                    sidePower = -.7;
                    rotationPower = gyroAngleCorrection();
                    if (Math.abs(robot.leftFrontMotor.getCurrentPosition()) > 200 / Hardware506.GEAR_RATIO) {
                        currentState = State.SHOOT_PARTICLE;
                        time = getRuntime();
                    }
                    break;

                case SHOOT_PARTICLE:
                    if (getRuntime() < time + 2) {
                        if (getRuntime() > time + 2 && getRuntime() < time + 3.5) {
                            sweeperPower = 1;
                        }
                        launcherPower = 1;
                    } else {
                        currentState = State.STOP;
                    }
                    break;

                case WAIT:
                    if (getRuntime() > time + waitTime) {
                        currentState = nextState;
                        time = getRuntime();
                    }
                    break;
                case STOP:
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
        if (left + right / 2.0 < 80) {
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
        int heading = robot.gyro.getTrueHeading();
        if (getDesiredColor() == RED){
            heading += 180;
        }
        heading %= 360;
        if (heading > 180) { // convert 0 - 360 range of heading to -180 - 180
            heading += 180;
            heading %= 360;
            heading -= 180;
        }

        double turnPower;
        if (Math.abs(heading - finalHeading) > 20) {
            turnPower = .5;
        } else if (Math.abs(heading - finalHeading) > 10) {
            turnPower = .3;
        } else if (Math.abs(heading - finalHeading) > 5) {
            turnPower = .1;
        } else {
            turnPower = .1;
        }
        if (Math.abs(heading - finalHeading) < 1) {
            return 0;
        }
        if (heading < finalHeading)
            return turnPower;
        else if (heading > finalHeading)
            return -turnPower;
        else
            return 0;
    }

    public boolean isParallel() {
        if (lineFound) {
            double left = robot.leftUltrasonic.getUltrasonicLevel();
            double right = robot.rightUltrasonic.getUltrasonicLevel();
            if (Math.abs(left - right) > 3) {
                {
                    return false;
                }
            }
            return true;
        } else {
            int heading = robot.gyro.getTrueHeading();
            if (heading > 180) { // convert 0 - 360 range of heading to -180 - 180
                heading += 180;
                heading %= 360;
                heading -= 180;
            }
            if (Math.abs(heading) > 5) {
                return false;
            } else {
                return true;
            }
        }
    }

    public abstract Hardware506.ColorDetected getDesiredColor();
}
