package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

/**
 * Hardware506 class for the robot. Initializes hardware and contains basic methods
 *
 * @author Oscar Kosar-Kosarewicz
 * @version 11/7/16
 */
public class Hardware506 extends Hardware {

    public final static double SLIDE_SERVO_POSITION_LEFT = .47;
    public final static double SLIDE_SERVO_POSITION_CENTER = .45 ;
    public final static double SLIDE_SERVO_POSITION_RIGHT = .35;
    public final static double SLIDE_SERVO_POSITION_LEFT_LIMIT = .52;
    public final static double SLIDE_SERVO_POSITION_RIGHT_LIMIT = .25;
    final static double GEAR_RATIO = 1;

    public final static double LIFT_SERVO_POSITION_UP = .225;
    public final static double LIFT_SERVO_POSITION_DOWN = .6;
    double LIGHT_THRESHHOLD = .04;


    DcMotorWrapper leftFrontMotor;
    DcMotorWrapper rightFrontMotor;
    DcMotorWrapper leftRearMotor;
    DcMotorWrapper rightRearMotor;
    DcMotorWrapper sweeperMotor;
    DcMotorWrapper launcherMotor;
    DcMotorWrapper liftMotor1;
    DcMotorWrapper liftMotor2;
    ServoWrapper slideServo;
    UltrasonicSensorWrapper leftUltrasonic;
    UltrasonicSensorWrapper rightUltrasonic;
    GyroSensorWrapper gyro;
    OpticalDistanceSensorWrapper lineDetectorCenter;
    OpticalDistanceSensorWrapper lineDetectorLeft;
    OpticalDistanceSensorWrapper lineDetectorRight;
    MRCompassSensorWrapper compass;

    ColorSensorWrapper beaconColorSensor;
    ServoWrapper liftServo;

    enum ColorDetected {
        BLUE("Blue"),
        RED("Red"),
        NONE("None");

        String description;

        public String toString() {
            return description;
        }

        ColorDetected(String description) {
            this.description = description;
        }
    }

    enum DriveMode {
        MECANUM,
        MECANUM_WITH_GYRO
    }


    DriveMode currentDriveMode;
    boolean reverseDriveTrain;

    /**
     * Constructor initializes hardware map
     *
     * @param hardwareMap robot's hardware map
     */
    public Hardware506(HardwareMap hardwareMap) {
        super(hardwareMap);
    }


    @Override
    public void initializeRobot() {
        leftFrontMotor = new DcMotorWrapper(getDevice(dcMotor, "lf"));
        leftRearMotor = new DcMotorWrapper(getDevice(dcMotor, "lr"));
        rightFrontMotor = new DcMotorWrapper(getDevice(dcMotor, "rf"));
        rightRearMotor = new DcMotorWrapper(getDevice(dcMotor, "rr"));
        sweeperMotor = new DcMotorWrapper(getDevice(dcMotor, "sm"));
        launcherMotor = new DcMotorWrapper(getDevice(dcMotor, "lm"));
        leftUltrasonic = new UltrasonicSensorWrapper(getDevice(ultrasonicSensor, "lu"));
        rightUltrasonic = new UltrasonicSensorWrapper(getDevice(ultrasonicSensor, "ru"));
        gyro = new GyroSensorWrapper(getDevice(gyroSensor, "g"));
        lineDetectorCenter = new OpticalDistanceSensorWrapper(getDevice(opticalDistanceSensor, "ldc"));
        lineDetectorRight = new OpticalDistanceSensorWrapper(getDevice(opticalDistanceSensor, "ldr"));
        lineDetectorLeft = new OpticalDistanceSensorWrapper(getDevice(opticalDistanceSensor, "ldl"));
        beaconColorSensor = new ColorSensorWrapper(getDevice(colorSensor, "bc"));
        slideServo = new ServoWrapper(getDevice(servo, "as"));
        liftServo = new ServoWrapper(getDevice(servo, "ls"));
        liftMotor1 = new DcMotorWrapper(getDevice(dcMotor, "liftm1"));
        liftMotor2 = new DcMotorWrapper(getDevice(dcMotor, "liftm2"));
        compass = new MRCompassSensorWrapper(getDevice(modernRoboticsI2cCompassSensor, "cs"));

        leftFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftRearMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRearMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        sweeperMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        launcherMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        liftMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        liftMotor2.setDirection(DcMotorSimple.Direction.FORWARD);

        beaconColorSensor.enableLed(false);
        setSlidePosition(SLIDE_SERVO_POSITION_LEFT_LIMIT);
        liftServo.setPosition(LIFT_SERVO_POSITION_UP);
        reverseDriveTrain = false;
    }

    private void powerDriveTrain(double forwardValue, double sideValue, double rotationValue) {
        double leftFrontPower;
        double leftRearPower;
        double rightFrontPower;
        double rightRearPower;

        leftFrontPower = forwardValue + sideValue + rotationValue;
        leftRearPower = forwardValue - sideValue + rotationValue;
        rightFrontPower = forwardValue - sideValue - rotationValue;
        rightRearPower = forwardValue + sideValue - rotationValue;

        double max = Double.MIN_VALUE;
        if (Math.abs(leftFrontPower) > max)
            max = Math.abs(leftFrontPower);
        if (Math.abs(leftRearPower) > max)
            max = Math.abs(leftRearPower);
        if (Math.abs(rightFrontPower) > max)
            max = Math.abs(rightFrontPower);
        if (Math.abs(rightRearPower) > max)
            max = Math.abs(rightRearPower);
        if (max > 1) {
            leftFrontPower /= max;
            leftRearPower /= max;
            rightFrontPower /= max;
            rightRearPower /= max;
        }

        leftFrontMotor.setPower(leftFrontPower);
        leftRearMotor.setPower(leftRearPower);
        rightFrontMotor.setPower(rightFrontPower);
        rightRearMotor.setPower(rightRearPower);
    }

    public void drive(double forwardValue, double sideValue, double rotationValue) {
        forwardValue = Range.clip(forwardValue, -1, 1);
        sideValue = Range.clip(sideValue, -1, 1);
        rotationValue = Range.clip(rotationValue, -1, 1);
        switch (currentDriveMode) {
            case MECANUM:
                if (reverseDriveTrain) {
                    forwardValue = -forwardValue;
                    sideValue = -sideValue;
                }
                powerDriveTrain(forwardValue, sideValue, rotationValue);
                break;
            case MECANUM_WITH_GYRO:
                double directionRelativeToRobot;
                if (forwardValue == 0) {
                    if (sideValue > 0)
                        directionRelativeToRobot = Math.PI / 2;
                    else
                        directionRelativeToRobot = -Math.PI / 2;
                } else
                    directionRelativeToRobot = Math.atan(sideValue / forwardValue);
                double velocity = Math.sqrt(Math.pow(forwardValue, 2) + Math.pow(sideValue, 2));
                if (forwardValue < 0) {
                    directionRelativeToRobot += Math.PI;
                }
                double adjustedDirection = directionRelativeToRobot - gyro.getHeading() * Math.PI / 180;
                double forwardPower = velocity * Math.cos(adjustedDirection);
                double sidePower = velocity * Math.sin(adjustedDirection);
                powerDriveTrain(forwardPower, sidePower, rotationValue);
                break;
        }
    }

    public void setDriveMotorMode(DcMotor.RunMode runMode) {
        leftFrontMotor.setMode(runMode);
        leftRearMotor.setMode(runMode);
        rightRearMotor.setMode(runMode);
        rightFrontMotor.setMode(runMode);
    }

    public void setDriveMode(DriveMode driveMode) {
        currentDriveMode = driveMode;
    }

    public double getUltrasonicAverageDistance() {
        double left = leftUltrasonic.getUltrasonicLevel();
        double right = rightUltrasonic.getUltrasonicLevel();

        if (right < 5 || right == 255) {
            left *= 2;
            right = 0;
        }

        if (left < 5 || left == 255) {
            right *= 2;
            left = 0;
        }

        if (left == 0 && right == 0)
            left = 500;
        return (left + right) / 2.0;
    }

    private boolean isLineDetected(OpticalDistanceSensorWrapper sensor) {
        if (sensor.getLightDetected() > LIGHT_THRESHHOLD) {
            return true;
        }
        return false;
    }

    public boolean isLineDetectedLeft() {
        return isLineDetected(lineDetectorLeft);
    }

    public boolean isLineDetectedRight() {
        return isLineDetected(lineDetectorRight);
    }

    public boolean isLineDetectedCenter() {
        return isLineDetected(lineDetectorCenter);
    }

    public LineDetected getLineDetected() {
        if (isLineDetectedLeft()) {
            return LineDetected.LEFT;
        } else if (isLineDetectedRight()) {
            return LineDetected.RIGHT;
        } else if (isLineDetectedCenter()) {
            return LineDetected.CENTER;
        } else {
            return LineDetected.NONE;
        }
    }


    enum LineDetected {
        LEFT("Left"),
        CENTER("Center"),
        RIGHT("Right"),
        NONE("None");

        String description;

        public String toString() {
            return description;
        }

        LineDetected(String description) {
            this.description = description;
        }

    }

    public void setReverseDriveTrain(boolean reverseDriveTrain) {
        this.reverseDriveTrain = reverseDriveTrain;
    }

    public ColorDetected getBeaconColor() {
        double blueColorThreshold = 3;
        double redColorThreshold = 2;
        double blueStrength = beaconColorSensor.blue();
        double redStrength = beaconColorSensor.red();
        if (blueStrength >= blueColorThreshold && blueStrength > redStrength) {
            return ColorDetected.BLUE;
        } else if (redStrength >= redColorThreshold && blueStrength < redStrength) {
            return ColorDetected.RED;
        } else
            return ColorDetected.NONE;
    }

    public void setSlidePosition(double position) {
        position = Range.clip(position, SLIDE_SERVO_POSITION_RIGHT_LIMIT, SLIDE_SERVO_POSITION_LEFT_LIMIT);
        slideServo.setPosition(position);
    }

}
