package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Hardware506 class for the robot. Initializes hardware and contains basic methods
 *
 * @author Oscar Kosar-Kosarewicz
 * @version 11/7/16
 */
public class Hardware506 extends Hardware {

    final static double ARM_SERVO_POSITION_UP = .8;
    final static double ARM_SERVO_POSITION_DOWN = .26;
    final static double GEAR_RATIO = 2;


    DcMotorWrapper leftFrontMotor;
    DcMotorWrapper rightFrontMotor;
    DcMotorWrapper leftRearMotor;
    DcMotorWrapper rightRearMotor;
    DcMotorWrapper sweeperMotor;
    DcMotorWrapper launcherMotor;
    ServoWrapper armServo;
    UltrasonicSensorWrapper leftUltrasonic;
    UltrasonicSensorWrapper rightUltrasonic;
    GyroSensorWrapper gyro;
    OpticalDistanceSensorWrapper lineDetector;
    ColorSensorWrapper beaconColorSensor;

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
        sweeperMotor = new DcMotorWrapper(getDevice(dcMotor, "sm")) ;
        launcherMotor = new DcMotorWrapper(getDevice(dcMotor, "lm"));
        leftUltrasonic = new UltrasonicSensorWrapper(getDevice(ultrasonicSensor, "lu"));
        rightUltrasonic = new UltrasonicSensorWrapper(getDevice(ultrasonicSensor, "ru"));
        gyro = new GyroSensorWrapper(getDevice(gyroSensor, "g"));
        lineDetector = new OpticalDistanceSensorWrapper(getDevice(opticalDistanceSensor, "ld"));
        beaconColorSensor = new ColorSensorWrapper(getDevice(colorSensor, "bc"));
        armServo = new ServoWrapper(getDevice(servo, "as"));

        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRearMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRearMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        sweeperMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        launcherMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        beaconColorSensor.enableLed(false);
        setArmPositionDown(false);
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
        switch (currentDriveMode) {
            case MECANUM:
                if (reverseDriveTrain){
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
        if (left == 0) {
            right *= 2;
        }
        if (right == 0)
            left *= 2;

        if (left == 0 && right == 0)
            left = 500;
        return (left + right) / 2.0;
    }

    public boolean isLineDetected() {
        double lightThreshold = .20;
        if (lineDetector.getLightDetected() > lightThreshold) {
            return true;
        }
        return false;
    }

    public void setReverseDriveTrain(boolean reverseDriveTrain) {
        this.reverseDriveTrain = reverseDriveTrain;
    }

    public ColorDetected getBeaconColor() {
        double blueColorThreshold = 5;
        double redColorThreshold = 5;
        double blueStrength = beaconColorSensor.blue();
        double redStrength = beaconColorSensor.red();
        if (blueStrength > blueColorThreshold && blueStrength > redStrength) {
            return ColorDetected.BLUE;
        } else if (redStrength > redColorThreshold && blueStrength < redStrength) {
            return ColorDetected.RED;
        } else
            return ColorDetected.NONE;
    }

    public void setArmPositionDown(boolean b) {
        if (b) {
            armServo.setPosition(ARM_SERVO_POSITION_DOWN);
        } else
            armServo.setPosition(ARM_SERVO_POSITION_UP);
    }
}
