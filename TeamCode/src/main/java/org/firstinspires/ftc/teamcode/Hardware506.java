package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Hardware506 class for the robot. Initializes hardware and contains basic methods
 *
 * @author Oscar Kosar-Kosarewicz
 * @version 9/10/16
 */
public class Hardware506 extends Hardware {
    DcMotorWrapper leftFrontMotor;
    DcMotorWrapper rightFrontMotor;
    DcMotorWrapper leftRearMotor;
    DcMotorWrapper rightRearMotor;
    UltrasonicSensorWrapper leftUltrasonic;
    UltrasonicSensorWrapper rightUltrasonic;
    GyroSensorWrapper gyro;

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
        leftUltrasonic = new UltrasonicSensorWrapper(getDevice(ultrasonicSensor, "lu"));
        rightUltrasonic = new UltrasonicSensorWrapper(getDevice(ultrasonicSensor, "ru"));
        gyro = new GyroSensorWrapper(getDevice(gyroSensor, "g"));

        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRearMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRearMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);

    }

    public void drive(double forwardValue, double sideValue, double rotationValue) {
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

    public void setDriveMotorMode(DcMotor.RunMode runMode){
        leftFrontMotor.setMode(runMode);
        leftRearMotor.setMode(runMode);
        rightRearMotor.setMode(runMode);
        rightFrontMotor.setMode(runMode);
    }

    public double getUltrasonicAverageDistance(){
        return (leftUltrasonic.getUltrasonicLevelMedian() + rightUltrasonic.getUltrasonicLevelMedian()) / 2;
    }
}
