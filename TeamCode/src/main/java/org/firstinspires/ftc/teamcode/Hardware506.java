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
    DcMotor leftFrontMotor;
    DcMotor rightFrontMotor;
    DcMotor leftRearMotor;
    DcMotor rightRearMotor;

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
        leftFrontMotor = (DcMotor) getDevice(dcMotor, "lf");
        leftRearMotor = (DcMotor) getDevice(dcMotor, "lr");
        rightFrontMotor = (DcMotor) getDevice(dcMotor, "rf");
        rightRearMotor = (DcMotor) getDevice(dcMotor, "rr");

        if (leftFrontMotor != null) {
            leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        if (leftRearMotor != null) {
            leftRearMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }

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

        if (leftFrontMotor != null) {
            leftFrontMotor.setPower(leftFrontPower);
        }
        if (leftRearMotor != null) {
            leftRearMotor.setPower(leftRearPower);
        }
        if (rightFrontMotor != null) {
            rightFrontMotor.setPower(rightFrontPower);
        }
        if (rightRearMotor != null) {
            rightRearMotor.setPower(rightRearPower);
        }
    }
}
