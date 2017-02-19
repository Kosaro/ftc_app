package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static org.firstinspires.ftc.teamcode.Hardware506.ColorDetected.RED;

/**
 * Created by okosa on 1/6/2017.
 */
@Autonomous(name = "Ultrasonic Autonomous", group = "Autonomous")
@Disabled

public abstract class SupportAuto extends LinearOpMode{
    Hardware506 robot;
    @Override
    public void runOpMode() throws InterruptedException {
        int directonMultiplier;

        if (getDesiredColor() == RED){
            directonMultiplier = 1;
        }
        else
        directonMultiplier = -1;

        robot = new Hardware506(hardwareMap);

        waitForStart();
        wait(7500);
        while (robot.leftFrontMotor.getCurrentPosition() < 5000){
            robot.drive(1.0, 0.0, 0.0);
            idle();
        }
        robot.drive(0, 0, 0);
        while(((robot.gyro.getHeading() < 89) || (robot.gyro.getHeading() > 91)) && opModeIsActive()){
            robot.drive(0,0,turn(90 * directonMultiplier));
            idle();
        }
        int l = robot.leftFrontMotor.getCurrentPosition();
        while ((robot.leftFrontMotor.getCurrentPosition() - l < 2500) && opModeIsActive()){
            robot.drive(1.0, 0.0, 0.0);
            idle();
        }
        robot.drive(0, 0, 0);
        robot.launcherMotor.setPower(1);
        while(opModeIsActive() && (robot.launcherMotor.getCurrentPosition() < 1120))
        {
            robot.launcherMotor.setPower(1);
            idle();
        }
        robot.launcherMotor.setPower(0);

    }





    public double turn(double finalHeading) {
        finalHeading %= 360;
        if (finalHeading > 180) {
            finalHeading -= 360;
        } else if (finalHeading < -180) {
            finalHeading += 360;
        }

        int heading = robot.gyro.getTrueHeading();
        /**
        if (getDesiredColor() == RED) {
            heading += 180;
            heading %= 360;
        }
         **/

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

    public abstract Hardware506.ColorDetected getDesiredColor();
}
