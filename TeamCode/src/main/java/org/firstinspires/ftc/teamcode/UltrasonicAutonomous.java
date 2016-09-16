package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Autonomous without vuforia, using two ultrasonic sensors and two color sensors(work in progress)
 *
 * @author Oscar Kosar-Kosarewicz
 * @version 9/15/16
 */
@Autonomous(name = "Ultrasonic Autonomous", group = "Autonomous")
//@Disabled
public class UltrasonicAutonomous extends LinearOpMode {
    Hardware506 robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Hardware506(hardwareMap);
        robot.setDriveMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();
        while (opModeIsActive()) {
            double distance = robot.getUltrasonicAverageDistance();
            telemetry.addData("Ultrasonic Left", robot.leftUltrasonic.getUltrasonicLevelMedian());
            telemetry.addData("Ultrasonic Right", robot.rightUltrasonic.getUltrasonicLevelMedian());
            telemetry.addData("Ultrasonic Average", distance);

            if (distance > 50) {
                robot.drive(1, 0, 0);
            } else if (distance > 20) {
                robot.drive(.3, 0, 0);
            } else if (distance < 10)
                robot.drive(-.3, 0, alignToWall());
            else
                robot.drive(0, -.4, alignToWall());

            telemetry.update();
            idle();
        }
        robot.leftUltrasonic.stopMedianCalculatingThread();
        robot.rightUltrasonic.stopMedianCalculatingThread();
    }


    /**
     * Returns necessary turn power to align to the wall
     *
     * @return turn power
     */
    public double alignToWall() {
        double left = robot.leftUltrasonic.getUltrasonicLevelMedian();
        double right = robot.rightUltrasonic.getUltrasonicLevelMedian();

        if (Math.abs(left - right) > 5) {
            if (left < right)
                return .2;
            else
                return -.2;
        }
        return 0;
    }
}
