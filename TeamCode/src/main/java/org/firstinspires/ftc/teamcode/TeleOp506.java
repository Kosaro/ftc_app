package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by okosa on 11/3/2016.
 */
@TeleOp(name = "TeleOp", group = "Iterative Opmode")
public class TeleOp506 extends OpMode {
    Hardware506 robot;
    boolean useGyro;
    boolean previousBackState;
    boolean servoDown;
    boolean reverseDriveTrain;

    @Override
    public void init() {
        robot = new Hardware506(hardwareMap);
        robot.setDriveMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
         useGyro = true;
        previousBackState = false;
        servoDown = false;
        telemetry.addData("Gyro", "Calibrating");
        robot.gyro.calibrate();
    }

    @Override
    public void init_loop() {
        if (!robot.gyro.isCalibrating()){
            telemetry.addData("Gyro", "Initialized");
        }
    }

    @Override
    public void start() {
        telemetry.clearAll();
    }

    @Override
    public void loop() {
        boolean backState = gamepad1.back;
        if (backState != previousBackState){
            if (backState){
                useGyro = !useGyro;
            }
            previousBackState = backState;
        }

        if (useGyro){
            robot.setDriveMode(Hardware506.DriveMode.MECANUM_WITH_GYRO);
            telemetry.addData("Drive Mode", "Gyro");
        }else{
            robot.setDriveMode(Hardware506.DriveMode.MECANUM);
            telemetry.addData("Drive Mode", "Standard");
        }

        if(gamepad1.dpad_down){
            reverseDriveTrain = true;
        }
        else if (gamepad1.dpad_up){
            reverseDriveTrain = false;
        }

        if(gamepad2.dpad_down){
            servoDown = true;
        }
        else if (gamepad2.dpad_up){
            servoDown = false;
        }

        if (gamepad1.x){
            robot.gyro.centerOffset();
        }

        robot.setReverseDriveTrain(reverseDriveTrain);
        robot.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        robot.setArmPositionDown(servoDown);
        robot.sweeperMotor.setPower(gamepad2.left_stick_y);
        robot.launcherMotor.setPower(gamepad2.right_stick_y);
        telemetry.addData("Gyro Heading", robot.gyro.getHeading());

    }
}
