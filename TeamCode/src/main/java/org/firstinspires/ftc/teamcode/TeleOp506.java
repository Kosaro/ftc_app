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
         useGyro = false;
        previousBackState = false;
        servoDown = false;
        telemetry.addData("Gyro", "Calibrating");
        robot.gyro.calibrate();
        robot.liftServo.setPosition(Hardware506.LIFT_SERVO_POSITION_UP);
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
            robot.liftServo.setPosition(Hardware506.LIFT_SERVO_POSITION_DOWN);
        }
        else if (gamepad2.dpad_up){
            servoDown = false;
            robot.liftServo.setPosition(Hardware506.LIFT_SERVO_POSITION_UP);
        }

        if (gamepad1.x){
            robot.gyro.centerOffset();
        }

        if (gamepad2.left_trigger > 0 ^ gamepad2.right_trigger > 0){
            robot.liftMotor.setPower(-gamepad2.left_trigger + gamepad2.right_trigger);
        }

        robot.setReverseDriveTrain(reverseDriveTrain);
        robot.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        //(servoDown);
        robot.sweeperMotor.setPower(gamepad2.left_stick_y);
        robot.launcherMotor.setPower(gamepad2.right_stick_y);
        robot.setSlidePosition(Hardware506.SLIDE_SERVO_POSITION_LEFT_LIMIT);
        telemetry.addData("Gyro Heading", robot.gyro.getHeading());

    }
}
