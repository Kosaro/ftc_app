
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Servo Tester", group="Iterative Opmode")  // @Autonomous(...) is the other common choice
//@Disabled
public class ServoTester extends OpMode
{
     Hardware506 robot;

    @Override
    public void init() {
        robot = new Hardware506(hardwareMap);
    }

    @Override
    public void loop() {
        double position = gamepad1.right_trigger;
        robot.armServo.setPosition(position);
        telemetry.addData("Servo Position", position);
    }
}
