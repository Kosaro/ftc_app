package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by okosa on 11/2/2016.
 */
@Autonomous(name = "Red Ultrasonic Autonomous", group = "Autonomous")
//@Disabled
public class UltrasonicAutonomousRed extends UltrasonicAutonomous{
    @Override
    public Hardware506.ColorDetected getDesiredColor() {
        return Hardware506.ColorDetected.RED;
    }
}
