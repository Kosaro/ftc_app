package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Created by okosa on 1/6/2017.
 */
@Autonomous(name = "Support Auto Blue", group = "Autonomous")
//@Disabled
public class SupportAutoBlue extends SupportAuto{


    @Override
    public Hardware506.ColorDetected getDesiredColor() {
        return Hardware506.ColorDetected.BLUE;
    }
}
