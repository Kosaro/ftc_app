package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by okosa on 1/6/2017.
 */

@Autonomous(name = "Support Auto Red", group = "Autonomous")
//@Disabled
public class SupportAutoRed extends SupportAuto{


    @Override
    public Hardware506.ColorDetected getDesiredColor() {
        return Hardware506.ColorDetected.RED;
    }
}
