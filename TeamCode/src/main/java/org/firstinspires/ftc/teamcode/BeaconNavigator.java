package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

/**
 * Navigates to beacon with 506 robot
 * @author Oscar Kosar-Kosarewicz
 * @version 9/13/16
 */
public class BeaconNavigator {
    Hardware506 robot;
    private VuforiaLocalizer vuforia;

    float mmPerInch        = 25.4f;
    float mmBotWidth       = 18 * mmPerInch;
    float mmFTCFieldWidth  = (12*12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels

    BeaconNavigator(Hardware506 robot){
        this.robot = robot;
        initVuforia();
    }

    private void initVuforia(){
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = robot.VUFORIA_LICENSE_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables images = this.vuforia.loadTrackablesFromAsset("FTC_2016-17");
        VuforiaTrackable wheelsTarget = images.get(0);

        VuforiaTrackable toolsTarget  = images.get(1);

        VuforiaTrackable legosTarget = images.get(2);

        VuforiaTrackable gearsTarget  = images.get(3);

        /** For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(images);


    }
}
