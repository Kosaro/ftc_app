package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Tester for Beacon Navigator
 *
 * @author Oscar Kosar-Kosarewicz
 * @version 9/20/16
 */
@Autonomous(name="Beacon Navigator Tester", group ="Tester")
//@Disabled
public class BeaconNavigatorTester extends LinearOpMode {
    Hardware506 robot;
    BeaconNavigator beaconNavigator;


    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Hardware506(hardwareMap);
        beaconNavigator = new BeaconNavigator(robot);
    }
}
