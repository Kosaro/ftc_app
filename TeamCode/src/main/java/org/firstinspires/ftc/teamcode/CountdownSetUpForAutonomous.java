package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Sets wait time during initialization for autonomous
 * a on gamepad to increase wait period and b to decrease it
 * @author Oscar Kosar-Kosarewicz
 * @version 9/13/16
 */
public class CountdownSetUpForAutonomous {
    ElapsedTime timer;
    int waitTimeIndex;
    Gamepad gamepad;
    static int[] timeIntervals = {0, 3, 5, 7, 9, 11};
    boolean previousAState;
    boolean previousBState;

    CountdownSetUpForAutonomous(Gamepad gamepad){
        this.gamepad = gamepad;
        waitTimeIndex = 0;
        previousAState = false;
        previousBState = false;
    }

    public int setUpWaitTime(){
        boolean aState = gamepad.a;
        boolean bState = gamepad.b;
        if (aState && bState){
            //do nothing
        }else if (aState != previousAState){
            if (aState){
                if (waitTimeIndex < timeIntervals.length - 1){
                    waitTimeIndex++;
                }
            }
        }else if (bState != previousBState){
            if (bState) {
                if (waitTimeIndex > 0){
                    waitTimeIndex --;
                }
            }
        }
        previousAState = aState;
        previousBState = bState;
        return timeIntervals[waitTimeIndex];
    }

    public void startTimer(){
        timer = new ElapsedTime();
    }

    public double getTimeRemaining(){
        if (timer == null){
            return -1;
        }
        return timeIntervals[waitTimeIndex] - timer.seconds();
    }
}
