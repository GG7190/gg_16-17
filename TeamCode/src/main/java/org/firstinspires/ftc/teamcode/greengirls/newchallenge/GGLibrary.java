package org.firstinspires.ftc.teamcode.greengirls.newchallenge;

import org.firstinspires.ftc.teamcode.greengirls.GGCore;

/**
 * Created by User on 8/2/2016.
 */
public class GGLibrary extends GGCore {

        //Set positions for button pushers
        public void buttonPushOut(){
            servo1.setPosition(SERVO1_MIN_RANGE);
            servo2.setPosition(SERVO1_MAX_RANGE);
        }
        public void buttonPushIn(){
            servo1.setPosition(SERVO1_MAX_RANGE);
            servo2.setPosition(SERVO1_MIN_RANGE);
        }


    //lift up
    public void liftUp() {
        motor1.setPower(-1);
        motor2.setPower(1);
    }

    //lift down
    public void liftDown() {
        motor1.setPower(1);
        motor2.setPower(-1);
    }

    //stop lift motor
    public void stopLift() {
        motor1.setPower(0);
        motor2.setPower(0);

    }

    //Start Funnel
    public void startFunnel(){
        motor3.setPower(1);
    }

    //Stop Funnel
    public void stopFunnel(){
        motor3.setPower(0);
    }

}