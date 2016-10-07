package org.firstinspires.ftc.teamcode.greengirls.newchallenge;

import org.firstinspires.ftc.teamcode.greengirls.GGCore;

/**
 * Created by User on 8/2/2016.
 */
public class GGLibrary extends GGCore {





    //HIs






      /*  // add GGLibrary specific code here
        public void leftWingOut() {
            servo2.setPosition(LWING_MIN_RANGE);}

        public void leftWingIn() {
            servo2.setPosition(LWING_MAX_RANGE);
        }

        public void rightWingOut() {
            servo3.setPosition(RWING_MIN_RANGE);
        }
        public void rightWingIn() {
            servo3.setPosition(RWING_MAX_RANGE);
        }
        public void dinoUp(){
            servo4.setPosition(LDINO_MIN_RANGE);
            servo5.setPosition(RDINO_MIN_RANGE);
        }

        public void dinoDown (){
            servo4.setPosition(LDINO_MAX_RANGE);
            servo5.setPosition(RDINO_MAX_RANGE);
        }*/

    //lift out
    public void liftOut() {
        motor3.setPower(-1);
        motor4.setPower(1);
    }

    //lift in
    public void liftIn() {
        motor3.setPower(1);
        motor4.setPower(-1);
    }

    //stop lift motor
    public void stoplift() {
        motor3.setPower(0);
        motor4.setPower(0);
        motor2.setPower(0);
    }

    //lift angle up
    public void liftUp() {
        motor2.setPower(1);
    }

    //lift angle down
    public void liftDown() {
        motor2.setPower(-1);
    }

    //angle motor stop
    public void liftStop() {
        motor2.setPower(0);
    }
}