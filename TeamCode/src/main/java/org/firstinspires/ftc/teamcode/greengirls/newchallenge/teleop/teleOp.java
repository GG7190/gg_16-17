package org.firstinspires.ftc.teamcode.greengirls.newchallenge.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;


import org.firstinspires.ftc.teamcode.greengirls.newchallenge.GGLibrary;

/**
 * Created by User on 8/16/2016.
 */
@TeleOp(name="teleOp", group = "teleOp")
public class teleOp extends GGLibrary {


    @Override public void loop()
    {
        //JOYSTICK ONE
        //Wheels being controlled by left and right sticks
        setRightMotors(gamepad1.left_stick_y);
        setLeftMotors(-gamepad1.right_stick_y);

        //JOYSTICK TWO
        //Lift movements in teleop
        if (gamepad2.y){
            liftUp();
        }
        else if (gamepad2.a){
            liftDown();
        }
        else {
            stopLift();
        }

        //Button Pusher movements in teleop
        if (gamepad2.right_bumper){
            buttonPushOut();
        }
        if (gamepad2.left_bumper){
            buttonPushIn();
        }

        //Funnel movement in teleop
      //  if (gamepad2.b){
          //  startFunnel();
       // }
       // else {
         //   stopFunnel();
     //   }




    }
}