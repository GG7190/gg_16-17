package org.firstinspires.ftc.teamcode.greengirls.newchallenge.teleop;

import android.graphics.Path;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;



import org.firstinspires.ftc.teamcode.greengirls.GGHardware;
import org.firstinspires.ftc.teamcode.greengirls.newchallenge.GGLibrary;

/**
 * Created by User on 8/16/2016.
 */
@TeleOp(name="teleOp", group = "teleOp")
public class teleOp extends GGLibrary {

    double rSpeed = 0;
    double lSpeed = 0;
    int reverse = 1;
    @Override public void loop()
    {
        //JOYSTICK ONE
        //Wheels being controlled by left and right sticks
        // (( -Y - Z ) * ((-slider+1)/2))) * fluff;





        //RIGHT 1 = 1
        //LEFT -1 = 1
        if (gamepad1.a) {
            reverse = reverse * -1;

        }

        rSpeed = reverse*(( gamepad1.right_stick_y + gamepad1.right_stick_x)) * 1;
        lSpeed = reverse*(( gamepad1.right_stick_y - gamepad1.right_stick_x)) * -1;
        //rSpeed = -gamepad1.right_stick_y;
        //lSpeed = gamepad1.left_stick_y;

        setRightMotors(rSpeed);
        setLeftMotors(lSpeed);

        telemetry.addData("rSpeed", rSpeed);
        telemetry.addData("lSpeed", lSpeed);
        telemetry.addData("rightY", gamepad1.right_stick_y);
        telemetry.addData("rightX", gamepad1.right_stick_x);
        telemetry.update();

        //JOYSTICK TWO
   //    Lift movements in teleop
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

       // Funnel movement in teleop
      if (gamepad2.b) {
          startFunnel();
      } else {
          stopFunnel();
      }
    }
}