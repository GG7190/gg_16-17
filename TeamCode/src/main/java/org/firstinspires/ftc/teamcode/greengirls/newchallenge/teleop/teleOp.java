package org.firstinspires.ftc.teamcode.greengirls.newchallenge.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.greengirls.newchallenge.GGLibrary;

/**
 * Created by User on 8/16/2016.
 */
@TeleOp(name="teleOp", group = "teleOp")
public class teleOp extends GGLibrary {

    @Override
    public void loop() {

        // Right wheels will be controlled by the right stick
        // Left wheels will be controlled by the left stick
        // write the values to the motors
        setRightMotors(-gamepad1.right_stick_y);
        setLeftMotors(gamepad1.left_stick_y);

        //When collector is collecting balls
        if (gamepad1.right_bumper) {
            setMotor1(1);
        }
        //When collector is spitting balls out
        else if (gamepad1.left_bumper) {
            setMotor1(-1);
        } else {
            stopMotor1();
        }

        //open and close basket button mapping
        //tilt left
        if (gamepad2.right_bumper) {
            minServo1();
        }
        //When button x is pressed on gamepad 2 tilt right
        else if (gamepad2.left_bumper) {
            maxServo1();
        }
        //When button right bumper and left bumper on gampe pad 2 is not pressed keeps basket in middle
        else {
            midServo1();
        }

        //When button y is pressed on game pad 2 the lift extends
        if (gamepad2.b) {
            liftOut();
            //coilOut();
        }
        //When button a is pressed on game pad 2 the lift retracts
        else if (gamepad2.x) {
            liftIn();
            //coilIn();
        }
        //when button a is not pressed stop lift motor
        else {
            stoplift();
            //coilStop();
        }

        //when button x is pressed on gamepad 2 lift angle increases
        if (gamepad2.y) {
            liftUp();
        }
        //when button b is pressed on gamepad 2 lift angle decreases
        else if (gamepad2.a) {
            liftDown();
        }
        //when b and x are not pressed stop lift angle motor
        else {
            liftStop();

        }

        /*
        //when dpad left is pressed the left basket wing extends
        if (gamepad2.dpad_left){
            rightWingOut();
        }
        //when dpad right is pressed the right basket wing extends
        else if (gamepad2.dpad_right){
            leftWingOut();
        }
        //if neither game pad buttons are pressed the wings stay in the starting position
        else {
            rightWingIn();
            leftWingIn();
        }
        //when dpad up is pressed the dino arms go up
        if (gamepad2.dpad_down){
            dinoUp();
        }
        //when dpad down is pressed the dino arms go down
        else if (gamepad2.dpad_up){
            dinoDown();
        }*/


        telemetry.addData("right bumper", gamepad2.right_bumper);
        telemetry.addData("left bumper", gamepad2.left_bumper);
        telemetry.addData("dpad left", gamepad2.dpad_left);
        telemetry.addData("dpad right", gamepad2.dpad_right);

    }
}
