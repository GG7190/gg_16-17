package org.firstinspires.ftc.teamcode.greengirls;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Green Girls on 8/2/2016.
 */
public class GGCore extends GGHardware {

    protected final static double SERVO1_MIN_RANGE  = 0.00;
    protected final static double SERVO1_MID_RANGE  = 0.45;
    protected final static double SERVO1_MAX_RANGE  = 1.00;


    //servo1 positions
    public void minServo1() {
        servo1.setPosition(SERVO1_MIN_RANGE);
    }

    public void midServo1() {
        servo1.setPosition(SERVO1_MID_RANGE);
    }

    public void maxServo1() {
        servo1.setPosition(SERVO1_MAX_RANGE);
    }

    //setters and getters for all motors for use outside this program
    //above 0= forward, below 0= backward, 0= not moving
    //right set
    public void setRightMotors(double power){
        rightFrontMotor.setPower(.9*power);
        rightBackMotor.setPower(power);
    }


    //stop right motors
    public void stopRightMotors(){
        rightFrontMotor.setPower(0);
        rightBackMotor.setPower(0);
    }

    //set power to left motors
    public void setLeftMotors(double power){
        leftFrontMotor.setPower(.9*power);
        leftBackMotor.setPower(power);
    }

    //stop left motors
    public void stopLeftMotors(){
        leftFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
    }

    //get the power to collector motor
    public double getMotor1(){
        return motor1.getPower();
    }

    //set the power to collector motor
    public void setMotor1(double power) {
        motor1.setPower(power);
    }

    //stop collector
    public void stopMotor1(){
        motor1.setPower(0);
    }

    public boolean checkRed (){
        return colorSensor.red()>0;
    }

    public boolean checkBlue (){return colorSensor.blue()>0;}


    //the set up of encoders
    // had to use math.round to convert a double to an int
    public void runWithEncoders() {
        if (leftBackMotor != null) {
            leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void resetEncoders() {

        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public boolean lEncoderCountReached(double rightCount) {
        //
        // Assume failure.
        //
        boolean l_status = false;

        //
        // Have the encoders reached the specified values?
        //
        // TODO Implement stall code using these variables.
        //
        if ((Math.abs(leftBackMotor.getCurrentPosition()) > rightCount)) {
            //
            // Set the status to a positive indication.
            //
            l_status = true;
        }
        // Return the status.
        //
        // Log.w("ENCODERCOUNTREACHED ","status " + String.valueOf(l_status));
        // Log.w("ENCODERCOUNTREACHED: ","count " + String.valueOf(rightCount));
        // Log.w("ENCODERCOUNTREACHED ", "currentPos "+String.valueOf(leftBackMotor.getCurrentPosition()));
        return l_status;

    }

    public boolean rEncoderCountReached(double rightCount)
    {
        //
        // Assume failure.
        //
        boolean l_status = false;

        //
        // Have the encoders reached the specified values?
        //
        // TODO Implement stall code using these variables.
        //
        if ((Math.abs (rightBackMotor.getCurrentPosition ()) > rightCount))
        {
            //
            // Set the status to a positive indication.
            //
            l_status = true;
        }

        //
        // Return the status.
        //
        //      Log.w("ENCODERCOUNTREACHED ","status " + String.valueOf(l_status));
        // Log.w("ENCODERCOUNTREACHED: ","count " + String.valueOf(rightCount));
        //Log.w("ENCODERCOUNTREACHED ", "currentPos "+String.valueOf(leftBackMotor.getCurrentPosition()));
        return l_status;
    }

    public boolean have_drive_encoders_reset ()
    {
        //
        // Assume failure.
        //
        boolean l_return = false;

        //
        // Have the encoders reached zero?
        //
        if (Math.abs (leftBackMotor.getCurrentPosition ())==0)
        {
            //
            // Set the status to a positive indication.
            //
            l_return = true;
        }

        //
        // Return the status.
        //
        return l_return;

    } // have_drive_encoders_reset

    //Gyroscope program


}

