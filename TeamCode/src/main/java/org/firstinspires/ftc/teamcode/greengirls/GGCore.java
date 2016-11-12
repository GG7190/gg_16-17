package org.firstinspires.ftc.teamcode.greengirls;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by Green Girls on 8/2/2016.
 */
public class GGCore extends GGHardware {



    protected final static double SERVO1_MIN_RANGE  = 0.00;
    protected final static double SERVO1_MID_RANGE  = 0.45;
    protected final static double SERVO1_MAX_RANGE  = 0.90;
    boolean colourTrigger = false;
    boolean colourReady = false;
    final int threshold = 4;
    double rightSpeed = 0;
    double leftSpeed = 0;
    int errorLeft = 0;
    int errorRight = 0;
    int alpha1 = 0;
    int alpha2 = 0;
    double turnSpeed;
    Orientation angles;




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
        rightFrontMotor.setPower(-.9*power);
        rightBackMotor.setPower(power);
    }


    //stop right motors
    public void stopRightMotors(){
        rightFrontMotor.setPower(0);
        rightBackMotor.setPower(0);
    }

    //set power to left motors
    public void setLeftMotors(double power){
        leftFrontMotor.setPower(-.9*power);
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

    //public boolean checkRed (){
    //    return colorSensor.red()>0;
    //}

    //public boolean checkBlue (){return colorSensor.blue()>0;}


    //the set up of encoders
    // had to use math.round to convert a double to an int

    public void runWithColor(int encoderCount, boolean center, double speed) {

        resetEncoders();

        if (center) {


            //turn until we reach white on the left sensor
            setRightMotors(speed);
            setLeftMotors(-speed);
            while (!colourTrigger) {
                alpha1 = sensorRGB1.alpha();
                alpha2 = sensorRGB2.alpha();
                if (alpha1 > 15) {
                    // the left sensor is detecting white, go to next step.
                    setRightMotors(0);
                    setLeftMotors(0);
                    colourTrigger = true;
                }
            }
            colourTrigger = false;
            //while left color sensor value is not white then keep turning
            setRightMotors(-speed);
            setLeftMotors(speed);
            while (!colourTrigger) {
                alpha1 = sensorRGB1.alpha();
                alpha2 = sensorRGB2.alpha();
                if (alpha2 > 15) {
                    //the right sensor is detecting white, we good
                    setRightMotors(0);
                    setLeftMotors(0);
                    colourTrigger = true;
                }
            }
            colourTrigger = false;
        }
        colourReady = false;

        runWithEncoders();

        while (!colourReady) {

            //get the alpha levels for the colour sensors (alpha ~ white)
            alpha1 = sensorRGB1.alpha();
            alpha2 = sensorRGB2.alpha();

            //evaluate if the sensor is detecting more white than we want
            errorLeft = alpha1 - threshold;
            errorRight = alpha2 - threshold;

            // we don't want negative error levels
            if (errorLeft < 0) {
                errorLeft = 0;
            }
            if (errorRight < 0) {
                errorRight = 0;
            }

            //adjust the speed for the wheels according to the error levels detected
            leftSpeed = speed - (speed * (errorLeft * .1)) + (speed * (errorRight * .1));
            rightSpeed = speed + (speed * (errorLeft * .1)) - (speed * (errorRight * .1));

            //set the speed
            setLeftMotors(leftSpeed);
            setRightMotors(rightSpeed);

            //use telemetry for debugging and tweaking purposes
            telemetry.addData("leftSpeed", leftSpeed);
            telemetry.addData("rightSpeed", rightSpeed);
            telemetry.addData("Alpha1", alpha1);
            telemetry.addData("Alpha2", alpha2);
            telemetry.update();
            if (rEncoderCountReached(encoderCount)) {
                // we are done, finish up and reset stuff
                colourReady = true;
                setRightMotors(0);
                setLeftMotors(0);
                resetEncoders();
            }
        }
        colourReady = false;


    }
    void turnToHeading(String direction, double heading) {
        if (direction == "left") {
            turnSpeed = -1;
        }
        if (direction == "right") {
            turnSpeed = 1;
        }

    }

    boolean checkColor(String colour) {
        if (sensorRGB3.red() > sensorRGB3.blue()) {
            if (colour == "red") {
                return true;
            } else {
                return false;
            }
        } else {
            if (colour == "blue") {
                return true;
            } else {
                return false;
            }
        }
    }


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

