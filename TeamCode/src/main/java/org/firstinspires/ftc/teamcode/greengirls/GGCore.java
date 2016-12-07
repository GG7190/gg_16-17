package org.firstinspires.ftc.teamcode.greengirls;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cAddr;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

/**
 * Created by Green Girls on 8/2/2016.
 */
public class GGCore extends GGHardware {



    protected final static double SERVO1_MIN_RANGE  = 0.10;
    protected final static double SERVO1_MID_RANGE  = 0.45;
    protected final static double SERVO1_MAX_RANGE  = 0.99;
    protected final static double SERVO2_MIN_RANGE  = 0.10;
    protected final static double SERVO2_MID_RANGE  = 0.45;
    protected final static double SERVO2_MAX_RANGE  = 0.99;
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
    double newHeading;
    double currentHeading;
    boolean reached;
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

    public void minServo2() {
        servo2.setPosition(SERVO2_MIN_RANGE);
    }

    public void midServo2() {
        servo2.setPosition(SERVO2_MID_RANGE);
    }

    public void maxServo2() {
        servo2.setPosition(SERVO2_MAX_RANGE);
    }

    //setters and getters for all motors for use outside this program
    //above 0= forward, below 0= backward, 0= not moving
    //right set
    public void setRightMotors(double power){
        rightFrontMotor.setPower(power);
        rightBackMotor.setPower(-power);
    }


    //stop right motors
    public void stopRightMotors(){
        rightFrontMotor.setPower(0);
        rightBackMotor.setPower(0);
    }

    //set power to left motors
    public void setLeftMotors(double power){
        leftFrontMotor.setPower(power);
        leftBackMotor.setPower(-power);
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
            setLeftMotors(speed);
            while (!colourTrigger) {
                alpha1 = sensorRGB1.alpha();
                al`pha2 = sensorRGB2.alpha();
                if (alpha1 > 15) {
                    // the left sensor is detecting white, go to next step.
                    setRightMotors(0);
                    setLeftMotors(0);
                    colourTrigger = true;
                }
                telemetry.addData("alpha1", alpha1);
                telemetry.addData("alpha2", alpha2);
                telemetry.update();
            }
            colourTrigger = false;
            //while left color sensor value is not white then keep turning
            setRightMotors(speed);
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
                telemetry.addData("alpha1", alpha1);
                telemetry.addData("alpha2", alpha2);
                telemetry.update();
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
            if (lEncoderCountReached(encoderCount)) {
                // we are done, finish up and reset stuff
                colourReady = true;
                setRightMotors(0);
                setLeftMotors(0);
                resetEncoders();
            }
        }
        colourReady = false;


    }

    public void waitForEncodersReduxVersionTwoPointZero(int encoderAmount) {
        reached = false;
        while (!reached) {
            if (Math.abs(leftBackMotor.getCurrentPosition()) > encoderAmount) {
                reached = true;
            } else {
                telemetry.addData("curPos", Math.abs(leftBackMotor.getCurrentPosition()));
                telemetry.update();
            }
        }
        reached = false;
    }

    public void turnToHeading(double heading)
    {
        if (heading < 0)
        {
            turnSpeed = -1;
        }
        if (heading > 0)
        {
            turnSpeed = 1;
        }

        currentHeading = getHeading();

        newHeading = currentHeading + heading;

        setRightMotors(turnSpeed);
        setLeftMotors(-turnSpeed);

        while (currentHeading != newHeading) {
            currentHeading = getHeading();
        }
        setRightMotors(0);
        setLeftMotors(0);

    }

    public double getHeading() {
        angles   = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        return AngleUnit.DEGREES.normalize(angles.firstAngle);



    }

    public void checkColor(String colour)
    {
        if (sensorRGB3.red() > sensorRGB3.blue())
        {
            if (colour == "red")
            {
                maxServo1();
                runWithEncoders();

                setLeftMotors(1);
                setRightMotors(-1);

                waitForEncodersReduxVersionTwoPointZero(850);

                resetEncoders();
                stopLeftMotors();
                stopRightMotors();

            }
            else
            {
                maxServo2();
                runWithEncoders();

                setLeftMotors(1);
                setRightMotors(1);

                waitForEncodersReduxVersionTwoPointZero(850);

                resetEncoders();
                stopLeftMotors();
                stopRightMotors();

            }
        }
        else
        {
            if (colour == "blue")
            {
                maxServo1();
                runWithEncoders();

                setLeftMotors(1);
                setRightMotors(1);

                waitForEncodersReduxVersionTwoPointZero(1500);

                resetEncoders();
                stopLeftMotors();
                stopRightMotors();

            }
            else
            {
                maxServo2();
                runWithEncoders();

                setLeftMotors(1);
                setRightMotors(1);

                waitForEncodersReduxVersionTwoPointZero(1500);

                resetEncoders();
                stopLeftMotors();
                stopRightMotors();

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

