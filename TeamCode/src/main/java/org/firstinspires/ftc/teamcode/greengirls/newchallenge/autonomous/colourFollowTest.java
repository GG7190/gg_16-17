package org.firstinspires.ftc.teamcode.greengirls.newchallenge.autonomous;

import org.firstinspires.ftc.teamcode.greengirls.GGCore;
import org.firstinspires.ftc.teamcode.greengirls.newchallenge.GGLibrary;


import org.firstinspires.ftc.teamcode.greengirls.GGHardware;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * Created by Dell User on 10/20/2016.
 * leftSpeed = speed - (speed * (errorLeft * .1)) + (speed * (errorRight * .1))
 * rightSpeed = speed + (speed * (errorLeft * .1)) - (speed * (errorRight * .1))
 * Basically, this line follower uses an error correction equation to correct itself.
 * The degree it corrects scales off of the alpha (light/white) the colour sensors detect for each side
 * The scaling is %10 of the amount above the threshold of alpha.
 * For example, the theory is that if the left colour sensor is detecting an alpha level of 7,
 * the left wheels would slow down by .3 (if the threshold is left at a level of 4),
 * and the right wheels would speed up by .3 and vice-versa.
 * This idea of a speed-adjusting line follower tries to negate the 'jumpy'-ness of some line follower code.
 */
@Autonomous(name="colourFollowTest", group="Tests")
public class colourFollowTest extends LinearOpMode {
    double speed;

    int threshold;
    int alpha1;
    int alpha2;
    int errorLeft;
    int errorRight;
    double leftSpeed;
    double rightSpeed;
    //we need hardware mapping
    GGHardware hardware = new GGHardware();

    public void setRightMotors(double power){
        hardware.rightFrontMotor.setPower(power);
        hardware.rightBackMotor.setPower(power);
        hardware.motor2.setPower(-power);
    }

    //set power to left motors
    public void setLeftMotors(double power){
        hardware.leftFrontMotor.setPower(power);
        hardware.leftBackMotor.setPower(power);
        hardware.motor1.setPower(-power);
    }

    @Override
    public void runOpMode() throws InterruptedException {


        hardware.init();

        hardware.sensorRGB1.enableLed(true);
        hardware.sensorRGB2.enableLed(true);

        speed = 1;
        threshold = 4;



        waitForStart();

        setLeftMotors(1);
        setRightMotors(1);

        while (opModeIsActive()) {

            //get the alpha levels for the colour sensors (alpha ~ white)
            alpha1 = hardware.sensorRGB1.alpha();
            alpha2 = hardware.sensorRGB2.alpha();

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
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }

        //stop
        setLeftMotors(0);
        setRightMotors(0);
    }
}
