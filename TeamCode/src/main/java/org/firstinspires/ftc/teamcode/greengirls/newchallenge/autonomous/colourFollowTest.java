package org.firstinspires.ftc.teamcode.greengirls.newchallenge.autonomous;

import org.firstinspires.ftc.teamcode.greengirls.newchallenge.GGLibrary;


import org.firstinspires.ftc.teamcode.greengirls.GGHardware;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
/**
 * Created by Dell User on 10/20/2016.
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
    GGHardware hardware = new GGHardware();

    public void setRightMotors(double power){
        hardware.rightFrontMotor.setPower(power);
        hardware.rightBackMotor.setPower(power);
        hardware.motor2.setPower(-power);
    }


    //stop right motors


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








        while (opModeIsActive()) {

            alpha1 = hardware.sensorRGB1.alpha();
            alpha2 = hardware.sensorRGB2.alpha();

            errorLeft = alpha1 - threshold;
            errorRight = alpha2 - threshold;
            if (errorLeft < 0) {
                errorLeft = 0;
            }
            if (errorRight < 0) {
                errorRight = 0;
            }

            leftSpeed = speed - (speed * (errorLeft * .1)) + (speed * (errorRight * .1));
            rightSpeed = speed + (speed * (errorLeft * .1)) - (speed * (errorRight * .1));

            setLeftMotors(leftSpeed);
            setRightMotors(rightSpeed);

            telemetry.addData("leftSpeed", leftSpeed);
            telemetry.addData("rightSpeed", rightSpeed);
            telemetry.addData("Alpha1", alpha1);
            telemetry.addData("Alpha2", alpha2);
            telemetry.update();
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
        setLeftMotors(0);
        setRightMotors(0);
    }
}
