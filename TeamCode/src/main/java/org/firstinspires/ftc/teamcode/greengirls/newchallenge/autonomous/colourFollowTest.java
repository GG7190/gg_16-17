package org.firstinspires.ftc.teamcode.greengirls.newchallenge.autonomous;

import org.firstinspires.ftc.teamcode.greengirls.newchallenge.GGLibrary;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.ColorSensor;
/**
 * Created by Dell User on 10/20/2016.
 */
@Autonomous(name="colourFollowTest", group="Tests")
public class colourFollowTest extends LinearOpMode {
    ColorSensor sensorRGB;

    @Override
    public void runOpMode() throws InterruptedException {

        sensorRGB = hardwareMap.colorSensor.get("sensorColour");
        sensorRGB.enableLed(true);

        waitForStart();

        while (opModeIsActive()) {

            // Display the light level
            telemetry.addData("Alapha", sensorRGB.alpha());
            telemetry.update();
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }
}
