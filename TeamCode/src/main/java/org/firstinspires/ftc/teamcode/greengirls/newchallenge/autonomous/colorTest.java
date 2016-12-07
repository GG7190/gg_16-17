package org.firstinspires.ftc.teamcode.greengirls.newchallenge.autonomous;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;

/**
 * Created by User on 9/26/2016.
 */

@TeleOp(name = "colorTest", group = "Tests")
public class colorTest extends LinearOpMode {
    ColorSensor sensorRGB1; ColorSensor sensorRGB2; ColorSensor sensorRGB3;

    I2cAddr sensorRGB2Addr = new I2cAddr(0x10);
    I2cAddr sensorRGB3Addr = new I2cAddr(0x12);



    @Override
    public void runOpMode() throws InterruptedException {

        // write some device information (connection info, name and type)
        // to the log file.
        hardwareMap.logDevices();

        // get a reference to our ColorSensor object.
        sensorRGB1 = hardwareMap.colorSensor.get("sensorColour1");
        sensorRGB2 = hardwareMap.colorSensor.get("sensorColour2");
        sensorRGB3 = hardwareMap.colorSensor.get("sensorColour3");



        // bEnabled represents the state of the LED.
        boolean bEnabled = true;

        // turn the LED on in the beginning, just so user will know that the sensor is active.
        sensorRGB1.enableLed(true);
        sensorRGB2.enableLed(true);

        // wait one cycle.
        waitOneFullHardwareCycle();

        // wait for the start button to be pressed.
        waitForStart();

        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};
        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(R.id.RelativeLayout);

        // bPrevState and bCurrState represent the previous and current state of the button.
        boolean bPrevState = false;
        boolean bCurrState = false;

        // while the op mode is active, loop and read the RGB data.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive()) {
            // check the status of the x button on either gamepad.
            bCurrState = gamepad1.x || gamepad2.x;

            // check for button state transitions.
            if (bCurrState == true && bCurrState != bPrevState) {
                // button is transitioning to a pressed state.

                // print a debug statement.
                DbgLog.msg("MY_DEBUG - x button was pressed!");

                // update previous state variable.
                bPrevState = bCurrState;

                // on button press, enable the LED.
                bEnabled = true;

                // turn on the LED.
                sensorRGB1.enableLed(bEnabled);
                sensorRGB2.enableLed(bEnabled);
                sensorRGB3.enableLed(bEnabled);
            } else if (bCurrState == false && bCurrState != bPrevState) {
                // button is transitioning to a released state.

                // print a debug statement.
                DbgLog.msg("MY_DEBUG - x button was released!");

                // update previous state variable.
                bPrevState = bCurrState;

                // on button press, enable the LED.
                bEnabled = false;

                // turn off the LED.
                sensorRGB1.enableLed(false);
                sensorRGB2.enableLed(false);
                sensorRGB3.enableLed(false);
            }

            // convert the RGB values to HSV values.
            //Color.RGBToHSV((colorSensor.red() * 8), (colorSensor.green() * 8), (colorSensor.blue() * 8), hsvValues);
            Color.RGBToHSV(sensorRGB1.red() * 8, sensorRGB1.green() * 8, sensorRGB1.blue() * 8, hsvValues);

            // send the info back to driver station using telemetry function.
            telemetry.addData("Clear", sensorRGB1.alpha());
            telemetry.addData("Red  ", sensorRGB1.red());
            telemetry.addData("Green", sensorRGB1.green());
            telemetry.addData("Blue ", sensorRGB1.blue());
            telemetry.addData("Clear", sensorRGB2.alpha());
            telemetry.addData("Red  ", sensorRGB2.red());
            telemetry.addData("Green", sensorRGB2.green());
            telemetry.addData("Blue ", sensorRGB2.blue());
            telemetry.addData("Clear", sensorRGB3.alpha());
            telemetry.addData("Red  ", sensorRGB3.red());
            telemetry.addData("Green", sensorRGB3.green());
            telemetry.addData("Blue ", sensorRGB3.blue());
            telemetry.update();

            // change the background color to match the color detected by the RGB sensor.
            // pass a reference to the hue, saturation, and value array as an argument
            // to the HSVToColor method.
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                }
            });

            // wait a hardware cycle before iterating.
            waitOneFullHardwareCycle();
        }
    }
}


