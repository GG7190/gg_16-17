package org.firstinspires.ftc.teamcode.greengirls.newchallenge.autonomous;

import android.util.Log;
import org.firstinspires.ftc.teamcode.greengirls.newchallenge.GGLibrary;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by User on 12/3/2016.
 */

@Autonomous(name="ballKnock", group="ballKnock")
public class ballknock extends GGLibrary {
    //set state to zero
    int state = 0;
//    int count = 0;
 //   boolean reached = false;

    @Override
    public void loop() {




        switch (state) {
            case 0:
                resetEncoders();
                //drive and line up with first beacon
                runWithEncoders();

                setLeftMotors(.55);
                setRightMotors(.55);

                //looop


                waitForEncodersReduxVersionTwoPointZero(3000);
                //waitForEncodersReduxVersionTwoPointZero(1000);
                //waitForEncodersReduxVersionTwoPointZero(500);
                setLeftMotors(0);
                setRightMotors(0);
                resetEncoders();


                state++;
                Log.w("State 0 ", String.valueOf(state));
                break;

            case 1:
                //check to see if the encoders have reset
                if (have_drive_encoders_reset()) {
                    state++;
                }
                Log.w("State 1 ", String.valueOf(state));
                break;

            /*case 2:
                runWithEncoders();
                //turn to face beacon
                // //turnToHeading(90);
                setLeftMotors(-1);
                setRightMotors(-1);
                waitForEncodersReduxVersionTwoPointZero(450);
                resetEncoders();
                setRightMotors(0);
                setLeftMotors(0);
                state++;
                Log.w("State 2 ", String.valueOf(state));
                break;

            case 3:
                //check to see if the encoders have reset
                if (have_drive_encoders_reset()) {
                    state++;
                }
                Log.w("State 3 ", String.valueOf(state));
                break;

            case 4:
                //get to white line
                runWithEncoders();

                setLeftMotors(1);
                setRightMotors(1);

                waitForEncodersReduxVersionTwoPointZero(1500);

                resetEncoders();
                stopRightMotors();
                stopLeftMotors();

                state++;
                Log.w("State 4 ", String.valueOf(state));
                break;

            case 5:
                //check to see if the encoders have reset
                if (have_drive_encoders_reset()) {
                    state++;
                }
                Log.w("State 5 ", String.valueOf(state));
                break;

            case 6:

                //runWithColor(encoder count, true/false self-centering);
                runWithColor(100, true, 1);

                state++;
                Log.w("State 6 ", String.valueOf(state));
                break;
*/
        }

    }
}