package org.firstinspires.ftc.teamcode.greengirls.newchallenge.autonomous;

import android.util.Log;
import org.firstinspires.ftc.teamcode.greengirls.newchallenge.autonomous.colourFollowTest;
import org.firstinspires.ftc.teamcode.greengirls.newchallenge.GGLibrary;

/**
 * Created by User on 10/16/2016.
 */

public class bbeaconAutonomous extends GGLibrary {

    //set state to zero
    int state = 0;
    int count = 0;

    @Override
    public void loop() {

        switch (state) {
            case 0:
                //drive and line up with first beacon
                runWithEncoders();

                setLeftMotors(1);
                setRightMotors(1);

                if (lEncoderCountReached(850)) {

                    resetEncoders();
                    stopLeftMotors();
                    stopRightMotors();

                    state++;
                }
                Log.w("State 0 ", String.valueOf(state));
                break;

            case 1:
                //check to see if the encoders have reset
                if (have_drive_encoders_reset()) {
                    state++;
                }
                Log.w("State 1 ", String.valueOf(state));
                break;

            case 2:
                //turn to face beacon
                turnToHeading("left", 90);
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

                if (rEncoderCountReached(100)){

                    resetEncoders();
                    stopRightMotors();
                    stopLeftMotors();

                    state++;
                }
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
                //use the color sensors to get to the beacon strait
                runWithColor(100, true, 1);

                state++;
                Log.w("State 6 ", String.valueOf(state));
                break;

            case 7:
                //check to see if the encoders have reset
                if (have_drive_encoders_reset()) {
                    state++;
                }
                Log.w("State 7 ", String.valueOf(state));
                break;

            case 8:
                checkColor("blue");
                state++;
                Log.w("State 8 ", String.valueOf(state));
                break;

            case 9:
                runWithColor(100, true, -1);
                state++;
                Log.w("State 9 ", String.valueOf(state));
                break;

            case 10:
                //check to see if the encoders have reset
                if (have_drive_encoders_reset()) {
                    state++;
                }
                Log.w("State 10 ", String.valueOf(state));
                break;

            case 11:
                //turn right
                turnToHeading("right", 120);
                state++;
                Log.w("State 11 ", String.valueOf(state));
                break;

            case 12:
                //check to see if the encoders have reset
                if (have_drive_encoders_reset()) {
                    state++;
                }
                Log.w("State 12 ", String.valueOf(state));
                break;

            case 13:
                //drive forward until lined up with the second beacon
                runWithEncoders();

                setRightMotors(1);
                setLeftMotors(1);

                if (rEncoderCountReached(300)){

                    resetEncoders();
                    stopRightMotors();
                    stopLeftMotors();

                    state++;
                }
                Log.w("State 13 ", String.valueOf(state));
                break;

            case 14:
                //check to see if the encoders have reset
                if (have_drive_encoders_reset()) {
                    state++;
                }
                Log.w("State 14 ", String.valueOf(state));
                break;

            case 15:
                //turn left to face the beacon
                turnToHeading("left", 90);
                state++;
                Log.w("State 15 ", String.valueOf(state));
                break;

            case 16:
                //get to white line
                runWithEncoders();

                setLeftMotors(1);
                setRightMotors(1);

                if (rEncoderCountReached(100)){

                    resetEncoders();
                    stopRightMotors();
                    stopLeftMotors();

                    state++;
                }
                Log.w("State 16 ", String.valueOf(state));
                break;

            case 17:
                //check to see if the encoders have reset
                if (have_drive_encoders_reset()) {
                    state++;
                }
                Log.w("State 17 ", String.valueOf(state));
                break;

            case 18:
                //use the color sensors to get to the beacon strait
                runWithColor(100, true, 1);

                state++;
                Log.w("State 18 ", String.valueOf(state));
                break;

            case 19:
                //check to see if the encoders have reset
                if (have_drive_encoders_reset()) {
                    state++;
                }
                Log.w("State 19 ", String.valueOf(state));
                break;

            case 20:
                checkColor("blue");
                state++;
                Log.w("State 20 ", String.valueOf(state));
                break;

            case 21:
                //back up until off white line using the color sensors
                runWithColor(100, true, -1);

                state++;
                Log.w("State 21 ", String.valueOf(state));
                break;

            case 22:
                //check to see if the encoders have reset
                if (have_drive_encoders_reset()) {
                    state++;
                }
                Log.w("State 22 ", String.valueOf(state));
                break;

            case 23:
                //turn left to face the center structure
                turnToHeading("left", 45);
                state++;
                Log.w("State 23 ", String.valueOf(state));
                break;

            case 24:
                //check to see if the encoders have reset
                if (have_drive_encoders_reset()) {
                    state++;
                }
                Log.w("State 24 ", String.valueOf(state));
                break;

            case 25:
                //drive to the center and push off the cap ball
                runWithEncoders();

                setLeftMotors(1);
                setRightMotors(1);

                if (lEncoderCountReached(125)){

                    resetEncoders();
                    stopRightMotors();
                    stopLeftMotors();

                    state++;
                }
                Log.w("State 25 ", String.valueOf(state));
                break;

            case 26:
                //check to see if the encoders have reset
                if (have_drive_encoders_reset()) {
                    state++;
                }
                Log.w("State 26 ", String.valueOf(state));
                break;
        }
    }
}

