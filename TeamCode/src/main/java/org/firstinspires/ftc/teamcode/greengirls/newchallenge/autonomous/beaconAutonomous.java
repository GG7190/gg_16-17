package org.firstinspires.ftc.teamcode.greengirls.newchallenge.autonomous;

import android.util.Log;

import org.firstinspires.ftc.teamcode.greengirls.newchallenge.GGLibrary;

/**
 * Created by User on 10/9/2016.
 */

public class beaconAutonomous extends GGLibrary {

    //set state to zero
    int state = 0;
    int count = 0;

    @Override
    public void loop() {

        switch (state) {
            case 0:
                //
                runWithEncoders();

                setLeftMotors(1);
                setRightMotors(1);

                //
                if (lEncoderCountReached(5850)) {

                    resetEncoders();
                    stopLeftMotors();
                    stopRightMotors();

                    state++;
                }
                Log.w("State 0 ", String.valueOf(state));
                break;

            case 1:

                //
                if (have_drive_encoders_reset()) {
                    state++;
                }
                Log.w("State 1 ", String.valueOf(state));
                break;

            case 2:

                runWithEncoders();

                setLeftMotors(0);
                setRightMotors(1);

                //turn to face beacon
                if(rEncoderCountReached(90)){

                    resetEncoders();
                    stopLeftMotors();
                    stopRightMotors();

                    state++;
                }

                Log.w("State 2 ", String.valueOf(state));
                break;

            case 3:
                //
                if (have_drive_encoders_reset()) {
                    state++;
                }
                Log.w("State 1 ", String.valueOf(state));
                break;

            case 4:

                runWithEncoders();

                setLeftMotors(1);
                setRightMotors(1);

                //get to white line
                if (rEncoderCountReached(100)){

                    resetEncoders();
                    stopRightMotors();
                    stopLeftMotors();

                    state++;
                }
                Log.w("State 3 ", String.valueOf(state));
                break;

            case 5:
                //
                if (have_drive_encoders_reset()) {
                    state++;
                }
                Log.w("State 1 ", String.valueOf(state));
                break;

            case 6:

                runWithEncoders();

                setLeftMotors(1);
                setRightMotors(1);

                //
        }
    }
}
