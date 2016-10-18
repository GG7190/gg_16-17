package org.firstinspires.ftc.teamcode.greengirls.newchallenge.autonomous;

import android.util.Log;

import org.firstinspires.ftc.teamcode.greengirls.GGHardware;
import org.firstinspires.ftc.teamcode.greengirls.newchallenge.GGLibrary;

/**
 * Created by GreenGirls on 8/9/2016.
 */
public class Autonomous extends GGLibrary {

    //set state to zero
    int state = 0;
    int count = 0;

    @Override
    public void loop() {

        switch (state) {
            case 0:
                //
                runWithEncoders();

                //
                if (count<40) {
                    //WRONG CODE HAD TO GET TELEOP WORKING

                    resetEncoders();

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
                //
                count++;
                if (count < 40) {
                    //openDinoArms();
                    //openDeflector();
                } else if (count >= 40 && count < 200) {
                    // stopDeflector();
                } else if (count >= 200 && count < 300) {
                    // shootBalls();
                } else {
                    // stopShootBalls();
                    state++;
                    count = 0;
                }
                Log.w("COUNTER ", String.valueOf(count));
                Log.w("State 2 ", String.valueOf(state));
                break;
        }
    }
}
