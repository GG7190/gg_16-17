package org.firstinspires.ftc.teamcode.greengirls.newchallenge.autonomous;

import org.firstinspires.ftc.teamcode.greengirls.newchallenge.GGLibrary;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * Created by User on 8/19/2016.
 */

public class sensorTestAuto extends OpMode {

    public DcMotorController leftMotorController;
    public DcMotor leftFrontMotor;
    public DcMotor leftBackMotor;
    public TouchSensor touchSensor;
    public GyroSensor gyroSensor;

    @Override
    public void init() {
        //map touch sensor
        touchSensor = hardwareMap.touchSensor.get("touch");

        //map gyro
        gyroSensor = hardwareMap.gyroSensor.get("gyro");

        //Map Hardware
        leftMotorController = hardwareMap.dcMotorController.get("leftdrive");
        leftFrontMotor = hardwareMap.dcMotor.get("lfront");
        leftBackMotor = hardwareMap.dcMotor.get("lback");

        hardwareMap.logDevices();
    }

    //set power to left motors
    public void setLeftMotors(double power) {
        leftFrontMotor.setPower(.9 * power);
        leftBackMotor.setPower(power);
    }

    //stop left motors
    public void stopLeftMotors() {
        leftFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
    }

    public void loop() {
        if (touchSensor.isPressed()) {
            setLeftMotors(0.0);
        } else {
            setLeftMotors(0.5);
        }

    }

    public boolean isTouchSensorPressed() {
        return touchSensor.isPressed();
    }
}
