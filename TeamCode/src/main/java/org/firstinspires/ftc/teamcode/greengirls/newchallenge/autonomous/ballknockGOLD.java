package org.firstinspires.ftc.teamcode.greengirls.newchallenge.autonomous;

/**
 * Created by Dell User on 1/14/2017.
 */
import android.util.Log;
import org.firstinspires.ftc.teamcode.greengirls.newchallenge.GGLibrary;

import com.qualcomm.hardware.adafruit.BNO055IMU;
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

@Autonomous(name="ballknockRED", group="Autonomous")
public class ballknockGOLD extends LinearOpMode{
    private DcMotorController rightWheelController;
    private DcMotor rightFrontMotor;
    private DcMotor rightBackMotor;
    private DcMotorController leftMotorController;
    private DcMotor leftFrontMotor;
    private DcMotor leftBackMotor;
    private DcMotorController attachmentMotorController1;
    private DcMotor motor1;
    private DcMotor motor2;
    private DcMotorController attachmentMotorController2;
    private DcMotor motor3;
    private ServoController servoController;
    private Servo servo1;
    private Servo servo2;
    private DeviceInterfaceModule cdim;
    //this sensor must be on the left side
    private ColorSensor sensorRGB1;
    //this sensor must be on the right side
    private ColorSensor sensorRGB2;
    //this sensor detects the red v.s. blue of the light we press
    private ColorSensor sensorRGB3;
    private boolean reached;

    static final int LED_CHANNEL = 5;


    @Override
    public void runOpMode() throws InterruptedException {
        cdim = hardwareMap.deviceInterfaceModule.get("dim");
        sensorRGB1 = hardwareMap.colorSensor.get("sensorColour1");
        sensorRGB2 = hardwareMap.colorSensor.get("sensorColour2");
        sensorRGB3 = hardwareMap.colorSensor.get("sensorColour3");
        rightWheelController = hardwareMap.dcMotorController.get("rightdrive");
        rightFrontMotor = hardwareMap.dcMotor.get("rfront");
        rightBackMotor = hardwareMap.dcMotor.get("rback");
        leftMotorController = hardwareMap.dcMotorController.get("leftdrive");
        leftFrontMotor = hardwareMap.dcMotor.get("lfront");
        leftBackMotor = hardwareMap.dcMotor.get("lback");
        attachmentMotorController1 = hardwareMap.dcMotorController.get("attachment1");
        motor1 = hardwareMap.dcMotor.get("motor1");
        motor2 = hardwareMap.dcMotor.get("motor2");
        attachmentMotorController2 = hardwareMap.dcMotorController.get("attachment2");
        motor3 = hardwareMap.dcMotor.get("motor3");
        servoController = hardwareMap.servoController.get("servo");
        servo1 = hardwareMap.servo.get("servo1");
        servo2 = hardwareMap.servo.get("servo2");
        hardwareMap.logDevices();
        waitForStart();

        boolean finished = false;

        while (opModeIsActive()) {
            while (!finished) {
                resetEncoders();
                runWithEncoders();
                setRightMotors(1);
                setLeftMotors(-1);
                waitForEncodersReduxVersionTwoPointZero(10000);
                setLeftMotors(0);
                setRightMotors(0);
                resetEncoders();
                sleep(200);
                runWithEncoders();
                setRightMotors(-.75);
                setLeftMotors(-.75);
                waitForEncodersReduxVersionTwoPointZero(950);
                setLeftMotors(0);
                setRightMotors(0);
                finished = true;
            }
            idle();
        }

    }
    private void runWithEncoders() {
        if (leftBackMotor != null) {
            leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    private void resetEncoders() {

        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void waitForEncodersReduxVersionTwoPointZero(int encoderAmount) throws InterruptedException {
        reached = false;
        while (!reached) {
            if (Math.abs(leftBackMotor.getCurrentPosition()) > encoderAmount) {
                reached = true;
            } else {
                telemetry.addData("curPos", Math.abs(leftBackMotor.getCurrentPosition()));
                telemetry.update();
            }
            idle();
        }
        reached = false;
    }
    private void setLeftMotors(double power){
        leftFrontMotor.setPower(power);
        leftBackMotor.setPower(-power);
    }
    private void setRightMotors(double power){
        rightFrontMotor.setPower(power);
        rightBackMotor.setPower(-power);
    }
}
