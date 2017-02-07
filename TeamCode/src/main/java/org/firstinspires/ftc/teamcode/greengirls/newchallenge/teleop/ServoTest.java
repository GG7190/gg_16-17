package org.firstinspires.ftc.teamcode.greengirls.newchallenge.teleop;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.I2cAddr;


import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by Greatsword on 1/16/2017.
 */
@TeleOp(name="ServoTest",group="TeleOp")
public class ServoTest extends LinearOpMode {


    protected final static double SERVO1_MIN_RANGE = 0.10;
    protected final static double SERVO1_MID_RANGE = 0.45;
    protected final static double SERVO1_MAX_RANGE = 0.99;
    protected final static double SERVO2_MIN_RANGE = 0.10;
    protected final static double SERVO2_MID_RANGE = 0.45;
    protected final static double SERVO2_MAX_RANGE = 0.99;
    boolean reached;
    public DcMotorController rightWheelController;
    public DcMotor rightFrontMotor;
    public DcMotor rightBackMotor;
    public DcMotorController leftMotorController;
    public DcMotor leftFrontMotor;
    public DcMotor leftBackMotor;
    public DcMotorController attachmentMotorController1;
    public DcMotor motor1;
    public DcMotor motor2;
    public DcMotorController attachmentMotorController2;
    public DcMotor motor3;
    public DcMotor motor4;
    public ServoController servoController;
    public Servo servo1;
    public Servo servo2;
    public DeviceInterfaceModule cdim;
    public GyroSensor gyro;
    //this sensor must be on the left side
    public ColorSensor sensorRGB1;
    //this sensor must be on the right side
    public ColorSensor sensorRGB2;
    //this sensor detects the red v.s. blue of the light we press
    public ColorSensor sensorRGB3;
    public ColorSensor sensorRGB4;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();


    public void minServo1() {
        servo1.setPosition(SERVO1_MIN_RANGE);
    }

    public void midServo1() {
        servo1.setPosition(SERVO1_MID_RANGE);
    }

    public void maxServo1() {
        servo1.setPosition(SERVO1_MAX_RANGE);
    }

    public void minServo2() {
        servo2.setPosition(SERVO2_MIN_RANGE);
    }

    public void midServo2() {
        servo2.setPosition(SERVO2_MID_RANGE);
    }

    public void maxServo2() {
        servo2.setPosition(SERVO2_MAX_RANGE);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        cdim = hardwareMap.deviceInterfaceModule.get("dim");
        sensorRGB1 = hardwareMap.colorSensor.get("sensorColour1");
        sensorRGB2 = hardwareMap.colorSensor.get("sensorColour2");
        sensorRGB3 = hardwareMap.colorSensor.get("sensorColour3");
        sensorRGB4 = hardwareMap.colorSensor.get("sensorColour4");

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
        gyro = hardwareMap.gyroSensor.get("gyro");
        hardwareMap.logDevices();

        sensorRGB1.setI2cAddress(I2cAddr.create8bit(0x12));
        sensorRGB2.setI2cAddress(I2cAddr.create8bit(0x10));
        sensorRGB3.setI2cAddress(I2cAddr.create8bit(0x3c));
        sensorRGB3.setI2cAddress(I2cAddr.create8bit(0x14));

        sensorRGB1.enableLed(true);
        sensorRGB2.enableLed(true);
        sensorRGB3.enableLed(false);
        sensorRGB4.enableLed(false);


        servo1.scaleRange(0.0,1.0);
        waitForStart();


        while (opModeIsActive()) {



            //rSpeed = -gamepad1.right_stick_y;
            //lSpeed = gamepad1.left_stick_y;

            servo1.setPosition(gamepad1.right_stick_x);
            servo2.setPosition(gamepad1.right_stick_y);

            telemetry.addData("lefty", gamepad1.right_stick_x);
            telemetry.addData("righty", gamepad1.right_stick_y);
            telemetry.update();

            idle();
        }
    }

    public void runWithEncoders() throws InterruptedException {
        if (leftBackMotor != null) {
            leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(250);
            idle();
        }
    }

    public void resetEncoders() throws InterruptedException {
        leftBackMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        sleep(250);
        idle();
    }

    public void waitForEncodersReduxVersionTwoPointZero(int encoderAmount) throws InterruptedException {
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

    public void moveWithEncoder(double left, double right, int targetEncoder) throws InterruptedException {
        resetEncoders();
        runWithEncoders();
        setLeftMotors(left);
        setRightMotors(right);
        waitForEncodersReduxVersionTwoPointZero(targetEncoder);
        stopLeftMotors();
        stopRightMotors();
        resetEncoders();
    }


    public void setRightMotors(double power) {
        rightFrontMotor.setPower(power);
        rightBackMotor.setPower(-power);
    }


    //stop right motors
    public void stopRightMotors() {
        rightFrontMotor.setPower(0);
        rightBackMotor.setPower(0);
    }

    //set power to left motors
    public void setLeftMotors(double power) {
        leftFrontMotor.setPower(power);
        leftBackMotor.setPower(-power);
    }

    //stop left motors
    public void stopLeftMotors() {
        leftFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
    }

    public void buttonPushOut(){
        servo1.setPosition(SERVO1_MIN_RANGE);
        servo2.setPosition(SERVO2_MIN_RANGE);
    }
    public void buttonPushIn(){
        servo1.setPosition(SERVO1_MAX_RANGE);
        servo2.setPosition(SERVO2_MAX_RANGE);
    }


    //lift up
    public void liftUp() {
        motor1.setPower(-1);
        motor2.setPower(1);
    }

    //lift down
    public void liftDown() {
        motor1.setPower(1);
        motor2.setPower(-1);
    }

    //stop lift motor
    public void stopLift() {
        motor1.setPower(0);
        motor2.setPower(0);

    }

    //Start Funnel
    public void startFunnel(){
        motor3.setPower(1);
    }

    //Stop Funnel

    public void stopFunnel(){
        motor3.setPower(0);
    }

}
