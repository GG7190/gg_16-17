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
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by Greatsword on 1/16/2017.
 */
@TeleOp(name="PracticeTeleOp",group="TeleOp")
public class PracticeTeleOp extends LinearOpMode {


    protected final static double SERVO1_MIN_RANGE = 0.10;
    protected final static double SERVO1_MID_RANGE = 0.45;
    protected final static double SERVO1_MAX_RANGE = 0.90;
    protected final static double SERVO2_MIN_RANGE = 0.90;
    protected final static double SERVO2_MID_RANGE = 0.45;
    protected final static double SERVO2_MAX_RANGE = 0.10;
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

        boolean pusher1 = false;
        boolean pusher2 = false;

        ElapsedTime time = new ElapsedTime();

        waitForStart();

        time.reset();


        boolean funnel = false;



        while (opModeIsActive()) {


            double rSpeed = (gamepad1.right_stick_y + gamepad1.right_stick_x) * 1;
            double lSpeed = (gamepad1.right_stick_y - gamepad1.right_stick_x) * -1;


            //rSpeed = -gamepad1.right_stick_y;
            //lSpeed = gamepad1.left_stick_y;



            setRightMotors(rSpeed);
            setLeftMotors(lSpeed);


            if (gamepad2.y){
                liftUp();
            }
            else if (gamepad2.a){
                liftDown();
            }
            else {
                stopLift();
            }


            //Button Pusher movements in teleop
            if (gamepad2.dpad_right) {
                if (pusher1) {
                    minServo1();
                    pusher1 = false;
                } else {
                    maxServo1();
                    pusher1 = true;
                }
                sleep(250);
            }
            if (gamepad2.dpad_left){
                if (pusher2) {
                    minServo2();
                    pusher2 = false;
                } else {
                    maxServo2();
                    pusher2 = true;
                }
                sleep(250);
            }
            if (gamepad2.dpad_down) {
                minServo1();
                minServo2();
                pusher2 = false;
                pusher1 = false;
                sleep(250);
            }
            if (gamepad2.dpad_up) {
                maxServo2();
                maxServo1();
                pusher1 = true;
                pusher2 = true;
                sleep(250);
            }

            /*telemetry.addData("alpha1", sensorRGB1.alpha());
            telemetry.addData("alpha2", sensorRGB2.alpha());
            telemetry.addData("gamepad1.Y", gamepad1.right_stick_y);
            telemetry.addData("gamepad1.X", gamepad1.right_stick_x);
            telemetry.addData("rspeed", rSpeed);
            telemetry.addData("lspeedx", lSpeed);
            telemetry.addData("gyro",gyro.getHeading());*/


            // Funnel movement in teleop
            if (gamepad2.x) {
                if (funnel) {
                    stopFunnel();
                    funnel = false;
                } else {
                    startFunnel();
                    funnel = true;
                }
                sleep(250);
            }
            idle();

            telemetry.addData("pusher1", pusher1);
            telemetry.addData("pusher2", pusher2);
            telemetry.addData("funnel", funnel);
            telemetry.update();

        }
    }

    public




    void runWithEncoders() throws InterruptedException {
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
