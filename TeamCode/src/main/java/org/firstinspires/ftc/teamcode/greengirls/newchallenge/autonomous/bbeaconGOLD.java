package org.firstinspires.ftc.teamcode.greengirls.newchallenge.autonomous;

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
import com.qualcomm.robotcore.hardware.I2cAddr;


import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by Greatsword on 1/16/2017.
 */
@Autonomous(name="bbeaconGOLD",group="Auto")
    public class bbeaconGOLD extends LinearOpMode {


    protected final static double SERVO1_MIN_RANGE  = 0.10;
    protected final static double SERVO1_MID_RANGE  = 0.45;
    protected final static double SERVO1_MAX_RANGE  = 0.99;
    protected final static double SERVO2_MIN_RANGE  = 0.10;
    protected final static double SERVO2_MID_RANGE  = 0.45;
    protected final static double SERVO2_MAX_RANGE  = 0.99;
    boolean colourTrigger = false;
    boolean colourReady = false;
    final int threshold = 4;
    double rightSpeed = 0;
    double leftSpeed = 0;
    double errorLeft = 0;
    double errorRight = 0;
    int alpha1 = 0;
    int alpha2 = 0;
    double turnSpeed;
    double newHeading;
    double currentHeading;
    boolean reached;
    Orientation angles;
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
    public Servo servo3;
    public Servo servo4;
    public Servo servo5;
    public Servo servo6;
    public DeviceInterfaceModule cdim;
    public TouchSensor touchSensor;
    public GyroSensor gyroSensor;
    //this sensor must be on the left side
    public ColorSensor sensorRGB1;
    //this sensor must be on the right side
    public ColorSensor sensorRGB2;
    //this sensor detects the red v.s. blue of the light we press
    public ColorSensor sensorRGB3;
    public BNO055IMU imu;
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

        sensorRGB1.setI2cAddress(I2cAddr.create8bit(0x12));
        sensorRGB2.setI2cAddress(I2cAddr.create8bit(0x10));
        sensorRGB3.setI2cAddress(I2cAddr.create8bit(0x3c));

        sensorRGB1.enableLed(true);
        sensorRGB2.enableLed(true);
        sensorRGB3.enableLed(true);

        waitForStart();

        boolean finished = false;

        while (opModeIsActive()) {

            while (!finished) {

                //drive forward
                moveWithEncoder(1, -1, 4000);

                //turn right
                moveWithEncoder(1, 1, 1150);

                //get to line
                moveWithEncoder(1, -1, 6100);

                //turn right a little
               moveWithEncoder(1, 1, 1575);

                //move forward a little
                moveWithEncoder(1, -1, 1850);

                sleep(2000);
                //drive back to start
               /* moveWithEncoder(-1, 1, 1500);
                moveWithEncoder(-1, -1, 1150);
                moveWithEncoder(-1, 1, 5500);
                moveWithEncoder(-1, -1, 1400);
                moveWithEncoder(-1, 1, 3500);
                */
                finished = true;

                //follow line
                runWithColor(1500, 1);
                //hit the blue button
                checkColor("blue");
                //back up
                runWithColor(1500, -1);
                //turn left
                moveWithEncoder(-1, -1, 1000);
                //drive forward to beacon 2
                //moveWithEncoder(1, -1, 1000);
                //turn right to face beacon
                //moveWithEncoders(1, 1, 1000);
                //move forward a little
                //moveWithEncoder(1, -1, 550);
                //follow line
                //runWithColor(1500, 1);
                //hit the blue button
                //checkColor("blue");
                //back up
                //runWithColor(1500, -1)
                //turn right
                //moveWithEncoder(1, 1, 500);
                //drive to knock the ball off the center and park on center
                //moveWithEncoder(1, -1, 3000);


            }
        }



    }


 public void checkColor(String colour) throws InterruptedException
    {
        if (sensorRGB3.red() > sensorRGB3.blue())
        {
            if (colour == "red")
            {
                maxServo1();
                runWithEncoders();

                setLeftMotors(1);
                setRightMotors(-1);

                waitForEncodersReduxVersionTwoPointZero(850);

                resetEncoders();
                stopLeftMotors();
                stopRightMotors();

            }

            else
            {
                maxServo2();
                runWithEncoders();

                setLeftMotors(1);
                setRightMotors(1);

                waitForEncodersReduxVersionTwoPointZero(850);

                resetEncoders();
                stopLeftMotors();
                stopRightMotors();

            }
        }

        else
        {
            if (colour == "blue")
            {
                maxServo1();
                runWithEncoders();

                setLeftMotors(1);
                setRightMotors(1);

                waitForEncodersReduxVersionTwoPointZero(1500);

                resetEncoders();
                stopLeftMotors();
                stopRightMotors();

            }

            else
            {
                maxServo2();
                runWithEncoders();

                setLeftMotors(1);
                setRightMotors(1);

                waitForEncodersReduxVersionTwoPointZero(1500);

                resetEncoders();
                stopLeftMotors();
                stopRightMotors();

            }
        }
    }

    public void runWithColor(int encoderCount, double speed) throws InterruptedException {
        resetEncoders();
        reached = false;
        runWithEncoders();
        while (!reached) {
            alpha1 = sensorRGB1.alpha();
            alpha2 = sensorRGB2.alpha();
            telemetry.addData("alpha1", alpha1);
            telemetry.addData("alpha2", alpha2);
            errorLeft = (alpha1 - threshold)*.25;
            errorRight = (alpha2 - threshold)*.25;
            leftSpeed = speed - errorLeft;
            rightSpeed = speed - errorRight;
            setRightMotors(leftSpeed);
            setLeftMotors(-rightSpeed);
            if (Math.abs(leftBackMotor.getCurrentPosition()) > encoderCount) {

                reached = true;
             } else {
                telemetry.addData("curPos", Math.abs(leftBackMotor.getCurrentPosition()));
                telemetry.update();
            }
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

    public void waitForEncodersReduxVersionTwoPointZero(int encoderAmount) throws InterruptedException{
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



    public void setRightMotors(double power){
        rightFrontMotor.setPower(power);
        rightBackMotor.setPower(-power);
    }


    //stop right motors
    public void stopRightMotors(){
        rightFrontMotor.setPower(0);
        rightBackMotor.setPower(0);
    }

    //set power to left motors
    public void setLeftMotors(double power){
        leftFrontMotor.setPower(power);
        leftBackMotor.setPower(-power);
    }

    //stop left motors
    public void stopLeftMotors(){
        leftFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
    }
}
