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
    protected final static double SERVO2_MIN_RANGE  = 0.99;
    protected final static double SERVO2_MID_RANGE  = 0.45;
    protected final static double SERVO2_MAX_RANGE  = 0.10;
    double errorWeight = .030;
    final int threshold = 1;
    double rightSpeed = 0;
    double leftSpeed = 0;
    double errorLeft = 0;
    double errorRight = 0;
    int alpha1 = 0;
    int alpha2 = 0;
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
    public ServoController servoController;
    public Servo servo1;
    public Servo servo2;
    public DeviceInterfaceModule cdim;
    public GyroSensor gyro;
    //this sensor must be on the left side
    public ColorSensor sensorRGB1;
    //this sensor must be on the right side
    public ColorSensor sensorRGB2;
    //this sensor detects the red v.s. blue of the light we press on the left side
    public ColorSensor sensorRGB3;
    //right side beacon sensor
    public ColorSensor sensorRGB4;



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

    public void maxServo2() { servo2.setPosition(SERVO2_MAX_RANGE); }

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


        gyro.calibrate(); //calibrate the gyro.
        while (gyro.isCalibrating()) { // waits until gyro is calibrated
            telemetry.addData("Gyro", "Gyro is calibrating... wait..."); // tell the user that the gyro is calibrating, and that they should (((WAIT)))
            telemetry.update(); // push the message
            idle(); // update hardware -- formerly waitOneFullHardwareCycle();
        }
        telemetry.addData("Gyro", "Done calibrating"); // tell the user that the gyro is done calibrating :)
        telemetry.addData("Colour Sensors", "Checking..."); //tell the user we are checking the colour sensors
        telemetry.update(); // push message


        telemetry.addData("Gyro", "Done calibrating"); //we need to add this message again because if we update telemetry without it added twice it gets removed

        if (sensorRGB1.alpha() == 255) { // check if the first sensor is reading 255 -- they tend to read 255 on all channels for an unknown reason.
            telemetry.addData("Colour Sensor 1", "FAIL! Reading 255, check sensor!");
        } else {
            telemetry.addData("Color Sensor 1", "Reading OK");
        }

        if (sensorRGB2.alpha() == 255) {
            telemetry.addData("Colour Sensor 2", "FAIL! Reading 255, check sensor!");
        } else {
            telemetry.addData("Color Sensor 2", "Reading OK");
        }

        if (sensorRGB3.alpha() == 255) {
            telemetry.addData("Colour Sensor 3", "FAIL! Reading 255, check sensor!");
        } else {
            telemetry.addData("Color Sensor 3", "Reading OK");
        }

        if (sensorRGB4.alpha() == 255) {
            telemetry.addData("Colour Sensor 4", "FAIL! Reading 255, check sensor!");
        } else {
            telemetry.addData("Color Sensor 4", "Reading OK");
        }



        telemetry.update();

        waitForStart();

        boolean finished = false;


        while (opModeIsActive()) {



            while (!finished) {



                //move forward
                moveToEncoder(.30, -.30, 1500);

                //turn left towards the beacon
                turnToHeading(28);

                //drive forward to reach the white line
                reachLine(30);

                //move forward a bit
                moveToEncoder(.30, -.30, 475);

                //turn to face the beacon
                turnToHeading(72);

                //back up a tad to get centered on the line
                followLineToEncoder(.30, -1, 800);


                //follow line to the beacon
                followLineToBeacon(.30);

                //detect the colour of the left side of the beacon
                String color = detectColour();

                //back up from the beacon
                followLineFromBeacon(.30);

                //extend the pushers
                extendPusher("blue", color);

                double speed = .30;

                reached = false;
                while (!reached) {
                    alpha1 = sensorRGB1.alpha();
                    alpha2 = sensorRGB2.alpha();
                    telemetry.addData("alpha1", alpha1);
                    telemetry.addData("alpha2", alpha2);
                    errorLeft = (alpha1 - threshold) * errorWeight;
                    errorRight = (alpha2 - threshold) * errorWeight;
                    if (errorLeft < 0) {
                        errorLeft = 0;
                    }
                    if (errorRight < 0) {
                        errorRight = 0;
                    }
                    leftSpeed = speed - errorLeft;
                    rightSpeed = speed - errorRight;
                    setRightMotors(-rightSpeed);
                    setLeftMotors(leftSpeed);
                    if (color == "blue") {
                        if (sensorRGB4.blue() >= 2) {
                            reached = true;
                            setRightMotors(0);
                            setLeftMotors(0);
                        }
                    } else {
                        if (sensorRGB3.blue() >= 2) {
                            reached = true;
                            setRightMotors(0);
                            setLeftMotors(0);
                        }
                    }
                    telemetry.addData("red3", sensorRGB3.red());
                    telemetry.addData("blue3", sensorRGB3.blue());
                    telemetry.addData("red4", sensorRGB4.red());
                    telemetry.addData("blue4", sensorRGB4.blue());
                    telemetry.update();
                    idle();
                }

                minServo1();
                minServo2();
                resetEncoders();
                idle();
                reached = false;
                runWithEncoders();
                while (!reached) {
                    alpha1 = sensorRGB1.alpha();
                    alpha2 = sensorRGB2.alpha();
                    telemetry.addData("alpha1", alpha1);
                    telemetry.addData("alpha2", alpha2);
                    errorLeft = (alpha1 - threshold)*errorWeight;
                    errorRight = (alpha2 - threshold)*errorWeight;
                    if (errorLeft<0) {errorLeft=0;}
                    if (errorRight<0) {errorRight=0;}
                    leftSpeed = speed - errorLeft;
                    rightSpeed = speed - errorRight;
                    setRightMotors(rightSpeed);
                    setLeftMotors(-leftSpeed);
                    if (Math.abs(leftBackMotor.getCurrentPosition()) > 1000) {
                        reached = true;
                    } else {
                        telemetry.addData("curPos", Math.abs(leftBackMotor.getCurrentPosition()));
                        telemetry.update();
                    }
                    idle();
                }




                reached = false;
                while (!reached) {
                    if (gyro.getHeading() <= 15 ) {
                        reached = true;
                        setRightMotors(0);
                        setLeftMotors(0);
                    }
                    double error;
                    if (gyro.getHeading() < 300) {
                        error = (gyro.getHeading()) * .02;
                    } else {
                        error = .45;
                    }
                    setRightMotors(-error);
                    setLeftMotors(-error);
                    telemetry.addData("heading", gyro.getHeading());
                    telemetry.addData("error", error);
                    telemetry.update();
                    idle();
                }

                reached = false;
                setRightMotors(-.35);
                setLeftMotors(.35);
                while (!reached) {
                    if (gyro.getHeading() < 5 || gyro.getHeading() > 5) {
                        leftSpeed = .25+((5-gyro.getHeading())*.030);
                        rightSpeed = .25-((5-gyro.getHeading())*.030);
                        setLeftMotors(leftSpeed);
                        setRightMotors(-rightSpeed);
                    } else {
                        setRightMotors(-.45);
                        setLeftMotors(.45);
                    }
                    if (sensorRGB2.alpha() > 0 || sensorRGB1.alpha() > 0) {
                        reached = true;
                        setRightMotors(0);
                        setLeftMotors(0);
                    }
                    telemetry.addData("alpha", sensorRGB2.alpha());
                    telemetry.addData("heading", gyro.getHeading());
                    telemetry.addData("leftSpeed", leftSpeed);
                    telemetry.addData("rightSpeed", rightSpeed);
                    telemetry.update();
                    idle();
                }

                reached = false;
                while (!reached) {
                    if (gyro.getHeading() >= 70) {
                        reached = true;
                        finished = true;
                        setRightMotors(0);
                        setLeftMotors(0);
                    }double error;
                    if (gyro.getHeading() < 300) {
                        error = (70 - gyro.getHeading()) * .02;
                    } else {
                        error = .45;
                    }
                    setRightMotors(error*1.2);
                    setLeftMotors(error);
                    telemetry.addData("heading", gyro.getHeading());
                    telemetry.addData("error", error);
                    telemetry.update();
                    idle();
                }

                reached = false;

                while (!reached) {
                    alpha1 = sensorRGB1.alpha();
                    alpha2 = sensorRGB2.alpha();
                    telemetry.addData("alpha1", alpha1);
                    telemetry.addData("alpha2", alpha2);
                    errorLeft = (alpha1 - threshold)*.05;
                    errorRight = (alpha2 - threshold)*.05;
                    if (errorLeft<0) {errorLeft=0;}
                    if (errorRight<0) {errorRight=0;}
                    leftSpeed = speed - errorLeft;
                    rightSpeed = speed - errorRight;
                    setRightMotors(-rightSpeed);
                    setLeftMotors(leftSpeed);
                    if (sensorRGB3.red() >= 2 || sensorRGB3.blue() >= 2 ) {
                        reached = true;
                        finished = true;
                        setRightMotors(0);
                        setLeftMotors(0);
                    } else {
                        telemetry.addData("red", sensorRGB3.red());
                        telemetry.addData("blue", sensorRGB3.blue());
                        telemetry.update();
                    }
                    idle();
                }
                if (sensorRGB3.blue() > 1) {
                    color = "blue";
                } else {
                    color = "red";
                }
                reached = false;
                while (!reached) {
                    alpha1 = sensorRGB1.alpha();
                    alpha2 = sensorRGB2.alpha();
                    telemetry.addData("alpha1", alpha1);
                    telemetry.addData("alpha2", alpha2);
                    errorLeft = (alpha1 - threshold) * .05;
                    errorRight = (alpha2 - threshold) * .05;
                    if (errorLeft < 0) {
                        errorLeft = 0;
                    }
                    if (errorRight < 0) {
                        errorRight = 0;
                    }
                    leftSpeed = speed - errorLeft;
                    rightSpeed = speed - errorRight;
                    setRightMotors(rightSpeed);
                    setLeftMotors(-leftSpeed);
                    if (sensorRGB3.red() == 0 && sensorRGB3.blue() == 0) {
                        reached = true;
                        setRightMotors(0);
                        setLeftMotors(0);
                    } else {
                        telemetry.addData("red", sensorRGB3.red());
                        telemetry.addData("blue", sensorRGB3.blue());
                        telemetry.update();
                    }
                    idle();
                }
                if (color == "blue") {
                    maxServo2();
                    minServo1();
                } else {
                    maxServo1();
                    minServo2();
                }
                /*
                reached = false;
                while (!reached) {
                    alpha1 = sensorRGB1.alpha();
                    alpha2 = sensorRGB2.alpha();
                    telemetry.addData("alpha1", alpha1);
                    telemetry.addData("alpha2", alpha2);
                    errorLeft = (alpha1 - threshold) * .20;
                    errorRight = (alpha2 - threshold) * .20;
                    if (errorLeft < 0) {
                        errorLeft = 0;
                    }
                    if (errorRight < 0) {
                        errorRight = 0;
                    }
                    leftSpeed = speed - errorLeft;
                    rightSpeed = speed - errorRight;
                    setRightMotors(-rightSpeed);
                    setLeftMotors(leftSpeed);
                    if ((sensorRGB3.red() + sensorRGB3.blue()) >= 2) {
                        reached = true;
                        setRightMotors(0);
                        setLeftMotors(0);
                    } else {
                        telemetry.addData("red", sensorRGB3.red());
                        telemetry.addData("blue", sensorRGB3.blue());
                        telemetry.update();
                    }
                    idle();
                }

                reached = false;
                setLeftMotors(-.25);
                setRightMotors(.25);
                while (!reached) {
                    if (sensorRGB3.red() < 1 && sensorRGB1.blue() < 1) {
                        reached = true;
                        setRightMotors(0);
                        setLeftMotors(0);
                    } else {
                        telemetry.addData("red", sensorRGB3.red());
                        telemetry.addData("blue", sensorRGB3.blue());
                        telemetry.update();
                    }
                    idle();
                }
                */
            }
        }
    }

    public void extendPusher(String targetColour,  String detectedColour) {
        if (targetColour == detectedColour) {
            maxServo1();
            minServo2();
        } else {
            maxServo2();
            minServo1();
        }
    }

    public void followLineFromBeacon(double speed) throws InterruptedException{
        reached = false;
        while (!reached) {
            alpha1 = sensorRGB1.alpha();
            alpha2 = sensorRGB2.alpha();
            telemetry.addData("alpha1", alpha1);
            telemetry.addData("alpha2", alpha2);
            errorLeft = (alpha1 - threshold) * errorWeight;
            errorRight = (alpha2 - threshold) * errorWeight;
            if (errorLeft < 0) {
                errorLeft = 0;
            }
            if (errorRight < 0) {
                errorRight = 0;
            }
            leftSpeed = speed - errorLeft;
            rightSpeed = speed - errorRight;
            setRightMotors(rightSpeed);
            setLeftMotors(-leftSpeed);
            if (sensorRGB3.red() < 2 && sensorRGB3.blue() < 1 && sensorRGB4.blue() < 1 && sensorRGB4.red() < 1) {
                reached = true;
                setRightMotors(0);
                setLeftMotors(0);
            } else {
                telemetry.addData("red", sensorRGB3.red());
                telemetry.addData("blue", sensorRGB3.blue());
                telemetry.update();
            }
            idle();
        }

    }

    public void followLineToBeacon(double speed) throws InterruptedException {
        reached = false;
        while (!reached) {
            alpha1 = sensorRGB1.alpha();
            alpha2 = sensorRGB2.alpha();
            telemetry.addData("alpha1", alpha1);
            telemetry.addData("alpha2", alpha2);
            errorLeft = (alpha1 - threshold)*errorWeight;
            errorRight = (alpha2 - threshold)*errorWeight;
            if (errorLeft<0) {errorLeft=0;}
            if (errorRight<0) {errorRight=0;}
            leftSpeed = speed - errorLeft;
            rightSpeed = speed - errorRight;
            setRightMotors(-rightSpeed);
            setLeftMotors(leftSpeed);
            if (sensorRGB3.red() >= 2 || sensorRGB3.blue() >= 2 || sensorRGB4.red() >=2 || sensorRGB4.blue() >=2 ) {
                reached = true;
                setRightMotors(0);
                setLeftMotors(0);
            } else {
                telemetry.addData("red", sensorRGB3.red());
                telemetry.addData("blue", sensorRGB3.blue());
                telemetry.update();
            }
            idle();
        }
    }

    private String detectColour() {
        if (sensorRGB3.blue() > sensorRGB4.blue()) {
            return "blue";
        } else {
            return "red";
        }
    }

    public void followLineToEncoder(double speed, double scale , int encoder) throws InterruptedException {
        resetEncoders();
        idle();
        reached = false;

        runWithEncoders();
        while (!reached) {
            alpha1 = sensorRGB1.alpha();
            alpha2 = sensorRGB2.alpha();
            telemetry.addData("alpha1", alpha1);
            telemetry.addData("alpha2", alpha2);
            errorLeft = (alpha1 - threshold)*errorWeight;
            errorRight = (alpha2 - threshold)*errorWeight;
            if (errorLeft<0) {errorLeft=0;}
            if (errorRight<0) {errorRight=0;}
            leftSpeed = speed - errorLeft;
            rightSpeed = speed - errorRight;
            setRightMotors(-rightSpeed*scale);
            setLeftMotors(leftSpeed*scale);
            if (Math.abs(leftBackMotor.getCurrentPosition()) > encoder) {
                reached = true;
            } else {
                telemetry.addData("curPos", Math.abs(leftBackMotor.getCurrentPosition()));
                telemetry.update();
            }
            idle();
        }

    }


    private void reachLine(int heading) throws InterruptedException {
        reached = false;
        setRightMotors(-.45);
        setLeftMotors(.45);
        while (!reached) {
            telemetry.addData("heading", gyro.getHeading());
            telemetry.update();
            if (gyro.getHeading() < heading || gyro.getHeading() > heading) {
                leftSpeed = .25 + ((heading - gyro.getHeading()) * .030);
                rightSpeed = .25 - ((heading - gyro.getHeading()) * .030);
                setLeftMotors(leftSpeed);
                setRightMotors(-rightSpeed);

            } else {
                setRightMotors(-.45);
                setLeftMotors(.45);
            }
            if (sensorRGB1.alpha() > 1 || sensorRGB2.alpha() > 1) {
                reached = true;
                setRightMotors(0);
                setLeftMotors(0);
            }
            idle();
        }
    }

    public void moveToEncoder(double left, double right, int encoder) throws InterruptedException {
        //drive forward
        resetEncoders();
        sleep(100);
        runWithEncoders();
        sleep(100);
        setRightMotors(right);
        setLeftMotors(left);
        reached = false;
        while (!reached) {
            if (Math.abs(leftBackMotor.getCurrentPosition()) > encoder) {
                reached = true;
                setRightMotors(0);
                setLeftMotors(0);
            }
            idle();
        }
    }

    public void turnToHeading(int heading) throws InterruptedException {
        reached = false;
        while (!reached) {
            if (gyro.getHeading() >= heading && gyro.getHeading() <= 300) {
                reached = true;
                setRightMotors(0);
                setLeftMotors(0);
            }
            double error;
            if (gyro.getHeading() < 300) {
                error = (heading -  gyro.getHeading()) * .03;
            } else {
                error = .45;
            }
            setRightMotors(error);
            setLeftMotors(error);
            telemetry.addData("heading", gyro.getHeading());
            telemetry.addData("error", error);
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


    /**
     *  Turns on the right motors to the level of the input
     *
     * @param power
     */
    public void setRightMotors(double power){
        rightFrontMotor.setPower(power);
        rightBackMotor.setPower(-power);
    }

    /**
     *  Sets right motor power to 0, thus stopping the robot and the motors
     */
    //stop right motors
    public void stopRightMotors(){
        rightFrontMotor.setPower(0);
        rightBackMotor.setPower(0);
    }

    /**
     *  Turns the left  motors on to the level of input
     *
     * @param power
     */
    //set power to left motors
    public void setLeftMotors(double power){
        leftFrontMotor.setPower(power);
        leftBackMotor.setPower(-power);
    }

    /**
     *  Set left motors power to 0, thus stopping the robot and the motors
     */
    public void stopLeftMotors(){
        leftFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
    }
}
