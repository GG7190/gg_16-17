package org.firstinspires.ftc.teamcode.greengirls;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * Created by GreenGirls on 8/2/2016.
 */
public class GGHardware extends OpMode {

    //define Motors and MotorControllers
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
    public ColorSensor colorSensor;
    public DeviceInterfaceModule cdim;
    public TouchSensor touchSensor;
    public GyroSensor gyroSensor;

    static final int LED_CHANNEL = 5;

    @Override public void init() {

        //Map sensors below

        // get a reference to our ColorSensor object.

        // set the digital channel to output mode.
        // remember, the Adafruit sensor is actually two devices.
        // It's an I2C sensor and it's also an LED that can be turned on or off.
        // cdim.setDigitalChannelMode(LED_CHANNEL, DigitalChannelController.Mode.OUTPUT);
        //cdim = hardwareMap.deviceInterfaceModule.get("dim");
        //colorSensor = hardwareMap.colorSensor.get("color");
        //colorSensor.enableLed(false);
        //cdim.setDigitalChannelState(LED_CHANNEL, false);

        //map touch sensor
       // touchSensor = hardwareMap.touchSensor.get("touch");

        //map gyro
        //gyroSensor = hardwareMap.gyroSensor.get("gyro");

        //Map Hardware

        //Map hardware for Right motor controller
        rightWheelController = hardwareMap.dcMotorController.get("rightdrive");
        rightFrontMotor = hardwareMap.dcMotor.get("rfront");
        rightBackMotor = hardwareMap.dcMotor.get("rback");

        //Map hardware for Left motor controller
        leftMotorController = hardwareMap.dcMotorController.get("leftdrive");
        leftFrontMotor = hardwareMap.dcMotor.get("lfront");
        leftBackMotor = hardwareMap.dcMotor.get("lback");

        //Map hardware for attachment motor controller 1
        attachmentMotorController1 = hardwareMap.dcMotorController.get("attachment1");
        motor1 = hardwareMap.dcMotor.get("motor1");
        motor2 = hardwareMap.dcMotor.get("motor2");

        //Map hardware for attachment motor controller 2
        attachmentMotorController2 = hardwareMap.dcMotorController.get("attachment2");
        motor3 = hardwareMap.dcMotor.get("motor3");
       // motor4 = hardwareMap.dcMotor.get("motor4");

        //Map hardware for servo controller
        servoController = hardwareMap.servoController.get("servo");
        servo1 = hardwareMap.servo.get("servo1");
        servo2 = hardwareMap.servo.get("servo2");
      //  servo3 = hardwareMap.servo.get("servo3");
        //servo4 = hardwareMap.servo.get("servo4");
        //servo5 = hardwareMap.servo.get("servo5");
        //servo6 = hardwareMap.servo.get("servo6");

        hardwareMap.logDevices();

        // get a reference to our DeviceInterfaceModule object.
        // cdim = hardwareMap.deviceInterfaceModule.get("dim");

        // get a reference to our ColorSensor object.
        // colorSensor = hardwareMap.colorSensor.get("color");

    }

    @Override public void loop(){

    }
}
