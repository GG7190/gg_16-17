package org.firstinspires.ftc.teamcode.greengirls;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
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

        //map touch sensor\
       // touchSensor = hardwareMap.touchSensor.get("touch");

        //map gyro
        //gyroSensor = hardwareMap.gyroSensor.get("gyro");

        //map colour sensors
     cdim = hardwareMap.deviceInterfaceModule.get("dim");
        sensorRGB1 = hardwareMap.colorSensor.get("sensorColour1");
        sensorRGB2 = hardwareMap.colorSensor.get("sensorColour2");
        sensorRGB3 = hardwareMap.colorSensor.get("sensorColour3");
        //sensorRGB2.setI2cAddress(I2cAddr.create8bit(0x10));
        //sensorRGB3.setI2cAddress(I2cAddr.create8bit(0x12));
        //sensorRGB1.enableLed(true);
        //sensorRGB2.enableLed(true);

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
     //   servo5 = hardwareMap.servo.get("servo5");
       // servo6 = hardwareMap.servo.get("servo6");
//        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
   //     parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
      //  parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
   //     parameters.loggingEnabled      = true;
      //  parameters.loggingTag          = "IMU";
        //parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
       // imu = hardwareMap.get(BNO055IMU.class, "imu");

        hardwareMap.logDevices();

        // get a reference to our DeviceInterfaceModule object.
//         cdim = hardwareMap.deviceInterfaceModule.get("dim");

        // get a reference to our ColorSensor object.
       // colorSensor = hardwareMap.colorSensor.get("color");

    }

    @Override public void loop(){

    }
}
