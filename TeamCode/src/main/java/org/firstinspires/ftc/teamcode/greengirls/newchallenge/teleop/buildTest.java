package org.firstinspires.ftc.teamcode.greengirls.newchallenge.teleop;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

/**
 * Created by User on 10/7/2016.
 */
@TeleOp(name="buildTest", group="buildTest")
public class buildTest extends OpMode {
    protected final static double SERVO1_MIN_RANGE  = 0.00;
    protected final static double SERVO1_MID_RANGE  = 0.45;
    protected final static double SERVO1_MAX_RANGE  = 0.90;
    protected final static double SERVO2_MIN_RANGE  = 0.00;
    protected final static double SERVO2_MID_RANGE  = 0.45;
    protected final static double SERVO2_MAX_RANGE  = 0.90;
    public DcMotorController rightWheelController;
    public DcMotor rightFrontMotor;
    public DcMotor rightBackMotor;
    public DcMotorController leftMotorController;
    public DcMotor leftFrontMotor;
    public DcMotor leftBackMotor;
    public DcMotorController wheelController3;
    public DcMotor motor1;
    public DcMotor motor2;
    public DcMotorController attachmentController;
    public DcMotor motor3;
    public DcMotor motor4;
    public ServoController servoController;
    public Servo servo1;
    public Servo servo2;

    @Override public void init() {

        //Map hardware for right motor controller
        rightWheelController = hardwareMap.dcMotorController.get("rightdrive");
        rightFrontMotor = hardwareMap.dcMotor.get("rfront");
        rightBackMotor = hardwareMap.dcMotor.get("rback");

        //Map hardware for Left motor controller
        leftMotorController = hardwareMap.dcMotorController.get("leftdrive");
        leftFrontMotor = hardwareMap.dcMotor.get("lfront");
        leftBackMotor = hardwareMap.dcMotor.get("lback");

        //Map hardware for wheel motor controller 3
        //wheelController3 = hardwareMap.dcMotorController.get("wheel3");
        //motor1 = hardwareMap.dcMotor.get("motor1");
        //motor2 = hardwareMap.dcMotor.get("motor2");

        /*//Map hardware for attachment controller
        attachmentController = hardwareMap.dcMotorController.get("attachment")
                motor4 = hardwareMap.dcMotor.get("motor4"); */

       // servoController = hardwareMap.servoController.get("servo");
        //servo1 = hardwareMap.servo.get("servo1");
        //servo2 = hardwareMap.servo.get("servo2");

    }

    @Override public void loop(){
        setRightMotors(gamepad1.left_stick_y);
        setLeftMotors(-gamepad1.right_stick_y); }
        //Lift movements in teleop
  //      if (gamepad2.a){
     //       buttonPushOut();
       // }
        //if (gamepad2.b){
           // buttonPushIn();
        //}
       // if (gamepad2.y){
           // buttonPushOut();
    //    }
       // if (gamepad2.x){
          //  buttonPushIn();
//        }
    //}

  /*  public void motor1RunF() {
        motor1.setPower(1);
       // motor2.setPower(1);
    }

    public void motor1RunB() {
        motor1.setPower(-1);
        // motor2.setPower(1);
    }

    public void motor1stop() {
        motor1.setPower(0);
        //motor2.setPower(0);
    } */

    //Set positions for button pushers
 //   public void buttonPushOut(){
    //    servo1.setPosition(SERVO1_MIN_RANGE);
       // servo2.setPosition(SERVO1_MAX_RANGE);
   // }
    //public void buttonPushIn(){
       // servo1.setPosition(SERVO1_MAX_RANGE);
       // servo2.setPosition(SERVO1_MIN_RANGE);
    //}

    //public void buttonPushOut2(){
       // servo1.setPosition(SERVO2_MIN_RANGE);
        //servo2.setPosition(SERVO2_MAX_RANGE);
    //}
    //public void buttonPushIn2(){
       // servo1.setPosition(SERVO2_MAX_RANGE);
        //s/ervo2.setPosition(SERVO2_MIN_RANGE);
   // }

    public void setRightMotors(double power){
        rightFrontMotor.setPower(power);
        rightBackMotor.setPower(power);
        //motor2.setPower(-power);
    }


    //stop right motors
    public void stopRightMotors(){
        rightFrontMotor.setPower(0);
        rightBackMotor.setPower(0);
        //motor2.setPower(0);
    }

    //set power to left motors
    public void setLeftMotors(double power){
        leftFrontMotor.setPower(power);
        leftBackMotor.setPower(power);
        //motor1.setPower(-power);
    }

    //stop left motors
    public void stopLeftMotors(){
        leftFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        //motor1.setPower(0);
    }
}
