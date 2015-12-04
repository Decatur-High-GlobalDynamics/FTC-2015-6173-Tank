package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


public class ArcadeTeleOp6173 extends OpMode {


    DcMotor driveRight;
    DcMotor driveLeft;
    DcMotor driveLeft2;
    DcMotor driveRight2;
    int i = 0;
    int x = 0;

    final static double LEFT_ARM_MIN = 0;
    final static double LEFT_ARM_MAX = .8;
    final static double RIGHT_ARM_MIN = 0;
    final static double RIGHT_ARM_MAX = .8;
    final static double ARM_DELAY = 0.250; //ms to wait for next arm press

    // position of the arm servo.
    double leftArmPosition;
    double rightArmPosition;

    double leftArmDelay = 0;
    double rightArmDelay = 0;

    // amount to change the arm servo position.
    double armDelta = 0.1;

    /*
    float currentLeftPower;
    float currentRightPower;
    float powerStep;
    long startTime;
    long currentTime;
    boolean lastGamePad1Y;
    Steerer currentSteerer;
    */

    Servo servoRight;
    Servo servoLeft;


    /**
     * Constructor
     */
    public ArcadeTeleOp6173() {

    }

    /*
     * Code to run when the op mode is first enabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    @Override
    public void init() {


		/*
         * Use the hardwareMap to get the dc motors and servos by name. Note
		 * that the names of the devices must match the names used when you
		 * configured your robot and created the configuration file.
		 */
		
		/*
		 *   
		 * We also assume that there are two servos "servo_1" and "servo_6"
		 *    "servo_1" controls the arm joint of the manipulator.
		 *    "servo_6" controls the claw joint of the manipulator.
		 */

        driveLeft = hardwareMap.dcMotor.get("left_front");
        driveLeft2 = hardwareMap.dcMotor.get("left_back");
        driveLeft.setDirection(DcMotor.Direction.REVERSE);
        driveLeft2.setDirection(DcMotor.Direction.REVERSE);

        driveRight = hardwareMap.dcMotor.get("right_front");
        driveRight2 = hardwareMap.dcMotor.get("right_front");
        driveRight.setDirection(DcMotor.Direction.FORWARD);
        driveRight2.setDirection(DcMotor.Direction.FORWARD);

        servoLeft = hardwareMap.servo.get("servo_left");
        servoLeft.setDirection(Servo.Direction.REVERSE);
        servoRight = hardwareMap.servo.get("servo_right");
        servoLeft.setDirection(Servo.Direction.FORWARD);

        leftArmPosition = LEFT_ARM_MIN; //(LEFT_ARM_MAX-LEFT_ARM_MIN)/2;
        rightArmPosition = RIGHT_ARM_MAX; //(RIGHT_ARM_MAX-RIGHT_ARM_MIN)/2;

        /*
        powerStep = 0.01f;
        currentLeftPower = 0.0f;
        currentRightPower = 0.0f;
        lastGamePad1Y = false;
        */


        //servoRightFront = hardwareMap.servo.get("right_front_servo");
        //servoRightRear = hardwareMap.servo.get("right_rear_servo");
        //servoLeftFront = hardwareMap.servo.get("left_front_servo");
        //servoLeftRear = hardwareMap.servo.get("left_rear_servo");

        //servoLeftFront.setPosition(.5);
        //servoLeftRear.setPosition(.5);
        //servoRightRear.setPosition(.5);
    }

    /*
     * This method will be called repeatedly in a loop
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
     */
    @Override
    public void loop() {


        float leftThrottle = gamepad1.left_stick_y;
        float rightThrottle = gamepad1.right_stick_y;


        // clip the right/left values so that the values never exceed +/- 1
        leftThrottle = Range.clip(leftThrottle, -1, 1);
        rightThrottle = Range.clip(rightThrottle, -1, 1);

        driveRight.setPower(rightThrottle);
        driveRight2.setPower(rightThrottle);
        driveLeft.setPower(leftThrottle);
        driveLeft2.setPower(leftThrottle);
        // scale the joystick value to make it easier to control
        // the robot more precisely at slower speeds.


        // clip the right/left values so that the values never exceed +/- 1

        // update the position of the arm.
//    if (gamepad1.a) {
//      // if the A button is pushed on gamepad1, increment the position of
//      // the arm servo.
//      // servoRightFront.setPosition(0.5);
//    } else {
//      //servoRightFront.setPosition(0.2);
//    }
//    if (gamepad1.right_bumper == true) {
//      servoRight.setPosition(.5);
//    } else if (gamepad1.left_bumper == true) {
//      servoLeft.setPosition(.5);
//    } else {
//      servoLeft.setPosition(1);
//      servoRight.setPosition(1);
//
//    }

        if (getRuntime() >= leftArmDelay) {
            if (gamepad1.left_bumper) {
                leftArmPosition -= armDelta;
                leftArmDelay = getRuntime() + ARM_DELAY;
            }

            if (gamepad1.left_trigger > 0.25) {
                leftArmPosition += armDelta;
                leftArmDelay = getRuntime() + ARM_DELAY;
            }
        }
        if (getRuntime() >= rightArmDelay) {
            if (gamepad1.right_bumper) {
                rightArmPosition += armDelta;
                rightArmDelay = getRuntime() + ARM_DELAY;
            }

            if (gamepad1.right_trigger > 0.25) {
                rightArmPosition -= armDelta;
                rightArmDelay = getRuntime() + ARM_DELAY;
            }
        }

        // clip the position values so that they never exceed their allowed range.
        leftArmPosition = Range.clip(leftArmPosition, LEFT_ARM_MIN, LEFT_ARM_MAX);
        rightArmPosition = Range.clip(rightArmPosition, RIGHT_ARM_MIN, RIGHT_ARM_MAX);

        // write position values to the wrist and claw servo
        servoLeft.setPosition(leftArmPosition);
        servoRight.setPosition(rightArmPosition);

/*
		 * Send telemetry data back to driver station. Note that if we are using
		 * a legacy NXT-compatible motor controller, then the getPower() method
		 * will return a null value. The legacy NXT-compatible motor controllers
		 * are currently write only.
		 */
        //telemetry.addData("Text", "*** Robot Data***");
        //telemetry.addData("arm", "arm:  " + String.format("%.2f", armPosition));
        //telemetry.addData("claw", "claw:  " + String.format("%.2f", clawPosition));
        telemetry.addData("left pwr", "left  pwr: " + String.format("%.2f", leftThrottle));
        telemetry.addData("right pwr", "right pwr: " + String.format("%.2f", rightThrottle));
        telemetry.addData("left arm", leftArmPosition);
        telemetry.addData("right arm", rightArmPosition);
    }

    /*
     * Code to run when the op mode is first disabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
     */
    @Override
    public void stop() {

    }
}

  /*
    * This method scales the joystick input so for low joystick values, the
    * scaled value is less than linear.  This is to make it easier to drive
    * the robot more precisely at slower speeds.
*/


// index cannot exceed size of array minus 1.


