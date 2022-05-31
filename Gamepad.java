package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Gamepad")
public class Gamepad extends LinearOpMode {

    private DcMotorEx Right_Front;
    private DcMotorEx Right_Rear;
    private DcMotorEx Left_Front;
    private DcMotorEx Left_Rear;
    private DcMotorEx arm_motor1;
    private DcMotorEx arm_motor2;
    private DcMotorEx duckDeliveryMotor;
    private Servo left_servo;
    private Servo right_servo;
    private boolean isServo;

    @Override
    public void runOpMode() {
        double x;
        double y;
        double rx;
        double constant = 0.73; //speed reduction

        Right_Front = hardwareMap.get(DcMotorEx.class, "rightFront");
        Right_Rear = hardwareMap.get(DcMotorEx.class, "rightRear");
        Left_Front = hardwareMap.get(DcMotorEx.class, "leftFront");
        Left_Rear = hardwareMap.get(DcMotorEx.class, "leftRear");
        arm_motor1 = hardwareMap.get(DcMotorEx.class, "armMotor1");
        arm_motor2 = hardwareMap.get(DcMotorEx.class, "armMotor2");
        duckDeliveryMotor = hardwareMap.get(DcMotorEx.class, "duckDeliveryMotor");
        left_servo = hardwareMap.servo.get("LeftServo");
        right_servo = hardwareMap.servo.get("RightServo");

        Right_Front.setDirection(DcMotorSimple.Direction.REVERSE);
        Right_Rear.setDirection(DcMotorSimple.Direction.REVERSE);
 
        isServo = false;
        
        waitForStart();
        if (opModeIsActive()) {
            left_servo.setPosition(0.1);
            right_servo.setPosition(0.38);

            while (opModeIsActive()) {
                y = -gamepad1.left_stick_y;
                x = -gamepad1.left_stick_x * 1.1;
                rx = -gamepad1.right_stick_x;

                Left_Front.setPower((y + x + rx) * constant);
                Left_Rear.setPower((y - x + rx) * constant);
                Right_Front.setPower((y - x - rx) * constant);
                Right_Rear.setPower((y + x - rx) * constant);
                
                if(gamepad1.left_trigger == 1)
                {
                    arm_motor1.setPower(-0.55);
                    arm_motor2.setPower(-0.55);
                }
                else
                {
                    arm_motor1.setPower(0);
                    arm_motor2.setPower(0);
                }
                if(gamepad1.right_trigger == 1)
                {
                    arm_motor1.setPower(0.95);
                    arm_motor2.setPower(0.95);
                }
                else
                {
                    arm_motor1.setPower(0);
                    arm_motor2.setPower(0);
                }
                if(gamepad1.a)
                {
                    duckDeliveryMotor.setPower(1);
                }
                else
                {
                    duckDeliveryMotor.setPower(0);
                }
                if(gamepad1.y)
                {
                    if (isServo == false)
                    {
                        left_servo.setPosition(0.4);
                        right_servo.setPosition(0.08);
                        sleep(200);
                        isServo = true;
                    }
                    else
                    {
                        left_servo.setPosition(0.17);
                        right_servo.setPosition(0.31);
                        sleep(200);
                        isServo = false;
                    }
                }
            }//end while
        }//end opModeIsActive()
    }//end runOpMode()
}//end class
