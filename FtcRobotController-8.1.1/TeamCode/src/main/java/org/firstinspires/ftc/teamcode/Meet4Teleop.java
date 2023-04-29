package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Meet4Teleop", group = "LinearOpMode")
public class Meet4Teleop extends LinearOpMode {

    private DcMotor frontLeftMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backRightMotor = null;
    private DcMotor liftMotorA = null;
    private DcMotor liftMotorB = null;
    private DcMotor grabberMotor = null;
//    private Servo grabberServoA = null;
//    private Servo grabberServoB = null;


    @Override
    public void runOpMode() throws InterruptedException {
        frontLeftMotor = hardwareMap.get(DcMotor.class, "Front_Left_Motor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "Front_Right_Motor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "Back_Left_Motor");
        backRightMotor = hardwareMap.get(DcMotor.class, "Back_Right_Motor");
        liftMotorA = hardwareMap.get(DcMotor.class, "Lift_Motor_A");
        liftMotorB = hardwareMap.get(DcMotor.class, "Lift_Motor_B");
        grabberMotor = hardwareMap.get(DcMotor.class, "Grabber_Motor");
//        grabberServoA = hardwareMap.get(Servo.class, "Grabber_Servo_A");
//        grabberServoB = hardwareMap.get(Servo.class, "Grabber_Servo_B");



        waitForStart();
        while (opModeIsActive()) {

            double rx = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double y = gamepad1.right_stick_x;
            boolean up = gamepad2.dpad_up;
            boolean down = gamepad2.dpad_down;
            boolean open = gamepad2.y;

//            telemetry.addData("Trigger Value",gamepad2.right_trigger);
//            telemetry.update();
//            if(gamepad2.right_trigger>0){
//                grabberServoA.setPosition(0);
//                grabberServoB.setPosition(1);
//            } else{
//                grabberServoA.setPosition(0.5);
//                grabberServoB.setPosition(0.5);
//            }
            if(gamepad2.right_trigger>0) {
                grabberMotor.setPower(0.8);
            }


            if(open == true) {
                grabberMotor.setPower(-0.2);
                sleep(50);
                grabberMotor.setPower(0);
            }

            if (up == true) {
                liftMotorA.setPower(1);
                liftMotorB.setPower(-1);
            } else {
                liftMotorA.setPower(0.05);
                liftMotorB.setPower(-0.05);
            }
            if (down == true) {
                liftMotorA.setPower(-1);
                liftMotorB.setPower(1);
            } else {
                liftMotorA.setPower(0.05);
                liftMotorB.setPower(-0.05);
            }

            double denomonator = Math.max(Math.abs(y)+Math.abs(x)+Math.abs(rx),1);

            frontLeftMotor.setPower((-y-x-rx)/denomonator);
            frontRightMotor.setPower((-y+x+rx)/denomonator);
            backLeftMotor.setPower((-y+x-rx)/denomonator);
            backRightMotor.setPower((-y-x+rx)/denomonator);
        }
    }
}


