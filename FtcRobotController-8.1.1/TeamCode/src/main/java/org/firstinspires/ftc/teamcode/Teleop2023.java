package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Teleop2023", group = "LinearOpMode")

public class Teleop2023 extends LinearOpMode {

    private DcMotor frontLeftMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backRightMotor = null;
    private DcMotor liftMotor = null;
//    private Servo grabberServo = null;
    private DcMotor grabberMotor = null;


    @Override
    public void runOpMode() throws InterruptedException {
        frontLeftMotor = hardwareMap.get(DcMotor.class,"Front_Left_Motor");
        frontRightMotor = hardwareMap.get(DcMotor.class,"Front_Right_Motor");
        backLeftMotor = hardwareMap.get(DcMotor.class,"Back_Left_Motor");
        backRightMotor = hardwareMap.get(DcMotor.class,"Back_Right_Motor");
//        grabberServo = hardwareMap.get(Servo.class, "Grabber_Servo");
        liftMotor = hardwareMap.get(DcMotor.class, "Lift_Motor");
        grabberMotor = hardwareMap.get(DcMotor.class, "Grabber_Motor");


        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);



        waitForStart();
        while(opModeIsActive()) {

//            telemetry.addData("Trigger Value",gamepad2.right_trigger);
//            telemetry.update();
//            if(gamepad2.right_trigger>0){
//                grabberServo.setPosition(1);
//            } else{
//                grabberServo.setPosition(0.75);
//            }

//            telemetry.addData("Trigger Value",gamepad2.right_trigger);
//            telemetry.update();
//            if(gamepad2.right_trigger>0){
//                grabberServo.setPosition(0);
//            } else {
//                grabberServo.setPosition(1);
//            }






//            if(gamepad2.right_trigger>0) {
//                grabberServo.setPosition(0);
//            } else {
//                grabberServo.setPosition(1);
//            }

            if (gamepad2.right_trigger>0) {
                grabberMotor.setPower(-0.5);
            }

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            boolean z = gamepad1.a;
            boolean h = gamepad1.b;
            boolean w = gamepad2.dpad_up;
            boolean v = gamepad2.dpad_down;
            boolean a = gamepad2.a;
            boolean m = gamepad2.y;

            if(m == true) {
                grabberMotor.setPower(0.2);
                sleep(50);
                grabberMotor.setPower(0);
            }

            if(w ==true){
                liftMotor.setPower(1);
            }
            else{
                liftMotor.setPower(0.05);
            }

            if(v ==true){
                liftMotor.setPower(-1);
            }
            else{
                liftMotor.setPower(0.05);
            }


            if(a == true){
                liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftMotor.setPower(1);
                liftMotor.setTargetPosition(3200);
                while (opModeIsActive() && liftMotor.isBusy()) {

                }
                liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                liftMotor.setPower(0.05);
            }



            double denomonator = Math.max(Math.abs(y)+Math.abs(x)+Math.abs(rx),1);

            frontLeftMotor.setPower((y+x+rx)/denomonator);
            frontRightMotor.setPower((y-x-rx)/denomonator);
            backLeftMotor.setPower((y-x+rx)/denomonator);
            backRightMotor.setPower((y+x-rx)/denomonator);
        }

    }
}
