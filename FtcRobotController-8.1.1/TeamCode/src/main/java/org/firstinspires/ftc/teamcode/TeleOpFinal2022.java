package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="TeleOpFinal2022", group = "LinearOpMode")
@Disabled
public class TeleOpFinal2022 extends LinearOpMode {

    private DcMotor frontLeftMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backRightMotor = null;
    private DcMotor wheelMotor = null;
    private DcMotor liftMotor = null;
    private Servo grabberServo = null;

    @Override
    public void runOpMode() throws InterruptedException {
        frontLeftMotor = hardwareMap.get(DcMotor.class,"Front_Left_Motor");
        frontRightMotor = hardwareMap.get(DcMotor.class,"Front_Right_Motor");
        backLeftMotor = hardwareMap.get(DcMotor.class,"Back_Left_Motor");
        backRightMotor = hardwareMap.get(DcMotor.class,"Back_Right_Motor");
        grabberServo = hardwareMap.get(Servo.class, "Grabber_Servo");
        wheelMotor = hardwareMap.get(DcMotor.class, "Wheel_Motor");
        liftMotor = hardwareMap.get(DcMotor.class, "Lift_Motor");


        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);



        waitForStart();
        while(opModeIsActive()) {

            telemetry.addData("Trigger Value",gamepad2.right_trigger);
            telemetry.update();
            if(gamepad2.right_trigger>0){
                grabberServo.setPosition(0);
            } else if(gamepad2.left_trigger>0){
                grabberServo.setPosition(0.4);
            } else{
                grabberServo.setPosition(1);
            }




//            if(gamepad2.right_trigger>0) {
//                grabberServo.setPosition(0);
//            } else {
//                grabberServo.setPosition(1);
//            }


            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            boolean z = gamepad1.a;
            boolean h = gamepad1.b;
            boolean w = gamepad2.dpad_up;
            boolean v = gamepad2.dpad_down;
            boolean m = gamepad2.a;

//            boolean akshat = gamepad2.b;
//            if (aksaht == true) {
//                liftMotor.setPower(1);
//            } else {
//                liftMotor.setPower(0.000001);
//            }


//            boolean shourya = gamepad1.a;
//            if (shourya == true) {
//                backLeftMotor.setPower(0.8);
//            } else if (shourya == false){
//                backLeftMotor.setPower(0);
//            } else {
//                backRightMotor.setPower(.68);
//            }


            if(w ==true){
                liftMotor.setPower(0.7);
            }
            else{
                liftMotor.setPower(0.05);
            }

            if(v ==true){
                liftMotor.setPower(-0.7);
            }
            else{
                liftMotor.setPower(0.05);
            }

            if(m == true){
                liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftMotor.setPower(0.8);
                liftMotor.setTargetPosition(2500);
                while (opModeIsActive() && liftMotor.isBusy()) {

                }
                liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                liftMotor.setPower(0.05);
            }



            if (z ==true){
                wheelMotor.setPower(0.7);
            }
            else{
                wheelMotor.setPower(0);
            }
            if(h==true){
                wheelMotor.setPower(-0.7);
            }
            else{
                wheelMotor.setPower(0);
            }

            double denomonator = Math.max(Math.abs(y)+Math.abs(x)+Math.abs(rx),1);

            frontLeftMotor.setPower((y+x+rx)/denomonator);
            frontRightMotor.setPower((y-x-rx)/denomonator);
            backLeftMotor.setPower((y-x+rx)/denomonator);
            backRightMotor.setPower((y+x-rx)/denomonator);
        }

    }
}
