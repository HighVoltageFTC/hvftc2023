package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="meet3Teleop2023", group = "LinearOpMode")
@Disabled
public class meet3Teleop2023 extends LinearOpMode {

    private DcMotor frontLeftMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backRightMotor = null;


    @Override
    public void runOpMode() throws InterruptedException {
        frontLeftMotor = hardwareMap.get(DcMotor.class,"Front_Left_Motor");
        frontRightMotor = hardwareMap.get(DcMotor.class,"Front_Right_Motor");
        backLeftMotor = hardwareMap.get(DcMotor.class,"Back_Left_Motor");
        backRightMotor = hardwareMap.get(DcMotor.class,"Back_Right_Motor");
//        grabberServo = hardwareMap.get(Servo.class, "Grabber_Servo");



        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);



        waitForStart();
        while(opModeIsActive()) {


            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double denomonator = Math.max(Math.abs(y)+Math.abs(x)+Math.abs(rx),1);

            frontLeftMotor.setPower((y+x+rx)/denomonator);
            frontRightMotor.setPower((y-x-rx)/denomonator);
            backLeftMotor.setPower((y-x+rx)/denomonator);
            backRightMotor.setPower((y+x-rx)/denomonator);
        }

    }
}
