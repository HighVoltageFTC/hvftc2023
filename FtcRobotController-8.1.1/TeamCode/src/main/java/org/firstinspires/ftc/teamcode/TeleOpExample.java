package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp(name = "Example", group = "Linear OpMode")
@Disabled
public class TeleOpExample extends LinearOpMode  {

    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private Servo servo = null;

    @Override
    public void runOpMode(){
        leftDrive = hardwareMap.get(DcMotor.class, "Left_Wheel_Motor");
        rightDrive = hardwareMap.get(DcMotor.class, "Right_Wheel_Motor");
        servo = hardwareMap.get(Servo.class, "ring stopper 1");

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        while(opModeIsActive()) {
            leftDrive.setPower(gamepad1.left_stick_y+gamepad1.left_stick_x);
            rightDrive.setPower(gamepad1.left_stick_y-gamepad1.left_stick_x);

            if(gamepad1.left_trigger>0) {
                servo.setPosition(0.5);
            } else {
                servo.setPosition(0.1);
            }
        }
    }
}
