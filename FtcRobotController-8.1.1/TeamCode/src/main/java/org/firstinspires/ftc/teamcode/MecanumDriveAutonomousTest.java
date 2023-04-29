package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
@Disabled

public class MecanumDriveAutonomousTest extends LinearOpMode {

    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;

    @Override
    public void runOpMode() {

        frontLeftMotor = hardwareMap.get(DcMotor.class,"Front_Left_Motor");
        frontRightMotor = hardwareMap.get(DcMotor.class,"Front_Right_Motor");
        backLeftMotor = hardwareMap.get(DcMotor.class,"Back_Left_Motor");
        backRightMotor = hardwareMap.get(DcMotor.class,"Back_Right_Motor");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);



        waitForStart();

        while (opModeIsActive()){

            frontLeftMotor.setPower(0.5);
            frontRightMotor.setPower(0.5);
            backLeftMotor.setPower(0.5);
            backRightMotor.setPower(0.5);

            sleep(1000);

            frontLeftMotor.setPower(0.2);
            frontRightMotor.setPower(0.2);
            backLeftMotor.setPower(0.2);
            backRightMotor.setPower(0.2);

            sleep(1000);

            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backLeftMotor.setPower(0);
            backRightMotor.setPower(0);

            sleep(2000);

            frontLeftMotor.setPower(-0.5);
            frontRightMotor.setPower(-0.5);
            backLeftMotor.setPower(-0.5);
            backRightMotor.setPower(-0.5);

            sleep(2000);

            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backLeftMotor.setPower(0);
            backRightMotor.setPower(0);

            sleep(2000);

            frontLeftMotor.setPower(-0.5);
            frontRightMotor.setPower(0.5);
            backLeftMotor.setPower(-0.5);
            backRightMotor.setPower(0.5);

            sleep(1000);

            break;

        }
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }
}
