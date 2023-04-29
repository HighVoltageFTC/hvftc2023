package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="EncodersAutonomous", group = "LinearOpMode")
@Disabled
public class EncodersAutonomous extends LinearOpMode {

    private DcMotor backLeftMotor = null;
    private DcMotor backRightMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor frontLeftMotor = null;

    @Override
    public void runOpMode(){
        backLeftMotor = hardwareMap.get(DcMotor.class, "Back_Left_Motor");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "Front_Left_Motor");
        backRightMotor = hardwareMap.get(DcMotor.class, "Back_Right_Motor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "Front_Right_Motor");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        forward(0.1,1000);


    }

    public void resetEncoder() {
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void runToPosition(int backLeftPosition, int frontLeftPosition, int frontRightPosition, int backRightPosition) {
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeftMotor.setTargetPosition(frontLeftPosition);
        backLeftMotor.setTargetPosition(backLeftPosition);
        backRightMotor.setTargetPosition(backRightPosition);
        frontRightMotor.setTargetPosition(frontRightPosition);
    }

    public void stopMotors() {
        backLeftMotor.setPower(0);
        frontLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        frontRightMotor.setPower(0);
    }
    public void forward(double speed, int position) {
        resetEncoder();
        runToPosition(position, position, position, position);
        backLeftMotor.setPower(speed);
        frontLeftMotor.setPower(speed);
        backRightMotor.setPower(speed);
        frontRightMotor.setPower(speed);
        while (opModeIsActive() && backLeftMotor.isBusy() && backRightMotor.isBusy() && frontLeftMotor.isBusy() && frontRightMotor.isBusy());

    }
    public void backward(double speed, int position) {
        resetEncoder();
        runToPosition(-position, -position, -position, -position);
        backLeftMotor.setPower(-speed);
        frontLeftMotor.setPower(-speed);
        backRightMotor.setPower(-speed);
        frontRightMotor.setPower(-speed);
        while (opModeIsActive() && backLeftMotor.isBusy() && backRightMotor.isBusy() && frontLeftMotor.isBusy() && frontRightMotor.isBusy());

    }
    public void left (double speed, int position) {
        resetEncoder();
        runToPosition(position, -position, -position, position);
        backLeftMotor.setPower(speed);
        frontLeftMotor.setPower(-speed);
        backRightMotor.setPower(-speed);
        frontRightMotor.setPower(speed);
        while (opModeIsActive() && backLeftMotor.isBusy() && backRightMotor.isBusy() && frontLeftMotor.isBusy() && frontRightMotor.isBusy());

    }
    public void right (double speed, int position) {
        resetEncoder();
        runToPosition(-position, position, position,-position);
        backLeftMotor.setPower(-speed);
        frontLeftMotor.setPower(speed);
        backRightMotor.setPower(speed);
        frontRightMotor.setPower(-speed);
        while (opModeIsActive() && backLeftMotor.isBusy() && backRightMotor.isBusy() && frontLeftMotor.isBusy() && frontRightMotor.isBusy());

    }
    public void diagonalFrontLeft(double speed, int position) {
        resetEncoder();
        runToPosition(position, 0, position, 0);
        backLeftMotor.setPower(speed);
        frontLeftMotor.setPower(0);
        backRightMotor.setPower(speed);
        frontRightMotor.setPower(0);
        while (opModeIsActive() && backLeftMotor.isBusy() && backRightMotor.isBusy() && frontLeftMotor.isBusy() && frontRightMotor.isBusy());

    }
    public void diagonalFrontRight(double speed,int position) {
        resetEncoder();
        runToPosition(0, position, 0, position);
        backLeftMotor.setPower(0);
        frontLeftMotor.setPower(speed);
        backRightMotor.setPower(0);
        frontRightMotor.setPower(speed);
        while (opModeIsActive() && backLeftMotor.isBusy() && backRightMotor.isBusy() && frontLeftMotor.isBusy() && frontRightMotor.isBusy());

    }
    public void diagonalBackLeft(double speed, int position) {
        resetEncoder();
        runToPosition(0, -position, 0, -position);
        backLeftMotor.setPower(0);
        frontLeftMotor.setPower(-speed);
        backRightMotor.setPower(0);
        frontRightMotor.setPower(-speed);
        while (opModeIsActive() && backLeftMotor.isBusy() && backRightMotor.isBusy() && frontLeftMotor.isBusy() && frontRightMotor.isBusy());

    }
    public void diagonalBackRight(double speed, int position) {
        resetEncoder();
        runToPosition(-position, 0, -position, 0);
        backLeftMotor.setPower(-speed);
        frontLeftMotor.setPower(0);
        backRightMotor.setPower(-speed);
        frontRightMotor.setPower(0);
        while (opModeIsActive() && backLeftMotor.isBusy() && backRightMotor.isBusy() && frontLeftMotor.isBusy() && frontRightMotor.isBusy());

    }
    public void spinRight(double speed, int position) {
        resetEncoder();
        runToPosition(-position, -position, position, position);
        backLeftMotor.setPower(speed);
        frontLeftMotor.setPower(speed);
        backRightMotor.setPower(-speed);
        frontRightMotor.setPower(-speed);
        while (opModeIsActive() && backLeftMotor.isBusy() && backRightMotor.isBusy() && frontLeftMotor.isBusy() && frontRightMotor.isBusy());

    }
    public void spinLeft(double speed, int position) {
        resetEncoder();
        runToPosition(position, position, -position, -position);
        backLeftMotor.setPower(-speed);
        frontLeftMotor.setPower(-speed);
        backRightMotor.setPower(speed);
        frontRightMotor.setPower(speed);
        while (opModeIsActive() && backLeftMotor.isBusy() && backRightMotor.isBusy() && frontLeftMotor.isBusy() && frontRightMotor.isBusy());

    }

}


