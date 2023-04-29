package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="meet1RedAndBlueLeftPosition", group = "LinearOpMode")
@Disabled
public class meet1RedAndBlueLeftPosition extends LinearOpMode {
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor liftMotor = null;
    private DcMotor grabberMotor = null;

    @Override
    public void runOpMode() {
        backLeftMotor = hardwareMap.get(DcMotor.class, "Back_Left_Motor");
        backRightMotor = hardwareMap.get(DcMotor.class, "Back_Right_Motor");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "Front_Left_Motor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "Front_Right_Motor");
        liftMotor = hardwareMap.get(DcMotor.class, "Lift_Motor");
        grabberMotor = hardwareMap.get(DcMotor.class, "Grabber_Motor");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        grabberMotor.setPower(0.5);
        waitForStart();
        left(0.4, 820);
        forward(0.5, 2200);
//        forward(0.2, 350);
        right(0.4, 900);
//        liftMotor.setPowcfxr er(1);
//        sleep(1800);
//        liftMotor.setPower(0.05);
//        forward(0.1, 190);
//        grabberMotor.setPower(-0.2);
//        sleep(80);
//        grabberMotor.setPower(0);
//        sleep(500);
//        grabberMotor.setPower(0);
//        right(0.4,650);








    }
    public void testMotorSpeed(DcMotor motor, double speed) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(speed);

        while (opModeIsActive()) {

            telemetry.addData("posotion value", motor.getCurrentPosition());
            telemetry.update();
            motor.setPower(speed);
        }
        telemetry.update();
        motor.setPower(0);
    }

    public void testMotorEncoder(DcMotor motor, int position, double speed) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setTargetPosition(position);
        motor.setPower(speed);

        while (opModeIsActive() && motor.isBusy()) {

            telemetry.addData("busy value",motor.isBusy() );
            telemetry.addData("posotion value", motor.getCurrentPosition());
            telemetry.update();
            motor.setPower(speed);
        }
        telemetry.addData("busy value",motor.isBusy() );
        telemetry.update();
        motor.setPower(0);
    }

    public void resetEncoder() {
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void runToPosition(int backLeftPosition, int frontLeftPosition, int frontRightPosition, int backRightPosition ) {
        if(backLeftPosition != 0 ) {
            backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftMotor.setTargetPosition(backLeftPosition);

        }
        if(backRightPosition != 0 ) {
            backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightMotor.setTargetPosition(backRightPosition);

        }
        if(frontLeftPosition != 0 ) {
            frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontLeftMotor.setTargetPosition(frontLeftPosition);

        }
        if(frontRightPosition != 0 ) {
            frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightMotor.setTargetPosition(frontRightPosition);
        }
        telemetry.update();
    }

    public void stopMotors() {
        backLeftMotor.setPower(0);
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
    }


    public void forward(double speed, int position) {
        resetEncoder();
        runToPosition(position, position, position, position);
        backLeftMotor.setPower(speed);
        frontLeftMotor.setPower(speed);
        frontRightMotor.setPower(speed);
        backRightMotor.setPower(speed);
        while (opModeIsActive() && backLeftMotor.isBusy() || backRightMotor.isBusy() || frontLeftMotor.isBusy() || frontRightMotor.isBusy()) {
        }
        stopMotors();
    }
    public void backward(double speed, int position) {
        resetEncoder();
        runToPosition(-position, -position, -position, -position);
        backLeftMotor.setPower(-speed);
        frontLeftMotor.setPower(-speed);
        frontRightMotor.setPower(-speed);
        backRightMotor.setPower(-speed);
        while (opModeIsActive() && backLeftMotor.isBusy() || backRightMotor.isBusy() || frontLeftMotor.isBusy() || frontRightMotor.isBusy()) {
        }
        stopMotors();
    }
    public void left(double speed, int position) {
        resetEncoder();
        runToPosition(position, -(position), position, -(position));
        backLeftMotor.setPower(speed);
        frontLeftMotor.setPower(-speed);
        frontRightMotor.setPower(speed);
        backRightMotor.setPower(-speed);
        while (opModeIsActive() && backLeftMotor.isBusy() || backRightMotor.isBusy() || frontLeftMotor.isBusy() || frontRightMotor.isBusy()) {

        }
        stopMotors();
    }
    public void right(double speed, int position) {
        resetEncoder();
        runToPosition(-position, position, -position, position);
        backLeftMotor.setPower(-speed);
        frontLeftMotor.setPower(speed);
        frontRightMotor.setPower(-speed);
        backRightMotor.setPower(speed);
        while (opModeIsActive() && backLeftMotor.isBusy() || backRightMotor.isBusy() || frontLeftMotor.isBusy() || frontRightMotor.isBusy()) {

        }
        stopMotors();
    }
    public void diagonalFrontRight(double speed, int position) {
        resetEncoder();
        runToPosition(0, position, 0, position);
        frontLeftMotor.setPower(speed);
        backRightMotor.setPower(speed);
        while (opModeIsActive() && backLeftMotor.isBusy() || backRightMotor.isBusy() || frontLeftMotor.isBusy() || frontRightMotor.isBusy()) {

        }
        stopMotors();
    }
    public void diagonalFrontLeft(double speed, int position) {
        resetEncoder();
        runToPosition(position, 0, position, 0);
        backLeftMotor.setPower(speed);
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(speed);
        backRightMotor.setPower(0);
        while (opModeIsActive() && backLeftMotor.isBusy() || backRightMotor.isBusy() || frontLeftMotor.isBusy() || frontRightMotor.isBusy()) {

        }
        stopMotors();
    }
    public void diagonalBackRight(double speed, int position) {
        resetEncoder();
        runToPosition(-position, 0, -position, 0);
        backLeftMotor.setPower(-speed);
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(-speed);
        backRightMotor.setPower(0);
        while (opModeIsActive() && backLeftMotor.isBusy() || backRightMotor.isBusy() || frontLeftMotor.isBusy() || frontRightMotor.isBusy()) {

        }
        stopMotors();
    }
    public void diagonalBackLeft(double speed, int position) {
        resetEncoder();
        runToPosition(0, -position, 0, -position);
        backLeftMotor.setPower(0);
        frontLeftMotor.setPower(-speed);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(-speed);
        while (opModeIsActive() && backLeftMotor.isBusy() || backRightMotor.isBusy() || frontLeftMotor.isBusy() || frontRightMotor.isBusy()) {
        }
        stopMotors();
    }
    public void spinRight(double speed, int position) {
        resetEncoder();
        runToPosition(position, position, -position, -position);
        backLeftMotor.setPower(speed);
        frontLeftMotor.setPower(speed);
        frontRightMotor.setPower(-speed);
        backRightMotor.setPower(-speed);
        while (opModeIsActive() && backLeftMotor.isBusy() || backRightMotor.isBusy() || frontLeftMotor.isBusy() || frontRightMotor.isBusy()) {

        }
        stopMotors();
    }
    public void spinLeft(double speed, int position) {
        resetEncoder();
        runToPosition(-position, -position, position, position);
        backLeftMotor.setPower(-speed);
        frontLeftMotor.setPower(-speed);
        frontRightMotor.setPower(speed);
        backRightMotor.setPower(speed);
        while (opModeIsActive() && backLeftMotor.isBusy() || backRightMotor.isBusy() || frontLeftMotor.isBusy() || frontRightMotor.isBusy()) {

        }
        stopMotors();
    }

}
