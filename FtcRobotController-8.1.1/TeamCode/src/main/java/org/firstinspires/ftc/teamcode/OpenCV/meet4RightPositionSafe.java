package org.firstinspires.ftc.teamcode.OpenCV;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import java.util.ArrayList;

@Autonomous
public class meet4RightPositionSafe extends LinearOpMode {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    //Tag ID of sleeve
    int position1 = 10;
    int position2 = 11;
    int position3 = 12;

    AprilTagDetection tagOfInterest = null;

    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor liftMotorA = null;
    private DcMotor liftMotorB = null;
    private DcMotor grabberMotor = null;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        backLeftMotor = hardwareMap.get(DcMotor.class, "Back_Left_Motor");
        backRightMotor = hardwareMap.get(DcMotor.class, "Back_Right_Motor");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "Front_Left_Motor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "Front_Right_Motor");
        liftMotorA = hardwareMap.get(DcMotor.class, "Lift_Motor_A");
        liftMotorB = hardwareMap.get(DcMotor.class, "Lift_Motor_B");
        grabberMotor = hardwareMap.get(DcMotor.class, "Grabber_Motor");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        grabberMotor.setPower(-0.5);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == position1 || tag.id == position2 || tag.id == position3) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */

        if (tagOfInterest.id == position1) {
            navigateShort();
            left(0.4, 600);
            forward(0.4, 900);
        } else if (tagOfInterest == null || tagOfInterest.id == position2) {
            navigateShort();
            right(0.4, 600);
            forward(0.4, 900);
        } else if (tagOfInterest.id == position3) {
            navigateShort();
            right(0.4, 1800);
            forward(0.4, 900);
        }



        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        while (opModeIsActive()) {
            sleep(20);
        }
    }

    public void navigateShort() {
        liftMotorA.setPower(1);
        liftMotorB.setPower(-1);
        sleep(600);
        liftMotorA.setPower(0.05);
        liftMotorB.setPower(-0.05);
        forward(0.2, 350);
        left(0.5, 850);
        grabberMotor.setPower(0);
        sleep(500);
        liftMotorA.setPower(0.05);
        liftMotorB.setPower(-0.05);
    }

    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
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
        runToPosition(position, -(position), -(position), position);
        backLeftMotor.setPower(speed);
        frontLeftMotor.setPower(-speed);
        frontRightMotor.setPower(-speed);
        backRightMotor.setPower(speed);
        while (opModeIsActive() && backLeftMotor.isBusy() || backRightMotor.isBusy() || frontLeftMotor.isBusy() || frontRightMotor.isBusy()) {

        }
        stopMotors();
    }
    public void right(double speed, int position) {
        resetEncoder();
        runToPosition(-position, position, position, -position);
        backLeftMotor.setPower(-speed);
        frontLeftMotor.setPower(speed);
        frontRightMotor.setPower(speed);
        backRightMotor.setPower(-speed);
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
