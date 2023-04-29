package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

@Autonomous(name="RedAlliancePosition1Camera", group = "LinearOpMode")
@Disabled
public class  RedAlliancePosition1Camera extends LinearOpMode {

    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor wheelMotor = null;
    private DcMotor liftMotor = null;
    private Servo grabberServo = null;

    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };

    private static final String VUFORIA_KEY =
            "Af6g/5T/////AAABmSTCgFYjeEbrtt8f88qMBkELieogvdyIvzCbFyCcptoKG4K6+4+Ag9gXXgdJd0DgpwndRc/TZ0axLMGdH1k9rLExMwrzHktMszFlHjz6d4s9zmLK37TLk3UeIe488y5+64WcyWFSbvjSfMFl+PL520i9TjDSI6KLgQqxwMhu70+8ndPtcsN+BaTaKEv2yrC9jQ1MjTZN4YFqV7qGzXHNo0/igidB+Olb+dlw+X/w1zY1YjcyGflfLifYShHnKRkLIWFRvuGHMefpRPsN2aBz6/GOZ8BRaI50yEieprlQqKiDP0159sLba8nCSMRSqdCAvrfPkI3rKBFH3TefV9UKrLVWhx4OoQ4A0AkMr8xPSJ0b";


    private VuforiaLocalizer vuforia;


    private TFObjectDetector tfod;
    @Override
    public void runOpMode() {
        initVuforia();
        initTfod();
        backLeftMotor = hardwareMap.get(DcMotor.class, "Back_Left_Motor");
        backRightMotor = hardwareMap.get(DcMotor.class, "Back_Right_Motor");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "Front_Left_Motor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "Front_Right_Motor");
        wheelMotor = hardwareMap.get(DcMotor.class, "Wheel_Motor");
        liftMotor = hardwareMap.get(DcMotor.class, "Lift_Motor");
        grabberServo = hardwareMap.get(Servo.class, "Grabber_Servo");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        grabberServo.setPosition(0);
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1, 16.0 / 9.0);
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();


        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = null;
                    Recognition duckRecognition = null;
                    for (int i = 0; i < 5; i++) {
                        updatedRecognitions = tfod.getUpdatedRecognitions();
                        telemetry.addData(String.format("Size (%d)", i), updatedRecognitions.size());
                        telemetry.update();
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals("Duck")) {
                                telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                                telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                        recognition.getLeft(), recognition.getTop());
                                telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                        recognition.getRight(), recognition.getBottom());
                                duckRecognition = recognition;
                            }
                        }
                        sleep(100);
                    }


                    if (duckRecognition != null) {
                        telemetry.addData("# Object Detected", "Duck");
                        telemetry.update();

                        telemetry.addData("  left,top", "%.03f , %.03f",
                                duckRecognition.getLeft(), duckRecognition.getTop());
                        telemetry.addData("  right,bottom", "%.03f , %.03f",
                                duckRecognition.getRight(), duckRecognition.getBottom());


                        // lets say middle was
                        //left top 274, 228
                        // right bottom 332, 281
                        if(duckRecognition.getLeft() > 90  && duckRecognition.getRight() < 250) {
                            // call program for duck on left
                            telemetry.addData("Duck Location", "left");
                            forward(0.5, 850);
                            spinRight(0.4, 425);

                            liftMotor.setPower(0.5);
                            sleep(500);
                            // top, 1600
                            // middle, 1100
                            // low, 500
                            liftMotor.setPower(0.05);

                            telemetry.addData("Going", "Forward" );
                            telemetry.update();
                            forward(0.1, 610);

                            grabberServo.setPosition(1);
                            sleep(1000);
//1275, 1350, 1000
                            backward(0.6, 1300);
                            spinLeft(0.4, 1375);
                            forward(0.3, 950);

                            wheelMotor.setPower(0.7);
                            sleep(3000);

                            backward(0.3, 200);
                            right(0.6, 950);

                        }
//
                        if(duckRecognition.getLeft() > 350 && duckRecognition.getRight() < 500) {
                            // call program for duck in the middle
                            telemetry.addData("Duck Location", "middle");
                            forward(0.5, 850);
                            spinRight(0.4, 425);

                            liftMotor.setPower(0.5);
                            sleep(1100);
                            // top, 1600
                            // middle, 1100
                            // low, 500
                            liftMotor.setPower(0.05);

                            telemetry.addData("Going", "Forward" );
                            telemetry.update();
                            forward(0.1, 610);

                            grabberServo.setPosition(1);
                            sleep(1000);
//1275, 1350, 1000
                            backward(0.6, 1300);
                            spinLeft(0.4, 1375);
                            forward(0.3, 950);

                            wheelMotor.setPower(0.7);
                            sleep(3000);

                            backward(0.3, 200);
                            right(0.4, 950);
                        }

                    } else {
                        telemetry.addData("Duck Location", "right");
                        forward(0.5, 850);
                        spinRight(0.4, 425);

                        liftMotor.setPower(0.5);
                        sleep(1600);
                        // top, 1600
                        // middle, 1100
                        // low, 500
                        liftMotor.setPower(0.05);

                        telemetry.addData("Going", "Forward" );
                        telemetry.update();
                        forward(0.1, 610);

                        grabberServo.setPosition(1);
                        sleep(1000);
//1275, 1350, 1000
                        backward(0.6, 1300);
                        spinLeft(0.4, 1350);
                        forward(0.3, 950);

                        wheelMotor.setPower(0.7);
                        sleep(3000);

                        backward(0.3, 200);
                        right(0.4, 950);
                    }
                    telemetry.update();
                    sleep(1000);
                }
                break;
            }
        }

    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }


    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
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
