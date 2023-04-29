    package org.firstinspires.ftc.teamcode;

    import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
    import com.qualcomm.robotcore.eventloop.opmode.Disabled;
    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
    import com.qualcomm.robotcore.hardware.DcMotor;
    import com.qualcomm.robotcore.hardware.DcMotorSimple;
    import java.util.List;
    import org.firstinspires.ftc.robotcore.external.ClassFactory;
    import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
    import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
    import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
    import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

    @Autonomous(name="meet2RightPosition", group = "LinearOpMode")
    @Disabled
    public class meet2RightPosition extends LinearOpMode {

        private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";

        private static final String[] LABELS = {
                "1 Bolt",
                "2 Bulb",
                "3 Panel"
        };

        private static final String VUFORIA_KEY =
                "Af6g/5T/////AAABmSTCgFYjeEbrtt8f88qMBkELieogvdyIvzCbFyCcptoKG4K6+4+Ag9gXXgdJd0DgpwndRc/TZ0axLMGdH1k9rLExMwrzHktMszFlHjz6d4s9zmLK37TLk3UeIe488y5+64WcyWFSbvjSfMFl+PL520i9TjDSI6KLgQqxwMhu70+8ndPtcsN+BaTaKEv2yrC9jQ1MjTZN4YFqV7qGzXHNo0/igidB+Olb+dlw+X/w1zY1YjcyGflfLifYShHnKRkLIWFRvuGHMefpRPsN2aBz6/GOZ8BRaI50yEieprlQqKiDP0159sLba8nCSMRSqdCAvrfPkI3rKBFH3TefV9UKrLVWhx4OoQ4A0AkMr8xPSJ0b";

        private VuforiaLocalizer vuforia;
        private TFObjectDetector tfod;


        private DcMotor backLeftMotor;
        private DcMotor backRightMotor;
        private DcMotor frontLeftMotor;
        private DcMotor frontRightMotor;
        private DcMotor liftMotor = null;
        private DcMotor grabberMotor = null;

        @Override
        public void runOpMode() {

            initVuforia();
            initTfod();

            if (tfod != null) {
                tfod.activate();
                tfod.setZoom(1.5, 16.0/9.0);
            }
            telemetry.addData(">", "Press Play to start op mode");
            telemetry.update();


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
            int parkingLot = 2;  // bolt = 1,  bulb = 2, panel = 3
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Objects Detected", updatedRecognitions.size());

                    // step through the list of recognitions and display image position/size information for each one
                    // Note: "Image number" refers to the randomized image orientation/number
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals("1 Bolt")) {
                            parkingLot = 1;
                        }
                        if (recognition.getLabel().equals("2 Bulb")) {
                            parkingLot = 2;
                        }
                        if (recognition.getLabel().equals("3 Panel")) {
                            parkingLot = 3;
                        }
                        telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100 );
                    }
                    telemetry.addData("parkingLot: ", parkingLot);
                    telemetry.update();
                }
            }

            if(parkingLot == 1) {
                right(0.2, 840);
                forward(0.4, 2200);
    //        forward(0.2, 350);
                spinRight(0.3, 40);
                left(0.3, 1770);
                liftMotor.setPower(1);
                sleep(1800);
                liftMotor.setPower(0.05);
                forward(0.1, 180);
                grabberMotor.setPower(-0.2);
                sleep(80);
                grabberMotor.setPower(0);
                sleep(500);
                grabberMotor.setPower(0);
                left(0.4,650);
                backward(0.4,190);
            } else if(parkingLot == 2) {
                right(0.2, 840);
                forward(0.4, 2200);
    //        forward(0.2, 350);
                spinRight(0.3, 40);
                left(0.3, 1770);
                liftMotor.setPower(1);
                sleep(1800);
                liftMotor.setPower(0.05);
                forward(0.1, 180);
                grabberMotor.setPower(-0.2);
                sleep(80);
                grabberMotor.setPower(0);
                sleep(500);
                grabberMotor.setPower(0);
                backward(0.4, 200);
                right(0.4,650);
            } else if(parkingLot == 3){
                right(0.2, 840);
                forward(0.4, 2200);
    //        forward(0.2, 350);
                spinRight(0.3, 40);
                right(0.3, 40);
                left(0.3, 1770);
                liftMotor.setPower(1);
                sleep(1800);
                liftMotor.setPower(0.05);
                forward(0.1, 180
                );
                grabberMotor.setPower(-0.2);
                sleep(80);
                grabberMotor.setPower(0);
                sleep(500);
                grabberMotor.setPower(0);
                backward(0.4, 200);
                right(0.4,1800);
            }



        }
        public void testMotorSpeed(DcMotor motor, double speed) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(speed);

            while (opModeIsActive()) {

                telemetry.addData("posotion value", motor.getCurrentPosition());
                telemetry.update();
                motor.setPower(speed);

                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Objects Detected", updatedRecognitions.size());

                        // step through the list of recognitions and display image position/size information for each one
                        // Note: "Image number" refers to the randomized image orientation/number
                        for (Recognition recognition : updatedRecognitions) {
                            double col = (recognition.getLeft() + recognition.getRight()) / 2 ;
                            double row = (recognition.getTop()  + recognition.getBottom()) / 2 ;
                            double width  = Math.abs(recognition.getRight() - recognition.getLeft()) ;
                            double height = Math.abs(recognition.getTop()  - recognition.getBottom()) ;

                            telemetry.addData(""," ");
                            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100 );
                            telemetry.addData("- Position (Row/Col)","%.0f / %.0f", row, col);
                            telemetry.addData("- Size (Width/Height)","%.0f / %.0f", width, height);
                        }
                        telemetry.update();
                    }
                }
            }
            telemetry.update();
            motor.setPower(0);
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
        }

        /**
         * Initialize the TensorFlow Object Detection engine.
         */
        private void initTfod() {
            int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                    "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
            tfodParameters.minResultConfidence = 0.75f;
            tfodParameters.isModelTensorFlow2 = true;
            tfodParameters.inputSize = 300;
            tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

            // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
            // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
            tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
            // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
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
