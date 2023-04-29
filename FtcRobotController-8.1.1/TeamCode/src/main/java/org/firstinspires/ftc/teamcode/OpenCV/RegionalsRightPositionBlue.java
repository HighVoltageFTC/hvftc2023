package org.firstinspires.ftc.teamcode.OpenCV;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XZY;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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
import java.util.List;
import java.util.Locale;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

@Autonomous
public class RegionalsRightPositionBlue extends LinearOpMode {

    BNO055IMU imu;

    private DistanceSensor sensorRange;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    private static final String VUFORIA_KEY =
            "Af6g/5T/////AAABmSTCgFYjeEbrtt8f88qMBkELieogvdyIvzCbFyCcptoKG4K6+4+Ag9gXXgdJd0DgpwndRc/TZ0axLMGdH1k9rLExMwrzHktMszFlHjz6d4s9zmLK37TLk3UeIe488y5+64WcyWFSbvjSfMFl+PL520i9TjDSI6KLgQqxwMhu70+8ndPtcsN+BaTaKEv2yrC9jQ1MjTZN4YFqV7qGzXHNo0/igidB+Olb+dlw+X/w1zY1YjcyGflfLifYShHnKRkLIWFRvuGHMefpRPsN2aBz6/GOZ8BRaI50yEieprlQqKiDP0159sLba8nCSMRSqdCAvrfPkI3rKBFH3TefV9UKrLVWhx4OoQ4A0AkMr8xPSJ0b";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;

    private static final float mmTargetHeight   = 6 * mmPerInch;          // the height of the center of the target image above the floor
    private static final float halfField        = 72 * mmPerInch;
    private static final float halfTile         = 12 * mmPerInch;
    private static final float oneAndHalfTile   = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation   = null;
    private VuforiaLocalizer vuforia    = null;
    private VuforiaTrackables targets   = null ;
    private WebcamName webcamName       = null;

    private boolean targetVisible       = false;

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

    private float xPosition = 0;
    private float yPosition = 0;

    private double currentDistance = 0;

    final int tickErrorMargin = 20;
    final int tickErrorMarginRotate = 60;

    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

    @Override
    public void runOpMode() {

        BNO055IMU.Parameters parameterss = new BNO055IMU.Parameters();
        parameterss.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameterss.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameterss.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameterss.loggingEnabled      = true;
        parameterss.loggingTag          = "IMU";
        parameterss.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameterss);
        composeTelemetry();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        //  ------------------------------



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

        sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) sensorRange;

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotorA.setDirection(DcMotorSimple.Direction.REVERSE);

        grabberMotor.setPower(1);

        boolean reachedSpot = false;
        boolean vuforiaInitialized = false;
        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {

            imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

            if (tagOfInterest == null) {

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
            }

            telemetry.update();
            sleep(20);

            if ((tagOfInterest != null) && (vuforiaInitialized == false)) {
                camera.closeCameraDevice();
                VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

                parameters.vuforiaLicenseKey = VUFORIA_KEY;

                webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
                // We also indicate which camera we wish to use.
                parameters.cameraName = webcamName;

                // Turn off Extended tracking.  Set this true if you want Vuforia to track beyond the target.
                parameters.useExtendedTracking = false;

                vuforia = ClassFactory.getInstance().createVuforia(parameters);

                // Load the data sets for the trackable objects. These particular data
                // sets are stored in the 'assets' part of our application.
                targets = this.vuforia.loadTrackablesFromAsset("PowerPlay");

                // For convenience, gather together all the trackable objects in one easily-iterable collection */

                allTrackables.addAll(targets);


                identifyTarget(0, "Red Audience Wall",   -halfField,  -oneAndHalfTile, mmTargetHeight, 90, 0,  90);
                identifyTarget(1, "Red Rear Wall",        halfField,  -oneAndHalfTile, mmTargetHeight, 90, 0, -90);
                identifyTarget(2, "Blue Audience Wall",  -halfField,   oneAndHalfTile, mmTargetHeight, 90, 0,  90);
                identifyTarget(3, "Blue Rear Wall",       halfField,   oneAndHalfTile, mmTargetHeight, 90, 0, -90);

                final float CAMERA_FORWARD_DISPLACEMENT  = 0.0f * mmPerInch;   // eg: Enter the forward distance from the center of the robot to the camera lens
                final float CAMERA_VERTICAL_DISPLACEMENT = 6.0f * mmPerInch;   // eg: Camera is 6 Inches above ground
                final float CAMERA_LEFT_DISPLACEMENT     = 0.0f * mmPerInch;   // eg: Enter the left distance from the center of the robot to the camera lens

                OpenGLMatrix cameraLocationOnRobot = OpenGLMatrix
                        .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                        .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XZY, DEGREES, 90, 90, 0));

                /**  Let all the trackable listeners know where the camera is.  */
                for (VuforiaTrackable trackable : allTrackables) {
                    ((VuforiaTrackableDefaultListener) trackable.getListener()).setCameraLocationOnRobot(parameters.cameraName, cameraLocationOnRobot);
                }

                targets.activate();
                updateVuforiaPosition(allTrackables);
                vuforiaInitialized = true;
            }

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
            navigateShort(allTrackables);
            backward(0.7, 1100);
            rotateLeft(0);
        } else if (tagOfInterest == null || tagOfInterest.id == position2) {
            navigateShort(allTrackables);
            backward(0.7, 100);
            rotateLeft(0);
        } else if (tagOfInterest.id == position3) {
            navigateShort(allTrackables);
            forward(0.7, 900);
            rotateLeft(0);
        }

        targets.deactivate();


        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        while (opModeIsActive()) {
            sleep(20);
        }
    }
    public boolean updateVuforiaPosition(List<VuforiaTrackable> trackables) {
        targetVisible = false;
        for (VuforiaTrackable trackable : trackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                telemetry.addData("Visible Target", trackable.getName());
                targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }

        // Provide feedback as to where the robot is located (if we know).
        if (targetVisible) {
            // express position (translation) of robot in inches.
            VectorF translation = lastLocation.getTranslation();
            telemetry.addData("Pos (inches)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

            // express the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);

            xPosition = translation.get(0)/mmPerInch;
            yPosition = translation.get(1)/mmPerInch;
        }
        else {
            telemetry.addData("Visible Target", "none");
        }
        telemetry.update();
        return(targetVisible);
    }
    public void navigateShort(List<VuforiaTrackable> trackables) {
        liftUp(1, 1200);
        forward(0.3, 300);
        left(0.4, 350);
        grabberMotor.setPower(0);
//        backward(0.3,100);
        left(0.5, 600);
        forward(0.5, 1900);
        rotateRight(-90);
        forward(0.5, 1700);
        liftDown(1, 800);
        positionByDistanceForward(12);
        grabberMotor.setPower(0.8);
        sleep(1000);
        liftUp(1, 800);
        positionByDistanceBackward(60);
        sendTextTelemetry("Move back from the Cone" + "..Stack");
        right(0.3, 675);
        sendTextTelemetry(" towards Junction2");
        grabberMotor.setPower(0);
   //     backward(0.3, 150);
        right(0.4, 500);
        while (!updateVuforiaPosition(allTrackables)) {};
        position();

//        rotateRight(-180);
//        sendTextTelemetry("Turn towards Junction2");
//        forward(0.4, 175);
//        sendTextTelemetry("Move forward to Junction2");
//        grabberMotor.setPower(0);
//        backward(0.4, 100);
    }

    public void navigateShortContinuous(List<VuforiaTrackable> trackables) {
        resetEncoder();
        forward(1, 325);
        setSpeed(1);
        leftContinuous(300);
        setSpeed(1);
        forwardContinuous( 1450);
        rotateRightContinuous(-90);
        liftUp(1, 1200);
        forwardContinuous( 200);
        grabberMotor.setPower(0);
        setSpeed(1);
        leftContinuous(700);
        setSpeed(1);
        forwardContinuous( 600);
        liftDown(1, 700);
        positionByDistanceForwardContinuous(12);
        grabberMotor.setPower(0.7);
        sleep(1000);
        liftUp(1, 800);
        positionByDistanceBackwardContinuous(39);
        rotateRightContinuous(-180);
        setSpeed(1);
        forwardContinuous( 175);
        grabberMotor.setPower(0);
        stopMotors();
    }


    public void position() {

        if(xPosition > -42) {
            telemetry.addData("Ticks to move backward: %d", (int) (xPosition+42)*50 );
            telemetry.update();
            backward(0.4,(int) (xPosition+42)*50 );
        } else if(xPosition < -42) {
            telemetry.addData("Ticks to move backward: %d", (int) (xPosition+42)*-50 );
            telemetry.update();
            forward(0.4,(int) (xPosition+42)*-50 );
        } if(yPosition > 39) {
            telemetry.addData("Ticks to move right: %d", (int) (yPosition-39)*50 );
            telemetry.update();
            right(0.4,(int) (yPosition - 39)*50 );
        } else if(yPosition < 39) {
            telemetry.addData("Ticks to move left: %d", (int) (39-yPosition)*50 );
            telemetry.update();
            left(0.4,(int) (39-yPosition)*50 );
        }

//        if(xPosition > 24) {
//            telemetry.addData("Ticks to move backward: %d", (int) (xPosition-24)*50 );
//            telemetry.update();
//            backward(0.4,(int) (xPosition-24)*50 );
//        } else if(xPosition < 24) {
//            telemetry.addData("Ticks to move backward: %d", (int) (xPosition-24)*50 );
//            telemetry.update();
//            forward(0.4,(int) (24-xPosition)*50 );
//        } if(yPosition > -30) {
//            telemetry.addData("Ticks to move right: %d", (int) (yPosition+ 30)*50 );
//            telemetry.update();
//            right(0.4,(int) (yPosition + 30)*50 );
//        } else if(yPosition < -30) {
//            telemetry.addData("Ticks to move left: %d", (int) (yPosition + 30)*-50 );
//            telemetry.update();
//            left(0.4,(int) (yPosition + 30)*-50 );
//        }
    }

    public float getCurrentDistance() {
        return (float) sensorRange.getDistance((DistanceUnit.CM));
    }

    void sendDistanceTelemetry(float Distance) {
        telemetry.addData("Current Distance: ", Distance);
        telemetry.update();
    }

    void sendTextTelemetry(String stepCompleted) {
        telemetry.addData("Completed: ", stepCompleted);
        telemetry.update();
    }

    public void positionByDistanceForward(float Distance) {
        int cmToMove = (int)Math.abs(Distance - getCurrentDistance());
        if (cmToMove > 300) {
            cmToMove = 14;
        }
        int ticksToMove = cmToMove * 20;
        forward(0.5, ticksToMove);
    }

    public void positionByDistanceForwardContinuous(float Distance) {
        int cmToMove = (int)Math.abs(Distance - getCurrentDistance());
        if (cmToMove > 300) {
            cmToMove = 14;
        }
        int ticksToMove = cmToMove * 20;
        setSpeed(1);
        forwardContinuous( ticksToMove);
    }

    public void positionByDistanceBackward(float Distance) {
        int cmToMove = (int)Math.abs(Distance - getCurrentDistance());
        if (cmToMove > 300) {
            cmToMove = 28;
        }
        int ticksToMove = cmToMove * 20;
        backward(0.4, ticksToMove);
    }

    public void positionByDistanceBackwardContinuous(float Distance) {
        int cmToMove = (int)Math.abs(Distance - getCurrentDistance());
        if (cmToMove > 300) {
            cmToMove = 28;
        }
        int ticksToMove = cmToMove * 20;
        setSpeed(1);
        backwardContinuous(ticksToMove);
    }



    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }


    void identifyTarget(int targetIndex, String targetName, float dx, float dy, float dz, float rx, float ry, float rz) {
        VuforiaTrackable aTarget = targets.get(targetIndex);
        aTarget.setName(targetName);
        aTarget.setLocation(OpenGLMatrix.translation(dx, dy, dz)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, rx, ry, rz)));
    }

    private void rotate(int degrees, double power)
    {
        double  frontleftpower, frontrightpower, backleftpower, backrightpower;

        runWithoutEncoder();
        // restart imu movement tracking.

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            frontleftpower = power;
            frontrightpower = -power;
            backleftpower = power;
            backrightpower = -power;
        }
        else if (degrees > 0)
        {   // turn left.
            frontleftpower = -power;
            frontrightpower = power;
            backleftpower = -power;
            backrightpower = power;
        }
        else return;

        // set power to rotate.
        ;

        backLeftMotor.setPower(backleftpower);
        frontLeftMotor.setPower(frontleftpower);
        backRightMotor.setPower(backrightpower);
        frontRightMotor.setPower(frontrightpower);


        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, DEGREES).firstAngle == 0) {
                composeTelemetry();
                telemetry.update();
            }

            while (opModeIsActive() && imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, DEGREES).firstAngle > degrees) {
                composeTelemetry();
                telemetry.addData("After: ",  imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, DEGREES).firstAngle);
                telemetry.update();
            }
        }
        else    // left turn.
            while (opModeIsActive() && imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, DEGREES).firstAngle < degrees) {
                composeTelemetry();
                telemetry.addData("After: ",  imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, DEGREES).firstAngle);
                telemetry.update();
            }

        // turn the motors off.
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);

        // wait for rotation to stop.

        sleep(1000);

        // reset angle tracking on new heading.
    }

    public float getCurrentHeading() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, DEGREES).firstAngle;
    }

    void sendAngleTelemetry(float angle) {
        telemetry.addData("Current Heading: ", angle);
        telemetry.update();
    }

    public void rotateRight(float angle) {
        int degressToRotate = (int)Math.abs(angle - getCurrentHeading());
        int ticksToRotate = degressToRotate * 9;
        spinRight(0.5, ticksToRotate);
    }

    public void rotateRightContinuous(float angle) {
        int degressToRotate = (int) Math.abs(angle - getCurrentHeading());
        int ticksToRotate = degressToRotate * 9;
        spinRightContinuous(ticksToRotate);
    }

    public void rotateLeft(float angle) {
        int degressToRotate = (int)Math.abs(getCurrentHeading() - angle);
        int ticksToRotate = degressToRotate * 9;
        spinLeft(0.5, ticksToRotate);
    }

    public void rotateLeftContinuous(float angle) {
        int degressToRotate = (int)Math.abs(getCurrentHeading() - angle);
        int ticksToRotate = degressToRotate * 9;
        spinLeftContinuous(ticksToRotate);
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

    public void runWithoutEncoder() {
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotorA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotorA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotorB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

    public void liftUp(double speed, int position) {
        liftMotorA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftMotorA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftToPosition(-position, -position);
        liftMotorA.setPower(-speed);
        liftMotorB.setPower(-speed);
        while (opModeIsActive() && liftMotorA.isBusy() || liftMotorB.isBusy()) {
            if(Math.abs(position-liftMotorA.getCurrentPosition())<tickErrorMargin &&
                    Math.abs(position- liftMotorB.getCurrentPosition()) <tickErrorMargin) {
                telemetry.addData("LiftA: %d",
                        liftMotorA.getCurrentPosition());
                telemetry.addData("LiftB: %d",
                        liftMotorB.getCurrentPosition());
                break;
            }
        }
        liftMotorA.setPower(0.05);
        liftMotorB.setPower(0.05);
    }

    public void liftDown(double speed, int position) {
        liftMotorA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftMotorA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftToPosition(position,position);
        liftMotorA.setPower(speed);
        liftMotorB.setPower(speed);
        while (opModeIsActive() && liftMotorA.isBusy() || liftMotorB.isBusy()) {
            if(Math.abs(position-liftMotorA.getCurrentPosition())<tickErrorMargin &&
                    Math.abs(position- liftMotorB.getCurrentPosition()) <tickErrorMargin) {
                break;
            }
        }
        telemetry.addLine("lift complete");
        telemetry.update();
        liftMotorA.setPower(0.05);
        liftMotorB.setPower(0.05);
    }

    public void liftToPosition(int liftMotorAPosition, int liftMotorBPosition) {
        if(liftMotorAPosition != 0 ) {
            liftMotorA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotorA.setTargetPosition(liftMotorAPosition);
        }
        if(liftMotorBPosition != 0 ) {
            liftMotorB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotorB.setTargetPosition(liftMotorBPosition);
        }
    }

    public void setSpeed(double speed) {
        backLeftMotor.setPower(speed);
        frontLeftMotor.setPower(speed);
        frontRightMotor.setPower(speed);
        backRightMotor.setPower(speed);
    }

    public void forward(double speed, int position) {
        resetEncoder();
        runToPosition(position, position, position, position);
        backLeftMotor.setPower(speed);
        frontLeftMotor.setPower(speed);
        frontRightMotor.setPower(speed);
        backRightMotor.setPower(speed);
        while (opModeIsActive() && backLeftMotor.isBusy() || backRightMotor.isBusy() || frontLeftMotor.isBusy() || frontRightMotor.isBusy()) {
            if(Math.abs(position-backLeftMotor.getCurrentPosition())< tickErrorMargin&&
                    Math.abs(position-backRightMotor.getCurrentPosition())<tickErrorMargin &&
                    Math.abs(position-frontLeftMotor.getCurrentPosition()) <tickErrorMargin &&
                    Math.abs(position- frontRightMotor.getCurrentPosition()) <tickErrorMargin) {
                telemetry.addData("bL: %d",
                        position-backLeftMotor.getCurrentPosition());
                telemetry.addData("bR: %d",
                        position-backRightMotor.getCurrentPosition());
                telemetry.addData("fL: %d",
                        position-frontLeftMotor.getCurrentPosition());
                telemetry.addData("fR: %d",
                        position-frontRightMotor.getCurrentPosition());
                telemetry.addData("Completed ", "forward");
                telemetry.update();

                break;
            }
        }
        stopMotors();
    }

    public void forwardContinuous(int position) {
        runToPosition((backLeftMotor.getCurrentPosition() + position),
                (frontLeftMotor.getCurrentPosition() + position),
                (frontRightMotor.getCurrentPosition() + position),
                (backRightMotor.getCurrentPosition() + position));
        while (opModeIsActive() && backLeftMotor.isBusy() || backRightMotor.isBusy() || frontLeftMotor.isBusy() || frontRightMotor.isBusy()) {
        }
    }

    public void backward(double speed, int position) {
        resetEncoder();
        runToPosition(-position, -position, -position, -position);
        backLeftMotor.setPower(-speed);
        frontLeftMotor.setPower(-speed);
        frontRightMotor.setPower(-speed);
        backRightMotor.setPower(-speed);
        while (opModeIsActive() && backLeftMotor.isBusy() || backRightMotor.isBusy() || frontLeftMotor.isBusy() || frontRightMotor.isBusy()) {
            if(Math.abs(position-backLeftMotor.getCurrentPosition())<tickErrorMargin &&
                    Math.abs(position-backRightMotor.getCurrentPosition())<tickErrorMargin &&
                    Math.abs(position-frontLeftMotor.getCurrentPosition()) <tickErrorMargin &&
                    Math.abs(position- frontRightMotor.getCurrentPosition()) <tickErrorMargin) {

                telemetry.addData("bL: %d",
                        position-backLeftMotor.getCurrentPosition());
                telemetry.addData("bR: %d",
                        position-backRightMotor.getCurrentPosition());
                telemetry.addData("fL: %d",
                        position-frontLeftMotor.getCurrentPosition());
                telemetry.addData("fR: %d",
                        position-frontRightMotor.getCurrentPosition());
                telemetry.addData("Completed ", "backward");
                telemetry.update();

                break;
            }
        }
        stopMotors();
    }

    public void backwardContinuous(int position) {
        runToPosition((backLeftMotor.getCurrentPosition() - position),
                (frontLeftMotor.getCurrentPosition() - position),
                (frontRightMotor.getCurrentPosition() - position),
                (backRightMotor.getCurrentPosition() - position));
        while (opModeIsActive() && backLeftMotor.isBusy() || backRightMotor.isBusy() || frontLeftMotor.isBusy() || frontRightMotor.isBusy()) {
        }
    }

    public void left(double speed, int position) {
        resetEncoder();
        runToPosition(position, -(position), -(position), position);
        backLeftMotor.setPower(speed);
        frontLeftMotor.setPower(-speed);
        frontRightMotor.setPower(-speed);
        backRightMotor.setPower(speed);
        while (opModeIsActive() && backLeftMotor.isBusy() || backRightMotor.isBusy() || frontLeftMotor.isBusy() || frontRightMotor.isBusy()) {
            if(Math.abs(position-backLeftMotor.getCurrentPosition())<tickErrorMargin &&
                    Math.abs(position-backRightMotor.getCurrentPosition())<tickErrorMargin &&
                    Math.abs(position-frontLeftMotor.getCurrentPosition()) <tickErrorMargin &&
                    Math.abs(position- frontRightMotor.getCurrentPosition()) <tickErrorMargin) {
                break;
            }
        }
        stopMotors();
    }

    public void leftContinuous(int position) {
        runToPosition((backLeftMotor.getCurrentPosition() + position),
                (frontLeftMotor.getCurrentPosition() - position),
                (frontRightMotor.getCurrentPosition() - position),
                (backRightMotor.getCurrentPosition() + position));
        while (opModeIsActive() && backLeftMotor.isBusy() || backRightMotor.isBusy() || frontLeftMotor.isBusy() || frontRightMotor.isBusy()) {
        }

    }

    public void right(double speed, int position) {
        resetEncoder();
        runToPosition(-position, position, position, -position);
        backLeftMotor.setPower(-speed);
        frontLeftMotor.setPower(speed);
        frontRightMotor.setPower(speed);
        backRightMotor.setPower(-speed);
        while (opModeIsActive() && backLeftMotor.isBusy() || backRightMotor.isBusy() || frontLeftMotor.isBusy() || frontRightMotor.isBusy()) {
            if(Math.abs(position-backLeftMotor.getCurrentPosition())<tickErrorMargin &&
                    Math.abs(position-backRightMotor.getCurrentPosition())<tickErrorMargin &&
                    Math.abs(position-frontLeftMotor.getCurrentPosition()) <tickErrorMargin &&
                    Math.abs(position- frontRightMotor.getCurrentPosition()) <tickErrorMargin) {
                break;
            }

        }
        stopMotors();
    }

    public void rightContinuous(int position) {
        runToPosition((backLeftMotor.getCurrentPosition() - position),
                (frontLeftMotor.getCurrentPosition() + position),
                (frontRightMotor.getCurrentPosition() + position),
                (backRightMotor.getCurrentPosition() - position));
        while (opModeIsActive() && backLeftMotor.isBusy() || backRightMotor.isBusy() || frontLeftMotor.isBusy() || frontRightMotor.isBusy()) {
        }
    }

    public void diagonalFrontRight(double speed, int position) {
        resetEncoder();
        runToPosition(0, position, 0, position);
        frontLeftMotor.setPower(speed);
        backRightMotor.setPower(speed);
        while (opModeIsActive() && backLeftMotor.isBusy() || backRightMotor.isBusy() || frontLeftMotor.isBusy() || frontRightMotor.isBusy()) {
            if(Math.abs(position-backLeftMotor.getCurrentPosition())<tickErrorMargin &&
                    Math.abs(position-backRightMotor.getCurrentPosition())<tickErrorMargin &&
                    Math.abs(position-frontLeftMotor.getCurrentPosition()) <tickErrorMargin &&
                    Math.abs(position- frontRightMotor.getCurrentPosition()) <tickErrorMargin) {
                break;
            }

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
            if(Math.abs(position-backLeftMotor.getCurrentPosition())<tickErrorMargin &&
                    Math.abs(position-backRightMotor.getCurrentPosition())<tickErrorMargin &&
                    Math.abs(position-frontLeftMotor.getCurrentPosition()) <tickErrorMargin &&
                    Math.abs(position- frontRightMotor.getCurrentPosition()) <tickErrorMargin) {
                break;
            }

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
            if(Math.abs(position-backLeftMotor.getCurrentPosition())<tickErrorMargin &&
                    Math.abs(position-backRightMotor.getCurrentPosition())<tickErrorMargin &&
                    Math.abs(position-frontLeftMotor.getCurrentPosition()) <tickErrorMargin &&
                    Math.abs(position- frontRightMotor.getCurrentPosition()) <tickErrorMargin) {
                break;
            }
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
            if(Math.abs(position-backLeftMotor.getCurrentPosition())<tickErrorMargin &&
                    Math.abs(position-backRightMotor.getCurrentPosition())<tickErrorMargin &&
                    Math.abs(position-frontLeftMotor.getCurrentPosition()) <tickErrorMargin &&
                    Math.abs(position- frontRightMotor.getCurrentPosition()) <tickErrorMargin) {
                break;
            }
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
            if(Math.abs(position-backLeftMotor.getCurrentPosition())<tickErrorMarginRotate &&
                    Math.abs(position-backRightMotor.getCurrentPosition())<tickErrorMarginRotate &&
                    Math.abs(position-frontLeftMotor.getCurrentPosition()) <tickErrorMarginRotate &&
                    Math.abs(position- frontRightMotor.getCurrentPosition()) <tickErrorMarginRotate) {

                telemetry.addData("bL: %d",
                        position-backLeftMotor.getCurrentPosition());
                telemetry.addData("bR: %d",
                        position-backRightMotor.getCurrentPosition());
                telemetry.addData("fL: %d",
                        position-frontLeftMotor.getCurrentPosition());
                telemetry.addData("fR: %d",
                        position-frontRightMotor.getCurrentPosition());
                telemetry.addData("Completed ", "spinright");
                telemetry.update();
                break;
            }
        }
        telemetry.addLine("rotate complete");
        telemetry.update();
        stopMotors();
    }

    public void spinRightContinuous(int position) {
        runToPosition((backLeftMotor.getCurrentPosition() + position),
                (frontLeftMotor.getCurrentPosition() + position),
                (frontRightMotor.getCurrentPosition() - position),
                (backRightMotor.getCurrentPosition() - position));
        while (opModeIsActive() && backLeftMotor.isBusy() || backRightMotor.isBusy() || frontLeftMotor.isBusy() || frontRightMotor.isBusy()) {
        }
    }
    public void spinLeft(double speed, int position) {
        resetEncoder();
        runToPosition(-position, -position, position, position);
        backLeftMotor.setPower(-speed);
        frontLeftMotor.setPower(-speed);
        frontRightMotor.setPower(speed);
        backRightMotor.setPower(speed);
        while (opModeIsActive() && backLeftMotor.isBusy() || backRightMotor.isBusy() || frontLeftMotor.isBusy() || frontRightMotor.isBusy()) {
            if(Math.abs(position-backLeftMotor.getCurrentPosition())<tickErrorMarginRotate &&
                    Math.abs(position-backRightMotor.getCurrentPosition())<tickErrorMarginRotate &&
                    Math.abs(position-frontLeftMotor.getCurrentPosition()) <tickErrorMarginRotate &&
                    Math.abs(position- frontRightMotor.getCurrentPosition()) <tickErrorMarginRotate) {

                telemetry.addData("bL: %d",
                        position-backLeftMotor.getCurrentPosition());
                telemetry.addData("bR: %d",
                        position-backRightMotor.getCurrentPosition());
                telemetry.addData("fL: %d",
                        position-frontLeftMotor.getCurrentPosition());
                telemetry.addData("fR: %d",
                        position-frontRightMotor.getCurrentPosition());
                telemetry.addData("Completed ", "spinleft");
                telemetry.update();
                break;
            }
        }
        stopMotors();
    }

    public void spinLeftContinuous(int position) {
        runToPosition((backLeftMotor.getCurrentPosition() - position),
                (frontLeftMotor.getCurrentPosition() - position),
                (frontRightMotor.getCurrentPosition() + position),
                (backRightMotor.getCurrentPosition() + position));
        while (opModeIsActive() && backLeftMotor.isBusy() || backRightMotor.isBusy() || frontLeftMotor.isBusy() || frontRightMotor.isBusy()) {
        }
    }
}


