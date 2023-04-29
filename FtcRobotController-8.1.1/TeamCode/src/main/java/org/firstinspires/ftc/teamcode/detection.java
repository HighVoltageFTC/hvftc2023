package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

@TeleOp(name = "Object detection", group = "TeleOp")
@Disabled
public class detection extends LinearOpMode {
    /* Note: This sample uses the all-objects Tensor Flow model (FreightFrenzy_BCDM.tflite), which contains
     * the following 4 detectable objects
     *  0: Ball,
     *  1: Cube,
     *  2: Duck,
     *  3: Marker (duck location tape marker)
     *
     *  Two additional model assets are available which only contain a subset of the objects:
     *  FreightFrenzy_BC.tflite  0: Ball,  1: Cube
     *  FreightFrenzy_DM.tflite  0: Duck,  1: Marker
     */
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

        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.15, 16.0/9.0);
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
                        sleep(200);
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
                        if(duckRecognition.getLeft() > 100  && duckRecognition.getRight() < 250) {
                            // call program for duck on left
                            telemetry.addData("Duck Location", "left");
                        }
//
                        if(duckRecognition.getLeft() > 350 && duckRecognition.getRight() < 500) {
                            // call program for duck in the middle
                            telemetry.addData("Duck Location", "middle");
                        }

                    } else {
                        telemetry.addData("Duck Location", "right");
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
}
