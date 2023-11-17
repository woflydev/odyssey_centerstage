package org.firstinspires.ftc.teamcode.drive.vision2;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.FSM_Auto_State.*;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;


import java.util.List;

public class TFPropPipeline {

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    private static final String RED_PROP_ASSET = "RedPropNewCamera.tflite";
    private static final String BLUE_PROP_ASSET = "BluePropNewCamera.tflite";

    public static final int MIDDLE_X = 640 / 2;
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    // Define the labels recognized in the model for TFOD (must be in training order!)

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    private Randomisation location = Randomisation.LOCATION_2;

    private double ANGLE_THRESHOLD = Math.toRadians(30);

    private RobotAlliance alliance;
    private CameraName cameraName;

    private Telemetry t;

    public enum Randomisation {
        LOCATION_1,
        LOCATION_2,
        LOCATION_3,
        NONE
    }

    public TFPropPipeline(CameraName camera, RobotAlliance alliance, Telemetry t) {
        this.alliance = alliance;
        this.cameraName = camera;
        initTfod();
        t.addData("DS preview on/off", "3 dots, Camera Stream");
        t.addData(">", "Touch Play to start OpMode");
        t.update();
    }

    public void stop() {
        visionPortal.close();
    }

    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                .setModelAssetName(alliance == RobotAlliance.RED ? RED_PROP_ASSET : BLUE_PROP_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                .setModelLabels(alliance == RobotAlliance.RED ? new String[]{"RedProp"} : new String[] {"BlueProp"})
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        builder.setCamera(this.cameraName);

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        //tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    public void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        t.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            t.addData(""," ");
            t.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            t.addData("- Position", "%.0f / %.0f", x, y);
            t.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop

    }   // end method telemetryTfod()

    public Randomisation getLocation() {
        List<Recognition> currentRecognitions = tfod.getRecognitions();

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {

            double angle = Math.atan2((recognition.getBottom() + recognition.getTop()) / 2, (recognition.getLeft() + recognition.getRight()) / 2 - MIDDLE_X);
            t.addData("Angle", Math.toDegrees(angle));
            t.update();
            return (angle - 90 > ANGLE_THRESHOLD) ? Randomisation.LOCATION_3 : ((angle - 90 > -ANGLE_THRESHOLD) ? Randomisation.LOCATION_2 : Randomisation.LOCATION_1);

        }   // end for() loop

        visionPortal.setProcessorEnabled(tfod, false);
        return Randomisation.NONE;
    }
}
