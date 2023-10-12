package org.firstinspires.ftc.teamcode.drive;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class CameraLocalizer implements Localizer {
    public Pose2d poseEstimate;
    public Pose2d lastEstimate;
    public HardwareMap hardwareMap;
    private static double CAMERA_HEIGHT = 0.313;

    private static int SLEEP_TIME = 20;

    private static int STARTUP_TIME = 5000;

    private String FRONT_CAMERA;
    private String BACK_CAMERA;

    private ElapsedTime elapsedTime = new ElapsedTime();
    private TimeUnit timeUnit = TimeUnit.MILLISECONDS;

    /**
     * {@link #aprilTag} is the variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * {@link #visionPortal} is the variable to store our instance of the vision portal.
     */
    public VisionPortal visionPortal;

    // This assumes the april tag starts facing along the y-axis, may change later
    private AprilTagMetadata[] tagArray;

    private VectorF previousPosition;

    private VectorF currentPosition;

    private VectorF currentVelocity;

    private double currentHeading;

    public List<AprilTagDetection> currentDetections;

    private long blindTime = 0;
    private boolean isBlind = false;

    private Telemetry t;
    private boolean TELEMETRY_GIVEN;

    public Pose2d getPoseEstimate() {
        return this.poseEstimate;
    }

    public void setPoseEstimate(Pose2d newPose) {
        this.poseEstimate = newPose;
    }

    public Pose2d getPoseVelocity() {
        return poseEstimate.minus(lastEstimate).div(SLEEP_TIME);
    }

    public CameraLocalizer(HardwareMap map, String front, String back, Pose2d startingPose, AprilTagMetadata[] tagArray) {
        this.hardwareMap = map;
        this.poseEstimate = startingPose;
        this.lastEstimate = startingPose;
        this.tagArray = tagArray;
        this.TELEMETRY_GIVEN = false;

        this.currentPosition = new VectorF((float) startingPose.getX(), (float) startingPose.getY(), (float)CAMERA_HEIGHT);
        this.previousPosition = new VectorF((float) startingPose.getX(), (float) startingPose.getY(), (float)CAMERA_HEIGHT);
        this.currentVelocity = new VectorF(0, 0, 0);
        this.currentHeading = startingPose.getHeading();

        this.FRONT_CAMERA = front;
        this.BACK_CAMERA = back;

        initAprilTag();
        while (elapsedTime.time() < STARTUP_TIME) {
            analyseDetections();
        }
    }

    public CameraLocalizer(HardwareMap map, String front, String back, Pose2d startingPose, AprilTagMetadata[] tagArray, Telemetry t) {
        this.hardwareMap = map;
        this.poseEstimate = startingPose;
        this.lastEstimate = startingPose;
        this.tagArray = tagArray;
        this.t = t;
        this.TELEMETRY_GIVEN = true;

        this.currentPosition = new VectorF((float) startingPose.getX(), (float) startingPose.getY(), (float)CAMERA_HEIGHT);
        this.previousPosition = new VectorF((float) startingPose.getX(), (float) startingPose.getY(), (float)CAMERA_HEIGHT);
        this.currentVelocity = new VectorF(0, 0, 0);
        this.currentHeading = startingPose.getHeading();

        this.FRONT_CAMERA = front;
        this.BACK_CAMERA = back;

        initAprilTag();
        while (elapsedTime.time() < STARTUP_TIME) {
            analyseDetections();
        }
    }

    public void update() {
        lastEstimate = poseEstimate;
        poseEstimate = analyseDetections();
    }

    private void initAprilTag() {

        AprilTagLibrary.Builder b = new AprilTagLibrary.Builder();
        for (AprilTagMetadata tag : tagArray) {
            b.addTag(tag);
        }

        AprilTagLibrary library = b.build();

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(library)
                .setOutputUnits(DistanceUnit.METER, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                .setLensIntrinsics(1389.80870649, 1389.80870649, 663.268596171, 399.045042197)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        builder.setCamera(hardwareMap.get(WebcamName.class, FRONT_CAMERA));

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(1280, 720));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableCameraMonitoring(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();
        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }
    public Pose2d analyseDetections() {
        currentDetections = aprilTag.getDetections();
        //telemetry.addData("# AprilTags Detected", currentDetections.size());

        VectorF smoothPos = new VectorF(0, 0, 0);
        double smoothHeading = 0;
        int notNullTags = 0;

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                smoothPos.add(vectorFromPose(detection));
                smoothHeading += yawFromPose(detection);
                notNullTags++;
            }
        }
        previousPosition = currentPosition;

        if (notNullTags > 0) {

            currentPosition = smoothPos.multiplied(1 / (float) notNullTags);
            currentVelocity = currentPosition.subtracted(previousPosition).multiplied(1 / (float) SLEEP_TIME);
            currentHeading = (smoothHeading / notNullTags) % (2 * Math.PI);
            isBlind = false;
        } else {
            // Assumes constant velocity if no April tags can be seen
            if (!isBlind) {
                blindTime = elapsedTime.time(timeUnit);
                isBlind = true;
            }
            //currentPosition = previousPosition.added(currentVelocity.multiplied(elapsedTime.time(timeUnit) - blindTime));
        }
        if (this.TELEMETRY_GIVEN) this.t.addData("Position: ", new Pose2d(currentPosition.get(0), currentPosition.get(1), currentHeading));
        this.t.update();
        return new Pose2d(currentPosition.get(0), currentPosition.get(1), currentHeading);
    }

    public VectorF vectorFromPose(AprilTagDetection detection) {
        AprilTagPoseFtc pose = detection.ftcPose;
        MatrixF rotationM = rotationMatrix(pose.pitch + pose.elevation, 0, 0)
                .multiplied(rotationMatrix(0, 0, pose.bearing - pose.yaw));
        VectorF relativeVector = rotationM.multiplied(new VectorF(0, 1, 0));
        MatrixF fieldRotation = detection.metadata.fieldOrientation.toMatrix();
        //if (TELEMETRY_GIVEN) this.t.addLine(fieldRotation.toString());
        //this.t.update();
        return fieldRotation.slice(3, 3).multiplied(relativeVector).added(detection.metadata.fieldPosition);
    }

    // Assumes pitch and roll are negligible
    public double yawFromPose(AprilTagDetection detection) {
        AprilTagPoseFtc pose = detection.ftcPose;
        return pose.bearing - pose.yaw + Math.acos(detection.metadata.fieldOrientation.w);
    }

    // All are in radians
    public MatrixF rotationMatrix(double pitch, double roll, double yaw) {
        MatrixF pitchM = MatrixF.identityMatrix(3);
        MatrixF rollM = MatrixF.identityMatrix(3);
        MatrixF yawM = MatrixF.identityMatrix(3);

        for (int i = 0; i < 3; i++) {
            if (i != 0) {
                pitchM.put(i, i, (float) Math.cos(pitch));
                pitchM.put(3 - i,i, (float) Math.sin(pitch) * (i % 2 == 0 ? 1 : -1));
            }
            if (i != 1) {
                rollM.put(i, i, (float) Math.cos(roll));
                rollM.put(2 - i, i, (float) Math.sin(pitch) * (i == 2 ? 1 : -1));
            }
            if (i != 2) {
                yawM.put(i, i, (float) Math.cos(yaw));
                yawM.put(1 - i, i, (float) Math.sin(pitch) * (i == 0 ? 1 : -1));
            }
        }

        // Euler angles, pitch, then roll than yaw
        return yawM.multiplied(rollM.multiplied(pitchM));
    }
}
