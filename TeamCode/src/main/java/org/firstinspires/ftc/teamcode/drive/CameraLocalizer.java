package org.firstinspires.ftc.teamcode.drive;

import android.annotation.SuppressLint;
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
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

public class CameraLocalizer implements Localizer {
    public Pose2d poseEstimate;
    public Pose2d lastEstimate;
    public HardwareMap hardwareMap;
    private static double CAMERA_HEIGHT = 0.313;

    private static int SLEEP_TIME = 20;

    private static int STARTUP_TIME = 5000;

    private static float CORRECTION_FACTOR = 1;

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

    private int AVERAGE_LENGTH = 5;

    private VectorF previousPosition;

    private ArrayList<VectorF> previousPositions;

    private VectorF currentPosition;

    private VectorF currentVelocity;

    private double currentHeading;

    private ArrayList<Double> previousHeadings;

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
        this.previousPosition = new VectorF((float) startingPose.getX(), (float) startingPose.getY(), (float)CAMERA_HEIGHT);;
        this.previousPositions = new ArrayList<>();
        this.currentVelocity = new VectorF(0, 0, 0);
        this.currentHeading = startingPose.getHeading();
        this.previousHeadings = new ArrayList<>();

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
        this.previousPosition = new VectorF((float) startingPose.getX(), (float) startingPose.getY(), (float)CAMERA_HEIGHT);;
        this.previousPositions = new ArrayList<>();
        this.currentVelocity = new VectorF(0, 0, 0);
        this.currentHeading = startingPose.getHeading();
        this.previousHeadings = new ArrayList<>();

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
                .setOutputUnits(DistanceUnit.METER, AngleUnit.RADIANS)
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
    @SuppressLint("DefaultLocale")
    public Pose2d analyseDetections() {
        currentDetections = aprilTag.getDetections();
        //telemetry.addData("# AprilTags Detected", currentDetections.size());

        double heading = 0;
        int notNullTags = 0;

        ArrayList<VectorF> normals = new ArrayList<VectorF>();
        //ArrayList<VectorF> positions = new ArrayList<VectorF>();

        tagTelemetry(currentDetections, this.t);
        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {

                normals.add(vectorFromPose(detection, false));
                //positions.add(detection.metadata.fieldPosition.multiplied(FEET_TO_METERS));
                heading += yawFromPose(detection);
                notNullTags++;
            }
        }

        /*VectorF[] normalArr = new VectorF[normals.size()];
        VectorF[] posArr = new VectorF[positions.size()];

        normals.toArray(normalArr);
        positions.toArray(posArr);*/

        if (notNullTags > 0) {
            heading /= notNullTags;
            previousHeadings.add(heading);

            while (previousHeadings.size() > AVERAGE_LENGTH) {
                previousHeadings.remove(0);
            }

            currentHeading = 0;

            for (int i = 0; i < previousHeadings.size(); i++) {
                currentHeading += previousHeadings.get(i) / previousHeadings.size();
            }

            /*if (notNullTags > 1) {
                currentPosition = intersectionEstimate(normalArr, posArr);
            } else {
                currentPosition = normalArr[0].multiplied((float) firstDetection.ftcPose.range * FEET_TO_METERS);
            }*/
            VectorF tmpPosition = new VectorF(0, 0, 0);

            for (int i = 0; i < normals.size(); i++) {
                tmpPosition.add(normals.get(i));
            }
            tmpPosition.multiply(1 / (float) normals.size());
            previousPositions.add(tmpPosition);

            if (previousPositions.size() > AVERAGE_LENGTH) {
                previousPositions.remove(0);
            }

            previousPosition = currentPosition;
            currentPosition = new VectorF(0, 0,0);

            for (int j = 0; j < previousPositions.size(); j++) {
                currentPosition.add(previousPositions.get(j));
            }
            currentPosition.multiply((float) (1 / previousPositions.size()));

            currentPosition = tmpPosition;

            currentVelocity = currentPosition.subtracted(previousPosition).multiplied(1 / (float) SLEEP_TIME);
            isBlind = false;
        } else {
            // Assumes constant velocity if no April tags can be seen
            if (!isBlind) {
                blindTime = elapsedTime.time(timeUnit);
                isBlind = true;
            }
            //currentPosition = previousPosition.added(currentVelocity.multiplied(elapsedTime.time(timeUnit) - blindTime));
        }

        Pose2d currentPose = new Pose2d(currentPosition.get(0), currentPosition.get(1), currentHeading);
        //int index = 0;
        if (this.TELEMETRY_GIVEN) {
            this.t.addData("Position: ",  currentPose);
            /*this.t.addLine();
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    this.t.addLine(String.format("Id: %d, X: %.3f m, Y: %.3f m, Range: %.3f m", detection.metadata.id, normals.get(index).get(0), normals.get(index).get(1),
                            currentPosition.subtracted(detection.metadata.fieldPosition).magnitude()));
                    index++;
                }
            }*/
            this.t.update();
        }
        return currentPose;
    }

    public VectorF vectorFromPose(AprilTagDetection detection) {
        AprilTagPoseFtc pose = detection.ftcPose;

        VectorF u = new VectorF(0, -1, 0);
        VectorF v = new VectorF(1, 0, 0);

        u = detection.metadata.fieldOrientation.applyToVector(u);
        v = detection.metadata.fieldOrientation.applyToVector(v);
        VectorF w = cross(u, v);

        VectorF newNormal = u;
        newNormal = rotationAboutAxis(pose.bearing - pose.yaw, w).applyToVector(newNormal);
        newNormal = rotationAboutAxis(pose.elevation, v).applyToVector(newNormal);

        return newNormal.multiplied((float) pose.range * CORRECTION_FACTOR).added(detection.metadata.fieldPosition);
    }
    public VectorF vectorFromPose(AprilTagDetection detection, boolean normal) {
        AprilTagPoseFtc pose = detection.ftcPose;

        VectorF u = new VectorF(0, -1, 0);
        VectorF v = new VectorF(1, 0, 0);

        u = detection.metadata.fieldOrientation.applyToVector(u);
        v = detection.metadata.fieldOrientation.applyToVector(v);
        VectorF w = cross(u, v);

        VectorF newNormal = u;
        newNormal = rotationAboutAxis(pose.bearing - pose.yaw, w).applyToVector(newNormal);
        newNormal = rotationAboutAxis(pose.elevation, v).applyToVector(newNormal);
        if (normal) {
            return newNormal;
        } else {
            return newNormal.multiplied((float) pose.range * CORRECTION_FACTOR).added(detection.metadata.fieldPosition);
        }
    }

    // Assumes pitch and roll are negligible
    public double yawFromPose(AprilTagDetection detection) {
        AprilTagPoseFtc pose = detection.ftcPose;
        return mod((float) (pose.bearing + Math.acos(detection.metadata.fieldOrientation.w) * 2), (float) (2 * Math.PI));
    }

    public VectorF cross(VectorF a, VectorF b) {
        return new VectorF(a.get(1) * b.get(2) - a.get(2) * b.get(1),
                a.get(2) * b.get(0) - a.get(0) * b.get(2),
                a.get(0) * b.get(1) - a.get(1) * b.get(0));
    }

    public Quaternion rotationAboutAxis(double theta, VectorF axis) {
        VectorF normAxis = axis.multiplied(1 / axis.magnitude());
        return new Quaternion((float) Math.cos(theta / 2),
                (float) (Math.sin(theta / 2) * normAxis.get(0)),
                (float) (Math.sin(theta / 2) * normAxis.get(1)),
                (float) (Math.sin(theta / 2) * normAxis.get(2)),
                0);
    }

    @SuppressLint("DefaultLocale")
    public void tagTelemetry(List<AprilTagDetection> detections, Telemetry telemetry) {
        for (AprilTagDetection detection : detections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("ID %d", detection.id));
                telemetry.addLine(String.format("XYZ %6.3f %6.3f %6.3f  (meter)", detection.ftcPose.x * CORRECTION_FACTOR, detection.ftcPose.y * CORRECTION_FACTOR, detection.ftcPose.z * CORRECTION_FACTOR));
                telemetry.addLine(String.format("PRY %6.3f %6.3f %6.3f  (rad)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.3f %6.3f %6.3f  (meter, rad, rad)", detection.ftcPose.range * CORRECTION_FACTOR, detection.ftcPose.bearing, detection.ftcPose.elevation));
            }
            telemetry.addLine();
        }
        //telemetry.update();
    }

    // Return negatives as well, if only positive use Math.floor
    public float mod(float n, float m) {
        return (n - m * Math.round(n / m));
    }
}
