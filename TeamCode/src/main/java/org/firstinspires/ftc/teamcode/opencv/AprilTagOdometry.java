package org.firstinspires.ftc.teamcode.opencv;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.List;
import java.util.Vector;

import android.annotation.SuppressLint;
import android.util.Size;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

@Autonomous(name = "Concept: AprilTag", group = "Concept")
public class AprilTagOdometry extends LinearOpMode {
    AprilTagProcessor processor;
    TfodProcessor tfProcessor;

    VisionPortal portal;
    AprilTagLocations locations;
    int[] idList = {};
    Transform[] transformList = {};
    Transform currentTransform;

    // Processor implements the EOCV pipeline that processes the frames, the builder builds it with custom parameters,
    // and the vision portal connects that pipeline to the camera and hardware

    public void runOpMode() {
        AprilTagProcessor.Builder builder;
        VisionPortal.Builder portalBuilder = new VisionPortal.Builder();

        locations = new AprilTagLocations(idList, transformList);

        // Sets the tag library for the current season
        builder = new AprilTagProcessor.Builder();
        // TODO: This may be in inches, change to metric later
        builder.setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary());

        // Optional: set other custom features of the AprilTag Processor (4 are shown here).
        builder.setDrawTagID(true);       // Default: true, for all detections.
        builder.setDrawTagOutline(true);  // Default: true, when tag size was provided (thus eligible for pose estimation).
        builder.setDrawAxes(true);        // Default: false.
        builder.setDrawCubeProjection(true);        // Default: false.

        processor = builder.build();
        tfProcessor = TfodProcessor.easyCreateWithDefaults();

        portalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        portalBuilder.addProcessor(processor);
        portalBuilder.setCameraResolution(new Size(640, 480));  // Each resolution, for each camera model, needs calibration values for good pose estimation.
        portalBuilder.setStreamFormat(VisionPortal.StreamFormat.YUY2);  // MJPEG format uses less bandwidth than the default YUY2.
        portalBuilder.enableCameraMonitoring(true);      // Enable LiveView (RC preview).
        portalBuilder.setAutoStopLiveView(true);     // Automatically stop LiveView (RC preview) when all vision processors are disabled.

        portal = portalBuilder.build();

        telemetry.addLine("Processor and portal have been initialised");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                List<AprilTagDetection> myAprilTagDetections = processor.getDetections();
                for (AprilTagDetection detection : myAprilTagDetections) {
                    if (detection.metadata != null) {
                    }
                }
            }
        }
    }

    public Transform TagOdometry(AprilTagDetection detection) {
        Vector<Double> relPos;
        Vector<Double> relRot;
        //TODO: Calculations
    }

    @SuppressLint("DefaultLocale")
    public void TagTelemetry(AprilTagDetection detection) {
        AprilTagPoseFtc pose = detection.ftcPose;
        telemetry.addLine(String.format("X: %.2f, Y: %.2f, Z: %.2f", pose.x, pose.y, pose.z));
        telemetry.addLine(String.format("Roll: %.2f, Pitch: %.2f, Yaw: %.2f", pose.roll, pose.pitch, pose.yaw));
        telemetry.update();
    }

    // Positions are in metres, rotations are in radians
    public class Transform {
        public Vector<Double> pos;
        public Vector<Double> rot;
        public int dim;
        public Transform(Vector<Double> p, Vector<Double> r) {
            if (p.size() == r.size()) {
                this.pos = p;
                this.rot = r;
                this.dim = p.size();
            } else {
                throw new IllegalArgumentException();
            }
        }

        // Calculates Euclidean distance for any two transforms between the one which the function is
        // called on, as well as another one as an argument
        public double distance(Transform t) {
            if (this.dim != t.dim) {
                // Not of right dimensions
                return 0;
            }
            float diffSum = 0;
            for (int i = 0; i < t.dim; i++) {
                diffSum += Math.pow(this.pos.get(i) - t.pos.get(i),2);
            }
            return Math.sqrt(diffSum);
        }

        public Transform smoothAvg(Transform[] arr) {
            // TODO: Smooth out multiple April tag transforms
        }
    }

    public class AprilTagLocations {
        public int[] idArray;
        public Transform[] transformArray;
        public int l;
        public AprilTagLocations(int[] id, Transform[] transforms) {
            // Ensuring all arrays are of the same size
            if (id.length == transforms.length) {
                this.idArray = id;
                this.transformArray = transforms;
                this.l = id.length;
            } else {
                throw new IllegalArgumentException();
            }
        }

        // Returns the transform of a particular April Tag with an id
        public Transform locate(int id) {
            for (int i = 0; i < l; i++) {
                if (idArray[i] == id) {
                    return transformArray[i];
                }
            }
            return null;
        }

        // Returns the closest tag to a specified location in real space

        public int closestTag(Transform t) {
            double minDistance = 0;
            int minIndex = -1;
            for (int i = 0; i < l; i++) {
                double currentDistance = t.distance(transformArray[i]);
                if (minIndex == -1 || currentDistance < minDistance) {
                    minDistance = currentDistance;
                    minIndex = i;
                }
            }
            return idArray[minIndex];
        }
    }
}