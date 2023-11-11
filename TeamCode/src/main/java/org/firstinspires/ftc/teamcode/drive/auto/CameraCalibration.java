package org.firstinspires.ftc.teamcode.drive.auto;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.Robotv8.AutoBase;
import org.firstinspires.ftc.teamcode.drive.Robotv8.Fullstack;
import org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotAlliance;
import org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotStartingPosition;
import org.firstinspires.ftc.teamcode.drive.vision2.PropPipeline;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
@Autonomous(name="CameraCalibration", group="Calibration")
public class CameraCalibration extends Fullstack {
    private Point r1;
    private Point r2;
    private Point r3;
    public RobotAlliance alliance = RobotAlliance.RED;
    public RobotStartingPosition startingPosition = RobotStartingPosition.BACKDROP;
    private PropPipeline.Randomization randomization;
    private final ElapsedTime autoTimer = new ElapsedTime();

    public CameraCalibration(Point r1, Point r2, Point r3) {
        this.startingPosition = RobotStartingPosition.BACKDROP;
        this.alliance = RobotAlliance.RED;
        this.r1 = r1;
        this.r2 = r2;
        this.r3 = r3;
    }

    public void MainInit() {
        OpenCvWebcam webcam;
        PropPipeline pipeline = new PropPipeline( alliance, r1, r2, r3 );

        @SuppressLint("DiscouragedApi") int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId",
                "id",
                hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(pipeline);

        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(webcam, 20);
            }

            @Override
            public void onError(int errorCode) {
                // intentional noop
                telemetry.addLine("error, you stupid idiot");
            }
        });

        autoTimer.reset();

        while(autoTimer.seconds() < 10) {
            randomization = pipeline.getRandomization();
            telemetry.addData("TEAM_PROP_LOCATION", randomization);
            telemetry.update();
        }

        webcam.closeCameraDevice();
    }
}