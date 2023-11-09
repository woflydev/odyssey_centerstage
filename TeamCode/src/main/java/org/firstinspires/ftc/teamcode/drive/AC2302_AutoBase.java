package org.firstinspires.ftc.teamcode.drive;

import static org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotConstants.ENCODER_TICKS_PER_TILE;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.analysis.function.Exp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.Robotv8.FSM_Fullstack;
import org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.FSM_Outtake;
import org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotAlliance;
import org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotConstants;
import org.firstinspires.ftc.teamcode.drive.vision2.PropPipeline;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

//@Autonomous(name="NationalsAutoBase", group="Final")
public class AC2302_AutoBase extends FSM_Fullstack {
    private PropPipeline.Randomization randomization;
    private final ElapsedTime autoTimer = new ElapsedTime();
    private RobotAlliance alliance;
    private final int dir = alliance == RobotAlliance.RED ? 1 : -1;
    private Point r1;
    private Point r2;
    private Point r3;
    private final PropPipeline pipeline = new PropPipeline( alliance, r1, r2, r3 );

    // note: custom behaviour -----------------------------------------------------------
    public AC2302_AutoBase(RobotAlliance alliance, Point r1, Point r2, Point r3) {
        this.alliance = alliance;
        this.r1 = r1;
        this.r2 = r2;
        this.r3 = r3;
    }

    public void MainInit() {
        BackboardToPixels(); // note: testing smooth spline
        Delay(5000);

        OpenCvWebcam webcam;

        @SuppressLint("DiscouragedApi")
        int cameraMonitorViewId = hardwareMap
            .appContext
            .getResources()
            .getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()
            );

        webcam = OpenCvCameraFactory
                .getInstance()
                .createWebcam(
                        hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId
                );

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
            }
        });

        autoTimer.reset();
        while (autoTimer.seconds() < 5) {
            randomization = pipeline.getRandomization();
            telemetry.addData("Randomization", randomization);
            telemetry.update();
        }
        webcam.closeCameraDevice();
    }

    public void MainStart() {
        randomization = pipeline.getRandomization();

        // note: drop off at correct spikemark
        switch(randomization) {
            case LOCATION_1: // note: left
                VisualMove(0.8, 1, 1, false, false, 3);
                AutoWait();
                VisualMove(0.5, -dir, dir, false, false, 3);
                AutoWait();
                ExpelPixel();
                AutoWait();
                break;
            case LOCATION_2: // note： forward
                VisualMove(0.8, 1.2, 1.2, false, false, 3);
                AutoWait();
                ExpelPixel();
                AutoWait();
                VisualMove(0.5, -0.3, -0.3, false, false, 3);
                AutoWait();
                VisualMove(0.6, -dir, dir, false, false, 3); // note: turn 90 deg on same tile.
                break;
            case LOCATION_3: // note: rights
                VisualMove(0.8, 1, 1, false, false, 3);
                AutoWait();
                VisualMove(0.5, dir, -dir, false, false, 3);
                AutoWait();
                ExpelPixel();
                AutoWait();
                VisualMove(0.5, -2 * dir, 2 * dir, false, false, 3);
                AutoWait();
                break;
            default:
                break;
        }

        VisualMove(0.6, -1.54, -1.54, false, false, 3);

        // note: drop off at correct april tag
        switch(randomization) {
            case LOCATION_1: // note: left
                break;
            case LOCATION_2: // note： forward
                VisualMove(0.5, 0.3, 0.3, false, false, 3);
                AutoWait();
                ExpelPixel();
                AutoWait();
                VisualMove(0.5, -0.3, -0.3, false, false, 3);
                AutoWait();
                VisualMove(0.6, -dir, dir, false, false, 3); // note: turn 90 deg on same tile.
                break;
            case LOCATION_3: // note: right
                break;
            default:
                break;
        }

        GrabAndReady();
        Delay(1000);
        RaiseAndPrime(300);
        Delay(1000);
        DropAndReset();

        VisualMove(0.7, 0.3, 0.3, false, false, 3);
        //BackboardToParking();
    }

    // note: sequenced movement  --------------------------------------------------------
    private void BackboardToParking() {
        VisualMove(0.7, 1, 1, true, false, 5); // note: strafe
        VisualMove(0.5, dir, -dir, false, false, 3);
        AutoWait();
        VisualMove(0.5, 0.2, 0.2, true, true, 3);
    }

    private void BackboardToPixels() {
        VisualMove(0.6, 1, 0.5, false, false, 3);
        VisualMove(0.6, 0.5, 1, false, false, 3);
        AutoWait();
    }

    // note: helper functions -----------------------------------------------------------
    private void ExpelPixel() {
        intake.setPower(-0.4);
        Delay(2000);
        intake.setPower(0);
    }

    public void GrabAndReady() {
        MoveElbow(RobotConstants.ELBOW_STANDBY);
        servoWrist.setPosition(RobotConstants.WRIST_STANDBY);
        Delay(500);

        servoFlap.setPosition(RobotConstants.FLAP_OPEN);
        Delay(700);

        // transfer stage sequence
        servoWrist.setPosition(RobotConstants.WRIST_PICKUP);
        Delay(200);
        MoveElbow(RobotConstants.ELBOW_STANDBY); // moves it up a little to avoid tubes
        Delay(200);
        MoveElbow(RobotConstants.ELBOW_PICKUP);

        Delay(200);
        servoClaw.setPosition(RobotConstants.CLAW_CLOSE);
        Delay(500);

        // primes the elbow
        MoveElbow(RobotConstants.ELBOW_STANDBY);
        Delay(100);
        servoWrist.setPosition(RobotConstants.WRIST_STANDBY);
    }

    public void RaiseAndPrime(int height) {
        intake.setPower(0); // make sure intake is not running

        targetOuttakePosition = height;
        UpdateOuttake(false, 0);

        servoFlap.setPosition(RobotConstants.FLAP_CLOSE);
        servoClaw.setPosition(RobotConstants.CLAW_CLOSE);
        servoWrist.setPosition(RobotConstants.WRIST_ACTIVE);

        MoveElbow(RobotConstants.ELBOW_ACTIVE);

        outtakeState = FSM_Outtake.PRIMED_FOR_DEPOSIT;
        Delay(50); // debounce
    }

    public void DropAndReset() {
        servoFlap.setPosition(RobotConstants.FLAP_CLOSE);
        servoClaw.setPosition(RobotConstants.CLAW_OPEN);
        Delay(800); // wait for claw to open

        servoWrist.setPosition(RobotConstants.WRIST_STANDBY);
        MoveElbow(RobotConstants.ELBOW_STANDBY);

        Delay(350); // elbow should come down after the slide is near done

        targetOuttakePosition = 10;
        UpdateOuttake(true, 0);
    }

    private double TilesToTicks(double input) {
        return ENCODER_TICKS_PER_TILE * input;
    }

    private void VisualMove(double power, double left, double right, boolean strafe, boolean strafeRight, double safetyTimeout) {

        int backLMTarget;
        int frontLMTarget;
        int backRMTarget;
        int frontRMTarget;

        if (!strafe) {
            backLMTarget = backLM.getCurrentPosition() - (int)(TilesToTicks(left));
            frontLMTarget = frontLM.getCurrentPosition() - (int)(TilesToTicks(left));
            backRMTarget = backRM.getCurrentPosition() - (int)(TilesToTicks(right));
            frontRMTarget = frontRM.getCurrentPosition() - (int)(TilesToTicks(right));
        }

        else {
            int dir = strafeRight ? 1 : -1;
            dir *= alliance == RobotAlliance.RED ? 1 : -1;
            backLMTarget = backLM.getCurrentPosition() + (int)(TilesToTicks(left) * dir);
            frontLMTarget = frontLM.getCurrentPosition() - (int)(TilesToTicks(left) * dir);
            backRMTarget = backRM.getCurrentPosition() - (int)(TilesToTicks(right) * dir);
            frontRMTarget = frontRM.getCurrentPosition() + (int)(TilesToTicks(right) * dir);
        }

        backLM.setTargetPosition(backLMTarget);
        frontLM.setTargetPosition(frontLMTarget);
        backRM.setTargetPosition(backRMTarget);
        frontRM.setTargetPosition(frontRMTarget);

        backLM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRM.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        backLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        encoderRuntime.reset();
        backLM.setPower(Math.abs(power));
        frontLM.setPower(Math.abs(power));
        backRM.setPower(Math.abs(power));
        frontRM.setPower(Math.abs(power));

        while ((encoderRuntime.seconds() <= safetyTimeout) && (backRM.isBusy() && backLM.isBusy())) {
            telemetry.clear();
            telemetry.addData("CURRENT COORDINATE: ",  "%7d :%7d", backLM.getCurrentPosition(), backRM.getCurrentPosition());
            telemetry.update();
        }

        backLM.setPower(0);
        frontLM.setPower(0);
        backRM.setPower(0);
        frontRM.setPower(0);

        backLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /*backLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/
    }

    private void AutoWait() {
        Delay(200);
    }
}
