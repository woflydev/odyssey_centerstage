package org.firstinspires.ftc.teamcode.drive.Robotv8.testing;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.Robotv8.Robotv8_Fullstack;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp
public class Webcam_Test extends OpMode
{
    OpenCvWebcam webcamFront;

    @SuppressLint("DefaultLocale")
    @Override
    public void init()
    {
        /*
         * Instantiate an OpenCvCamera object for the camera we'll be using.
         * In this sample, we're using a webcam. Note that you will need to
         * make sure you have added the webcam to your configuration file and
         * adjusted the name here to match what you named it in said config file.
         *
         * We pass it the view that we wish to use for camera monitor (on
         * the RC phone). If no camera monitor is desired, use the alternate
         * single-parameter constructor instead (commented out below)
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcamFront = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcamFront.setPipeline(new SamplePipeline());
        webcamFront.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
        webcamFront.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                telemetry.addLine("Opened!");
                telemetry.update();
                webcamFront.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
                telemetry.addData("pain", "pain");
                telemetry.update();
            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();

    }

    public void loop() {
        telemetry.addData("Frame Count", webcamFront.getFrameCount());
        telemetry.addData("FPS", String.format("%.2f", webcamFront.getFps()));
        telemetry.addData("Total frame time ms", webcamFront.getTotalFrameTimeMs());
        telemetry.addData("Pipeline time ms", webcamFront.getPipelineTimeMs());
        telemetry.addData("Overhead time ms", webcamFront.getOverheadTimeMs());
        telemetry.addData("Theoretical max FPS", webcamFront.getCurrentPipelineMaxFps());
        telemetry.update();

        if(gamepad1.a)
        {
            /*
             * IMPORTANT NOTE: calling stopStreaming() will indeed stop the stream of images
             * from the camera (and, by extension, stop calling your vision pipeline). HOWEVER,
             * if the reason you wish to stop the stream early is to switch use of the camera
             * over to, say, Vuforia or TFOD, you will also need to call closeCameraDevice()
             * (commented out below), because according to the Android Camera API documentation:
             *         "Your application should only have one Camera object active at a time for
             *          a particular hardware camera."
             *
             * NB: calling closeCameraDevice() will internally call stopStreaming() if applicable,
             * but it doesn't hurt to call it anyway, if for no other reason than clarity.
             *
             * NB2: if you are stopping the camera stream to simply save some processing power
             * (or battery power) for a short while when you do not need your vision pipeline,
             * it is recommended to NOT call closeCameraDevice() as you will then need to re-open
             * it the next time you wish to activate your vision pipeline, which can take a bit of
             * time. Of course, this comment is irrelevant in light of the use case described in
             * the above "important note".
             */
            webcamFront.stopStreaming();
            //webcam.closeCameraDevice();
        }
    }

    class SamplePipeline extends OpenCvPipeline
    {
        boolean viewportPaused;

        @Override
        public Mat processFrame(Mat input)
        {
            Imgproc.rectangle(
                    input,
                    new Point(
                            input.cols()/4,
                            input.rows()/4),
                    new Point(
                            input.cols()*(3f/4f),
                            input.rows()*(3f/4f)),
                    new Scalar(0, 255, 0), 4);
            return input;
        }

        @Override
        public void onViewportTapped()
        {
            viewportPaused = !viewportPaused;

            if(viewportPaused)
            {
                webcamFront.pauseViewport();
            }
            else
            {
                webcamFront.resumeViewport();
            }
        }
    }
}