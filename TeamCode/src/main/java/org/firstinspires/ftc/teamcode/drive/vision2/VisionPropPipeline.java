package org.firstinspires.ftc.teamcode.drive.vision2;

import org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.FSM_Auto_State.*;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
public class VisionPropPipeline extends OpenCvPipeline {
    private final Point region1Pos;
    private final Point region2Pos;
    private final Point region3Pos;

    private static final int REGION_WIDTH = 60;
    private static final int REGION_HEIGHT = 55;

    private static final Scalar BLUE = new Scalar(0, 0, 255);
    private static final Scalar GREEN = new Scalar(0, 255, 0);

    private final RobotAlliance alliance;
    private Rect leftRectangle;
    private Rect centerRectangle;
    private Rect rightRectangle;

    private Mat YCrCb = new Mat();
    private Mat cR = new Mat();

    private Mat leftRegion;
    private Mat centerRegion;
    private Mat rightRegion;

    private Randomization randomization = Randomization.LOCATION_2;

    public VisionPropPipeline(RobotAlliance alliance, Point region1Pos,
                              Point region2Pos,
                              Point region3Pos) {
        this.alliance = alliance;
        this.region1Pos = region1Pos;
        this.region2Pos = region2Pos;
        this.region3Pos = region3Pos;
    }

    @Override
    public void init(Mat mat) {
        leftRectangle = new Rect(
                region1Pos,
                new Point(
                        region1Pos.x + REGION_WIDTH,
                        region1Pos.y + REGION_HEIGHT));

        centerRectangle = new Rect(
                region2Pos,
                new Point(
                        region2Pos.x + REGION_WIDTH,
                        region2Pos.y + REGION_HEIGHT));

        rightRectangle = new Rect(
                region3Pos,
                new Point(
                        region3Pos.x + REGION_WIDTH,
                        region3Pos.y + REGION_HEIGHT));

        inputToCb(mat);

        leftRegion = cR.submat(leftRectangle);
        centerRegion = cR.submat(centerRectangle);
        rightRegion = cR.submat(rightRectangle);
    }

    @Override
    public Mat processFrame(Mat input) {
        Mat matToShow = cR;

        inputToCb(input);

        Imgproc.rectangle(matToShow, leftRectangle, BLUE);
        Imgproc.rectangle(matToShow, centerRectangle, BLUE);
        Imgproc.rectangle(matToShow, rightRectangle, BLUE);

        Imgproc.rectangle(input, leftRectangle, BLUE);
        Imgproc.rectangle(input, centerRectangle, BLUE);
        Imgproc.rectangle(input, rightRectangle, BLUE);

        double leftAverage = Core.mean(leftRegion).val[0];
        double centerAverage = Core.mean(centerRegion).val[0];
        double rightAverage = Core.mean(rightRegion).val[0];

        double min = Math.min(leftAverage, Math.min(centerAverage, rightAverage));

        if (min == leftAverage) {
            randomization = Randomization.LOCATION_1;
        }
        else if (min == centerAverage) {
            randomization = Randomization.LOCATION_2;
        }
        else if (min == rightAverage) {
            randomization = Randomization.LOCATION_3;
        }

        Imgproc.putText(input, "" + randomization, new Point(0, 50), Imgproc.FONT_HERSHEY_PLAIN, 2.0, GREEN);

        //return matToShow;
        return input;
    }

    public Randomization getRandomization() {
        return randomization;
    }

    /*
     * This function takes the RGB frame, converts to YCrCb,
     * and extracts the Cb channel to the 'cB' variable
     */
    private void inputToCb(Mat input) {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb, cR, this.alliance == RobotAlliance.RED ? 2 : 1);
    }

    public enum Randomization {
        LOCATION_1,
        LOCATION_2,
        LOCATION_3
    }
}