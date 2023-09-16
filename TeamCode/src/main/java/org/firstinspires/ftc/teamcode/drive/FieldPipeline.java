package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.opencv.core.MatOfPoint;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.opencv.core.Scalar;
import org.opencv.core.CvType;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;

public class FieldPipeline extends OpenCvPipeline {

    public static Scalar[][] PIXEL_BOUNDS = {
            {new Scalar(0, 0, 0), new Scalar(0, 0, 0)},
            {new Scalar(0, 0, 0), new Scalar(0, 0, 0)},
            {new Scalar(0, 0, 0), new Scalar(0, 0, 0)}
    };

    public static String[] PIXEL_COLOURS = {
            "white",
            "purple",
            "yellow"
    };

    public static int PIXEL_THRESHOLD = 1000;
    public static int HEXAGON = 6;

    public Mat processFrame (Mat input) {
        return input;
    }

    public static Pixel[] recognisePixels(Mat input) {
        List<Pixel> currentPixels = new ArrayList<Pixel>();

        for (int i = 0; i < PIXEL_BOUNDS.length; i++) {
            Scalar[] pixelBound = PIXEL_BOUNDS[i];
            Mat masked = new Mat();
            Core.inRange(input, pixelBound[0], pixelBound[1], masked);

            List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
            Mat hierarchy = new Mat();
            // Chain approx simple because we are only searching for hexagons
            Imgproc.findContours(masked, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

            for (MatOfPoint contour : contours) {
                // If the shape is a hexagon and is big enough
                if (Imgproc.contourArea(contour) > PIXEL_THRESHOLD && contour.size(0) == HEXAGON) {
                    Moments M = Imgproc.moments(contour);
                    long cX = Math.round(M.m10 / M.m00);
                    long cY = Math.round(M.m01 / M.m00);

                    // The hexagon is considered to have a heading of 0 if it has an apex / \
                    //                                                                    | |
                    //                                                                    \ /

                    // How we calculate this is by finding the highest point in the contour,
                    // and the point 3 indices away and calculating its angle away from the vertical
                    Point[] points = contour.toArray();
                    Function<Point, Double> f = (Point p) -> {return p.y;};

                    int highest = maxOfArr(points, f);
                    int lowest = (highest + points.length / 2) % (points.length);

                    double heading = Math.atan2(points[highest].y - points[lowest].y,
                                                points[highest].x - points[lowest].x) - Math.PI / 2;

                    currentPixels.add(new Pixel(new Pose2d(cX, cY, heading), i));
                }
            }
        }

        Pixel[] tmpArray = new Pixel[currentPixels.size()];
        return currentPixels.toArray(tmpArray);
    }

    public static <T> int maxOfArr(T[] points, Function<T, Double> sort) {
        int maxIndex = -1;
        double maxValue = 0;
        for (int i = 0; i < points.length; i++) {
            double newValue = sort.apply(points[i]);
            if (maxIndex == -1 || newValue > maxValue) {
                maxValue = newValue;
                maxIndex = i;
            }
        }
        return maxIndex;
    }

    public static class Pixel {
        public Pose2d pose;
        // Refers to the index of PIXEL_BOUNDS or PIXEL_COLOURS
        public int colour;

        public Pixel(Pose2d p, int i) {
            this.pose = p;
            this.colour = i;
        }
    }
}
