package org.firstinspires.ftc.teamcode.drive.vision;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.opencv.core.MatOfPoint;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.opencv.core.Scalar;
import org.opencv.core.Point;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;

public class FieldPipeline extends OpenCvPipeline {

    public static Scalar[][] PIXEL_BOUNDS = {
            {new Scalar(0, 0, 248), new Scalar(179, 17, 255)},
            {new Scalar(44, 64, 166), new Scalar(106, 252, 255)},
            {new Scalar(112, 55, 189), new Scalar(128, 252, 255)},
            {new Scalar(17, 91, 0), new Scalar(236, 30, 255)}
    };

    public static String[] PIXEL_COLOURS = {
            "white",
            "green",
            "purple",
            "yellow"
    };

    // Blue prop
    public static Scalar[] TEAM_PROP_BOUNDS = {new Scalar(97, 85, 107), new Scalar(110, 190, 206)};
    public static long PROP_THRESHOLD = 1000;
    public static double ANGLE_THRESHOLD = Math.toRadians(15);

    public static int PIXEL_THRESHOLD = 1000;
    // Distance from opposite edges of the pixel in metres
    public static double PIXEL_EDGE_TO_EDGE = 0.0762;
    public static double PIXEL_CORNER_TO_CORNER = PIXEL_EDGE_TO_EDGE * 2 / Math.sqrt(3);
    public static double PIXEL_SIDE_LENGTH = PIXEL_CORNER_TO_CORNER / 2;

    public static int SCORE_PER_BACKDROP_PIXEL = 5;

    public static int ROWS_PER_LINE = 3;
    public static int ROWS_FOR_FIRST = 3;
    public static int SCORE_PER_LINE = 10;
    public static int MAXIMUM_LINE_SCORE = 30;

    public static int SCORE_PER_MOSAIC = 10;

    public static int HEXAGON = 6;

    public static int SPIKE_BACKDROP_COLOUR = 2;
    // This assumes team prop, use 10 if using white pixel
    public static int SCORE_SPIKE_BACKDROP = 20;

    public static int APRIL_TAG_BACKDROP_SPACING = 2;
    public static int FIRST_APRIL_TAG_LOCATION = 0;

    public static int SCREEN_WIDTH = 1080;
    public static int SCREEN_HEIGHT = 720;

    public static float BACKDROP_Z_OFFSET = 0.18f;
    public static float PIXEL_HEIGHT = 0.1f;

    public Pixel.Backdrop backdrop = null;

    public int spikeMark;

    public int mode;

    public FieldPipeline(int mode) {
        this.mode = mode;
    }

    public Mat processFrame (Mat input) {
        spikeMark = propLocation(input);
        backdrop = new Pixel.Backdrop(recognisePixels(input));
        return input;
    }

    public static int propLocation(Mat input) {
        Mat masked = new Mat();
        Core.inRange(input, TEAM_PROP_BOUNDS[0], TEAM_PROP_BOUNDS[1], masked);
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(masked, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        Function<MatOfPoint, Double> sort = Imgproc::contourArea;
        MatOfPoint[] contourArr = new MatOfPoint[contours.size()];
        contours.toArray(contourArr);

        int maxIndex = maxOfArr(contourArr, sort);

        if (Imgproc.contourArea(contourArr[maxIndex]) > PROP_THRESHOLD) {
            Moments M = Imgproc.moments(contourArr[maxIndex]);

            long cX = Math.round(M.m10 / M.m00);
            long cY = Math.round(M.m01 / M.m00);

            double angle = Math.atan2(cY, cX - SCREEN_WIDTH / 2) - Math.PI / 2;
            if (Math.abs(angle) > ANGLE_THRESHOLD) {
                return angle > 0 ? 5 : 1;
            }
            return 3;
        }
        // Prop not found
        return -1;
    }

    // Given a backdrop image, this function approximates the score that
    public static int scoreOnBackdrop(Mat input) {
        Pixel[] pixels = recognisePixels(input);
        Pixel.Backdrop backdrop = new Pixel.Backdrop(pixels);
        return backdrop.score();
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
                double contourArea = Imgproc.contourArea(contour);
                // If the shape is a hexagon and is big enough
                if (contourArea > PIXEL_THRESHOLD && contour.size(0) == HEXAGON) {
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

                    double estimatedSideLength = Math.sqrt(2 * contourArea / HEXAGON / Math.tan(2 * Math.PI / HEXAGON));

                    currentPixels.add(new Pixel(new Pose2d(cX, cY, heading), i, estimatedSideLength));
                }
            }
        }

        Pixel[] tmpArray = new Pixel[currentPixels.size()];
        return currentPixels.toArray(tmpArray);
    }

    // Finds the index of the element with the highest sort value
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

    // Finds the index of the element with the highest sort value, unless reverse is true, in which case
    // it does the opposite
    public static <T> int maxOfArr(T[] points, Function<T, Double> sort, boolean reverse) {
        int maxIndex = -1;
        double maxValue = 0;
        for (int i = 0; i < points.length; i++) {
            double newValue = sort.apply(points[i]);
            if (maxIndex == -1 || ((newValue > maxValue && !reverse) ||(newValue < maxValue && reverse))) {
                maxValue = newValue;
                maxIndex = i;
            }
        }
        return maxIndex;
    }

    public static class Pixel {
        public Pose2d pose;
        public double sideLength;
        // Refers to the index of PIXEL_BOUNDS or PIXEL_COLOURS
        public int colour;

        public Pixel(Pose2d p, int i, double s) {
            this.pose = p;
            this.colour = i;
            this.sideLength = s;
        }

        public static class Backdrop {
            public ArrayList<Pixel>[] pixels;
            public int pixelNum;
            public int rows;
            // This is in pixels
            public double rowHeight;
            // Creates a backdrop object from a list of pixels
            // Note, the height difference in each of the rows of pixels is 1.5 * sidelength
            // The sidelength can be estimated from the average contour area of the pixels and
            // assuming it is a perfect hexagon
            public Backdrop(Pixel[] p) {

                // The number of rows must be given because the pixels' positions on the camera changes with distance
                Function<Pixel, Double> height = (Pixel a) -> {return a.pose.getY();};

                int highestIndex = maxOfArr(p, height, false);
                int lowestIndex = maxOfArr(p, height, false);

                double highY = p[highestIndex].pose.getY();
                double lowY = p[lowestIndex].pose.getY();

                double averageSideLength = 0;
                for (Pixel pixel : p) {
                    averageSideLength += pixel.sideLength / p.length;
                }

                this.rows = (int) Math.floor((highY - lowY) / averageSideLength);
                this.rowHeight = (highY - lowY) / this.rows;
                this.pixelNum = p.length;

                pixels = new ArrayList[rows];

                for (Pixel pixel : p) {
                    int row = (int) Math.floor((pixel.pose.getY() - lowY) / this.rowHeight);
                    this.pixels[row].add(pixel);
                }

                for (int i = 0; i < rows; i++) {
                    this.pixels[i].sort((Pixel a, Pixel b) -> (int) Math.round(a.pose.getX() - b.pose.getX()));
                }
            }
            public int score() {
                int baseScore = SCORE_PER_BACKDROP_PIXEL * this.pixelNum;
                // Change this later because we need more information
                int bonusFromRandomisation = 0;

                return baseScore + bonusFromRandomisation;
            }
        }
    }
}
