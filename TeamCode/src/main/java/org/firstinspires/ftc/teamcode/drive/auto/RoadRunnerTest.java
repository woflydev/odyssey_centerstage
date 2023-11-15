package org.firstinspires.ftc.teamcode.drive.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.AutoMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotConstants;
import org.firstinspires.ftc.teamcode.drive.vision.CameraLocalizer;

import java.util.function.Function;


@Autonomous(name="Road Runner Test", group="Test")

public class RoadRunnerTest extends LinearOpMode {
    private AutoMecanumDrive drive;

    public static Pose2d[] RED_STARTING_POSES = {new Pose2d(0.29, -1.565, Math.PI / 2).times(RobotConstants.ROAD_RUNNER_SCALE),
            new Pose2d(-0.9, -1.565, Math.PI / 2).times(RobotConstants.ROAD_RUNNER_SCALE)};
    public static Pose2d[] BLUE_STARTING_POSES = {new Pose2d(0.29, 1.565, 3 * Math.PI / 2).times(RobotConstants.ROAD_RUNNER_SCALE),
            new Pose2d(-0.9, 1.565, 3* Math.PI / 2).times(RobotConstants.ROAD_RUNNER_SCALE)};
    public void runOpMode() {
        drive = new AutoMecanumDrive(hardwareMap, RED_STARTING_POSES[1], telemetry);
        telemetry.addLine("Drive initialised!");
        telemetry.update();
        waitForStart();
        telemetry.addLine("Beginning trajectory!");
        telemetry.update();
        drive.setPoseEstimate(RED_STARTING_POSES[1]);
        drive.followTrajectory(path(RED_STARTING_POSES[1], new Pose2d(-49.26, 59.18, Math.toRadians(135))));
        telemetry.addLine("Finished trajectory!");
        telemetry.update();
    }
    public Trajectory path(Pose2d start, Pose2d end) {
        // Same side of the truss
        if (start.getX() * end.getX() >= 0) {
            return drive.trajectoryBuilder(start)
                    .splineTo(end.vec(), end.getHeading())
                    .build();
        } else {
            double avgY = (start.getY() + end.getY()) / 2;
            Double[] diffY = RobotConstants.PATH_Y;
            for (int i = 0; i < diffY.length; i++) {
                diffY[i] = Math.abs(diffY[i] - avgY);
            }

            Function<Double, Double> cmpFn = (Double x) -> x;

            double pathValue = RobotConstants.PATH_Y[CameraLocalizer.maxOfArr(diffY, cmpFn, false)];
            return drive.trajectoryBuilder(start)
                    .splineTo(new Vector2d(((start.getX() >= 0) ? 1 : -1) * RobotConstants.TRUSS_WIDTH / 2, pathValue),
                            ((start.getX() >= 0) ? 1 : -1) * RobotConstants.HEADING)
                    .splineTo(new Vector2d(((start.getX() >= 0) ? -1 : 1) * RobotConstants.TRUSS_WIDTH / 2, pathValue),
                            ((start.getX() >= 0) ? 1 : -1) * RobotConstants.HEADING)
                    .splineTo(end.vec(), end.getHeading())
                    .build();
        }
    }

    public Trajectory path(Pose2d[] points) {
        TrajectoryBuilder currentTrajectory = drive.trajectoryBuilder(points[0]);
        for (int i = 0; i < points.length - 1; i++) {
            Pose2d start = points[i];
            Pose2d end = points[i + 1];
            // Same side of the truss
            if (start.getX() * end.getX() >= 0) {
                currentTrajectory = currentTrajectory.splineTo(end.vec(), end.getHeading());
            } else {
                double avgY = (start.getY() + end.getY()) / 2;
                Double[] diffY = RobotConstants.PATH_Y;
                for (int j = 0; j < diffY.length; j++) {
                    diffY[j] = Math.abs(diffY[j] - avgY);
                }

                Function<Double, Double> cmpFn = (Double x) -> x;

                double pathValue = RobotConstants.PATH_Y[CameraLocalizer.maxOfArr(diffY, cmpFn, false)];
                currentTrajectory = currentTrajectory
                        .splineTo(new Vector2d(((start.getX() >= 0) ? 1 : -1) * RobotConstants.TRUSS_WIDTH / 2, pathValue),
                                ((start.getX() >= 0) ? 1 : -1) * RobotConstants.HEADING)
                        .splineTo(new Vector2d(((start.getX() >= 0) ? -1 : 1) * RobotConstants.TRUSS_WIDTH / 2, pathValue),
                                ((start.getX() >= 0) ? 1 : -1) * RobotConstants.HEADING)
                        .splineTo(end.vec(), end.getHeading());
            }

        }
        return currentTrajectory.build();
    }
}
