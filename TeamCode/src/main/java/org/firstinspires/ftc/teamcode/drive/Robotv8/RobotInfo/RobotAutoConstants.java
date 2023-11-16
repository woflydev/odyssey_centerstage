package org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class RobotAutoConstants {
    // -------------------------------------------------------------- AUTO CONFIG
    public static final double INCHES_PER_TILE = 24;

    public static final Pose2d[] RED_STARTING_POSES = {
            new Pose2d(11, -63, Math.toRadians(90)),
            new Pose2d(-35, -63, Math.toRadians(90)),
    };
    public static final Pose2d[] BLUE_STARTING_POSES = {
            new Pose2d(11.5, 63, Math.toRadians(270)),
            new Pose2d(-35, 63, Math.toRadians(270)),
    };
    public static final Pose2d[] RED_PARKING_POSES = {
            // note: inner is first
            new Pose2d(50, -8.5, Math.toRadians(90)),
            new Pose2d(50, -53.5, Math.toRadians(90)),
    };
    public static final Pose2d[] BLUE_PARKING_POSES = {
            new Pose2d(50, 8.5, Math.toRadians(270)),
            new Pose2d(50, 53.5, Math.toRadians(270))
    };
    public static final Pose2d[] BACKBOARD_CENTER_POSES = {
            // note: red is first
            new Pose2d(48.5, -36.5, Math.toRadians(180)),
            new Pose2d(48.5, 36.5, Math.toRadians(180)),
    };
    public static final double[] YELLOW_PIXEL_VARIANCE = {
            1,
            1.2,
            1.4,
    };
    public static final double[] PURPLE_PIXEL_VARIANCE = {
            1.3,
            0.8,
            0.3,
    };
}
