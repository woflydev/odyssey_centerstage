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
            new Pose2d(45, -15, Math.toRadians(90)),
            new Pose2d(45, -53.5, Math.toRadians(90)),
    };
    public static final Pose2d[] BLUE_PARKING_POSES = {
            new Pose2d(45, 15, Math.toRadians(270)),
            new Pose2d(45, 53.5, Math.toRadians(270))
    };
    public static final Pose2d[] SPIKEMARK_CENTER_POSES = {
            // note: red is first
            new Pose2d(48.5, -30, Math.toRadians(180)),
            new Pose2d(48.5, 30, Math.toRadians(180)),
    };
    public static final double[] BACKDROP_YELLOW_PIXEL_VARIANCE = {
            1,
            1.2,
            1.4,
    };
    public static final double[] BACKDROP_PURPLE_PIXEL_VARIANCE = {
            0.35,
            0.8,
            1.4,
    };
    public static final double[] AUDIENCE_PURPLE_PIXEL_VARIANCE = {
            2.2,
            2.2,
            2.2,
    };
    public static final double CENTER_SPIKEMARK_ALIGN_TURN = 20;
    public static final double SPIKE_TO_BACKBOARD_TRANSIT = 1.55;
}
