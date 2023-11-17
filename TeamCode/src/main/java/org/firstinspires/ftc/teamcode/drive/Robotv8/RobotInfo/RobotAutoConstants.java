package org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

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
    public static final Pose2d[] BACKBOARD_CENTER_POSES = {
            new Pose2d(48.00, -36.00, Math.toRadians(180)),
            new Pose2d(48.00, 36.00, Math.toRadians(180))
    };
    public static final Pose2d[] SPIKEMARK_CENTER_POSES = {
            // note: red is first
            new Pose2d(48.5, -30, Math.toRadians(180)),
            new Pose2d(48.5, 30, Math.toRadians(180)),
    };
    public static final Pose2d[] CYCLING_STACK_INNER_POSES = {
            // note: again, red is first
            new Pose2d(new Vector2d(-61.62, -11.02), Math.toRadians(180.00)),
            new Pose2d(new Vector2d(-61.62, 11.02), Math.toRadians(180.00)),
    };
    public static final Pose2d[] STAGE_DOOR_POSES = {
            new Pose2d(new Vector2d(23.81, -11.73), Math.toRadians(173.04)),
            new Pose2d(new Vector2d(22.67, 14.00), Math.toRadians(187.29)),
    };
    public static final double[] BACKDROP_YELLOW_PIXEL_VARIANCE = {
            1.45,
            1.2,
            1.1,
    };
    // IMPORTANT NOTE: this changes based on alliance.
    // IMPORTANT NOTE: while this is handled automatically, note that this is from perspective of red
    public static final double[] BACKDROP_PURPLE_PIXEL_VARIANCE = {
            1.4,
            0.8,
            0.5,
    };
    public static final double[] AUDIENCE_PURPLE_PIXEL_VARIANCE = {
            2.2,
            2.2,
            2.2,
    };
    public static final double CENTER_SPIKEMARK_ALIGN_TURN = 22;
    public static final double SPIKE_TO_BACKBOARD_TRANSIT = 1.532;
}
