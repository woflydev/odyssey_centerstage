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
            new Pose2d(49, -15, Math.toRadians(90)),
            new Pose2d(49, -53.5, Math.toRadians(90)),
    };
    public static final Pose2d[] BLUE_PARKING_POSES = {
            new Pose2d(49, 15, Math.toRadians(270)),
            new Pose2d(49, 53.5, Math.toRadians(270)),
    };
    public static final Pose2d[] BACKDROP_CENTER_POSES = {
            new Pose2d(43, -35.00, Math.toRadians(180)),
            new Pose2d(43, 35.00, Math.toRadians(180)),
    };
    public static final Pose2d[] SPIKEMARK_CENTER_POSES = {
            // note: red is first, must be backwards to deposit
            new Pose2d(46.5, -30, Math.toRadians(0)),
            new Pose2d(46.5, 30, Math.toRadians(0)),
    };
    // note: used for aligning robot SPIKEMARK -> BACKDROP when starting from audience side
    public static final Pose2d[] SPIKEMARK_TRANSIT_CENTER_POSES = {
            new Pose2d(40, -11, Math.toRadians(180)),
            new Pose2d(40, -11, Math.toRadians(180)),
    };
    public static final Pose2d[] RED_YELLOW_PIXEL_BACKDROP_POSES = {
            // note: starts with LOC_1
            new Pose2d(50, -30, Math.toRadians(180)),
            new Pose2d(50, -35, Math.toRadians(180)),
            new Pose2d(50, -40, Math.toRadians(180)),
    };
    public static final Pose2d[] BLUE_YELLOW_PIXEL_BACKUP_POSES = {
            new Pose2d(50, 30, Math.toRadians(180)),
            new Pose2d(50, 35, Math.toRadians(180)),
            new Pose2d(50, 40, Math.toRadians(180)),
    };
    public static final Pose2d[] CYCLING_STACK_INNER_POSES = {
            // note: again, red is first
            // note: goes forward slowly in FSM_Auto
            new Pose2d(new Vector2d(-58, -12), Math.toRadians(180.00)),
            new Pose2d(new Vector2d(-58, 12), Math.toRadians(180.00)),
    };
    public static final Pose2d[] STAGE_DOOR_POSES = {
            new Pose2d(new Vector2d(28, -8), Math.toRadians(173.04)),
            new Pose2d(new Vector2d(28, 8), Math.toRadians(187.29)),
    };
    public static final double[] BACKDROP_YELLOW_PIXEL_VARIANCE = {
            1.33,
            1.2,
            0.925,
    };
    public static final double[] AUDIENCE_YELLOW_PIXEL_VARIANCE = {
            1.4,
            1.2,
            0.94,
    };

    // IMPORTANT NOTE: this changes based on alliance.
    // IMPORTANT NOTE: while this is handled automatically, note that this is from perspective of red
    public static final double[] BACKDROP_PURPLE_PIXEL_VARIANCE = {
            1.54,
            1.15,
            0.60,
    };
    public static final double[] AUDIENCE_PURPLE_PIXEL_VARIANCE = {
            2.1,
            2.25,
            2.1,
    };
    public static final double CAUTION_SPEED = 10;
    public static final double BACKDROP_CENTER_SPIKEMARK_ALIGN_TURN_DEG = 27;
    public static final double BACKDROP_DEPOSIT_PUSHBACK_AMOUNT = 0.17;

    public static final double[] AUDIENCE_PURPLE_PIXEL_ALIGN_VARIANCE = {
            20,
            0,
            20,
    };

    public static final double DEPOSIT_YELLOW_TO_BACKDROP_TRANSIT = 1.7;
    public static final double CYCLE_STACK_APPROACH_AMOUNT = 1.5;
}
