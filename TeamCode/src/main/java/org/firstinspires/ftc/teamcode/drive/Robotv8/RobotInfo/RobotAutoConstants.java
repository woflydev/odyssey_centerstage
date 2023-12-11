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
            new Pose2d(11, 63, Math.toRadians(270)),
            new Pose2d(-35, 63, Math.toRadians(270)),
    };
    public static final Pose2d[] RED_PARKING_POSES = {
            // note: inner is first
            new Pose2d(47, -15, Math.toRadians(90)),
            new Pose2d(46, -60, Math.toRadians(90)),
    };
    public static final Pose2d[] BLUE_PARKING_POSES = {
            new Pose2d(47, 15, Math.toRadians(270)),
            new Pose2d(46, 60, Math.toRadians(270)),
    };
    public static final Pose2d[] BACKDROP_CENTER_POSES = {
            new Pose2d(43, -35.00, Math.toRadians(180)),
            new Pose2d(43, 35.00, Math.toRadians(180)),
    };
    public static final Pose2d[] SPIKEMARK_CENTER_POSES = {
            // note: red is first, must be backwards to deposit
            new Pose2d(44.5, -30, Math.toRadians(0)),
            new Pose2d(44.5, 30, Math.toRadians(0)),
    };
    // note: used for aligning robot SPIKEMARK -> BACKDROP when starting from audience side
    public static final Pose2d[] SPIKEMARK_TRANSIT_CENTER_POSES = {
            new Pose2d(-36, -7, Math.toRadians(180)),
            new Pose2d(-36, 7, Math.toRadians(180)),
    };
    public static final Pose2d[] RED_YELLOW_PIXEL_BACKDROP_POSES = {
            // note: starts with LOC_1
            new Pose2d(42, -28, Math.toRadians(180)),
            new Pose2d(42, -33.3, Math.toRadians(180)),
            new Pose2d(42, -38, Math.toRadians(180)),
    };
    public static final Pose2d[] BLUE_YELLOW_PIXEL_BACKUP_POSES = {
            new Pose2d(42, 38, Math.toRadians(180)),
            new Pose2d(42, 33.3, Math.toRadians(180)),
            new Pose2d(42, 28, Math.toRadians(180)),
    };
    public static final Pose2d[] CYCLING_STACK_INNER_POSES = {
            // note: again, red is first
            // note: goes forward slowly in FSM_Auto
            new Pose2d(new Vector2d(-57.5, -14), Math.toRadians(180.00)),
            new Pose2d(new Vector2d(-57.5, 14), Math.toRadians(180.00)),
    };
    public static final Pose2d[] STAGE_DOOR_POSES = {
            new Pose2d(new Vector2d(16, -4), Math.toRadians(180.0)), // note: old values - (28, 8)
            new Pose2d(new Vector2d(16, 4), Math.toRadians(180.0)),
    };
    public static final Pose2d[] CYCLE_RETURN_POSES = {
            new Pose2d(new Vector2d(30, -5), Math.toRadians(180.0)),
            new Pose2d(new Vector2d(30, 5), Math.toRadians(180.0)),
    };
    public static final double[] BACKDROP_YELLOW_PIXEL_VARIANCE = {
            1.38,
            1.20,
            0.89,
    };
    public static final double[] AUDIENCE_YELLOW_PIXEL_VARIANCE = {
            1.4,
            1.2,
            0.94,
    };

    // IMPORTANT NOTE: this changes based on alliance.
    // IMPORTANT NOTE: while this is handled automatically, note that this is from perspective of red
    public static final double[] BACKDROP_RED_PURPLE_PIXEL_VARIANCE = {
            1.65,
            1.03,
            0.55,
    };
    public static final double[] BACKDROP_BLUE_PURPLE_PIXEL_VARIANCE = {
            1.65,
            1.03,
            0.55,
    };
    public static final double[] AUDIENCE_PURPLE_PIXEL_VARIANCE = {
            1.80,
            2.17,
            1.80,
    };
    public static final double[] AUDIENCE_PURPLE_PIXEL_ALIGN_VARIANCE = {
            // IMPORTANT NOTE: loc 3 is first here, since is flipped
            54,
            0,
            62,
    };
    public static final int YELLOW_PIXEL_DEPOSIT_HEIGHT = 0; // todo: test if new val works - old was 250

    public static final double CAUTION_SPEED = 14;
    public static final double BACKDROP_CENTER_SPIKEMARK_ALIGN_TURN_DEG = 30.5;
    public static final double BACKDROP_DEPOSIT_PUSHBACK_AMOUNT = 0.22;

    public static final double DEPOSIT_YELLOW_TO_BACKDROP_TRANSIT_RED = 1.55;
    public static final double DEPOSIT_YELLOW_TO_BACKDROP_TRANSIT_BLUE = 1.45;

    public static final double CYCLE_STACK_APPROACH_AMOUNT = 0.05;
    public static final double CYCLE_STACK_REVERSE_AMOUNT = CYCLE_STACK_APPROACH_AMOUNT - 0.22;
    public static final double CYCLE_BACKDROP_APPROACH_AMOUNT = 0.45;

    public static final double AUDIENCE_PURPLE_APPROACH_SPEED = 20;
    public static final double AUDIENCE_YELLOW_BACKDROP_APPROACH_AMOUNT = 0.485;
}
