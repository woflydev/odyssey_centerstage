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
            new Pose2d(49, 53.5, Math.toRadians(270))
    };
    public static final Pose2d[] BACKDROP_CENTER_POSES = {
            new Pose2d(50.5, -36.00, Math.toRadians(180)),
            new Pose2d(50.5, 36.00, Math.toRadians(180))
    };
    public static final Pose2d[] SPIKEMARK_CENTER_POSES = {
            // note: red is first, must be backwards to deposit
            new Pose2d(48.5, -30, Math.toRadians(0)),
            new Pose2d(48.5, 30, Math.toRadians(0)),
    };
    public static final Pose2d[] CYCLING_STACK_INNER_POSES = {
            // note: again, red is first
            // note: goes forward slowly in FSM_Auto
            new Pose2d(new Vector2d(-54.5, -12.4), Math.toRadians(180.00)),
            new Pose2d(new Vector2d(-54.5, 12.4), Math.toRadians(180.00)),
    };
    public static final Pose2d[] STAGE_DOOR_POSES = {
            new Pose2d(new Vector2d(32.5, -8.73), Math.toRadians(173.04)),
            new Pose2d(new Vector2d(32.5, 8.00), Math.toRadians(187.29)),
    };
    public static final double[] BACKDROP_YELLOW_PIXEL_VARIANCE = {
            1.4,
            1.2,
            0.94,
    };
    // IMPORTANT NOTE: this changes based on alliance.
    // IMPORTANT NOTE: while this is handled automatically, note that this is from perspective of red
    public static final double[] BACKDROP_PURPLE_PIXEL_VARIANCE = {
            1.45,
            0.85,
            0.5,
    };
    public static final double[] AUDIENCE_PURPLE_PIXEL_VARIANCE = {
            2.2,
            2.2,
            2.2,
    };
    public static final double CAUTION_SPEED = 5;
    public static final double CENTER_SPIKEMARK_ALIGN_TURN = 23;
    public static final double SPIKE_TO_BACKBOARD_TRANSIT = 1.525;
    public static final double CYCLE_STACK_APPROACH_AMOUNT = 1.5;
}
