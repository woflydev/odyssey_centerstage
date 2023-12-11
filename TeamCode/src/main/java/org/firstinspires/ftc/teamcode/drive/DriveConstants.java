package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class DriveConstants {
    public static final double TICKS_PER_REV = 537.6;
    public static final double MAX_RPM = 312;

    // note: not using
    public static final boolean RUN_USING_ENCODER = false;
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(0, 0, 0,
            getMotorVelocityF(MAX_RPM / 60 * TICKS_PER_REV));

    public static double WHEEL_RADIUS = 1.8898; // in
    public static double GEAR_RATIO = 1.47107712333; // output (wheel) speed / input (motor) speed //note: old - 1.933094243921361
    public static double TRACK_WIDTH = 28.52; // in

    /*
     * These are the feedforward parameters used to model the drive motor behavior. If you are using
     * the built-in velocity PID, *these values are fine as is*. However, if you do not have drive
     * motor encoders or have elected not to use them for velocity control, these values should be
     * empirically tuned.
     */
    //public static double kV = 1.0 / rpmToVelocity(MAX_RPM);
    public static double kV = 0.01; // note old: 0.0118
    public static double kA = 0.002; // note old: 0.003
    public static double kStatic = 0.16; // note old: 0.09
    public static double wheelBase = TRACK_WIDTH;

    public static double MAX_VEL = 65; // note: old - 60
    public static double MAX_ACCEL = 70.9755986904361; // note: old - 70.9755986904361
    public static double MAX_ANG_VEL = Math.toRadians(85.40001506801428); //135.50002483038097
    public static double MAX_ANG_ACCEL = Math.toRadians(70.9779569697387); //135.9779569697387

    public static RevHubOrientationOnRobot.LogoFacingDirection LOGO_FACING_DIR =
            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
    public static RevHubOrientationOnRobot.UsbFacingDirection USB_FACING_DIR =
            RevHubOrientationOnRobot.UsbFacingDirection.UP;

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }

    public static double getMotorVelocityF(double ticksPerSecond) {
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        return 32767 / ticksPerSecond;
    }

    public static boolean USE_LOCALIZER = false;
}