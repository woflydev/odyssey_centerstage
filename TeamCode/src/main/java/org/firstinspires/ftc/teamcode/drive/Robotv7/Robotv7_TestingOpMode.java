package org.firstinspires.ftc.teamcode.drive.Robotv7;

import static java.lang.Thread.sleep;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp()
public class Robotv7_TestingOpMode extends OpMode {
    public DcMotorEx backLM = null;
    public DcMotorEx backRM = null;
    public DcMotorEx frontLM = null;
    public DcMotorEx frontRM = null;
    public Servo servoClaw = null;
    public Servo servoWrist = null;
    public Servo servoElbowR = null;
    public Servo servoElbowL = null;
    public DcMotorEx armR = null;
    public DcMotorEx armL = null;
    public IMU imu = null;

    public final ElapsedTime encoderRuntime = new ElapsedTime();
    public final ElapsedTime armRuntime = new ElapsedTime();
    public final ElapsedTime resetTimer = new ElapsedTime();

    public double targetClawPosition = 0.4;
    public double targetWristPosition = 0.5;
    public double targetElbowPosition = 0.5;
    public boolean clawOpen = false;
    public boolean wristActive = false;
    public boolean elbowActive = false;

    public double current_v1 = 0;
    public double current_v2 = 0;
    public double current_v3 = 0;
    public double current_v4 = 0;

    public double driveSpeedModifier = 1;
    public boolean adjustmentAllowed = true;
    public boolean fieldCentricRed = true;
    public int targetOuttakePosition = 0;

    public void Delay(double time) {
        try { sleep((long)time); } catch (Exception e) { System.out.println("interrupted"); }
    }

    public static boolean IsPositive(double d) { return !(Double.compare(d, 0.0) < 0); }

    public double Stabilize(double new_accel, double current_accel) {
        double dev = new_accel - current_accel;
        return Math.abs(dev) > RobotConstants.MAX_ACCELERATION_DEVIATION ? current_accel + RobotConstants.MAX_ACCELERATION_DEVIATION * dev / Math.abs(dev) : new_accel;
    }

    public double GetHeading() {
        double currentHeading = imu.getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
        double rot = (double)(Math.round(-currentHeading + 720) % 360);
        rot = rot == 0 ? 360 : rot;
        return rot;
    }

    public void MoveElbow() {
        if (targetElbowPosition - 1 < 0) {
            servoElbowR.setPosition(targetElbowPosition);
            servoElbowL.setPosition(Math.abs(targetElbowPosition - 1)); // servos are mounted facing each other
        }
        Delay(50);
    }

    public void InitializeBlock() {
        driveSpeedModifier = RobotConstants.BASE_DRIVE_SPEED_MODIFIER;

        armR = hardwareMap.get(DcMotorEx.class, "armR");
        armR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // -------------------------------------------------------------- IMU INIT

        telemetry.addData("Status", "CALIBRATING IMU...");
        telemetry.addData("Important Information", "PLACE ROBOT FACING AWAY FROM ALLIANCE BOX!");
        telemetry.update();

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));

        imu = hardwareMap.get(IMU.class, RobotConstants.HUB_IMU);
        imu.initialize(parameters);
        imu.resetYaw();

        Delay(500);
    }

    public void init() {
        telemetry.addData("Status", "INITIALIZING ROBOT...");    //
        telemetry.update();

        InitializeBlock();

        telemetry.addData("Status", "INITIALIZATION COMPLETE!");
        telemetry.update();
    }

    public void loop() {
        // yes
    }
}
