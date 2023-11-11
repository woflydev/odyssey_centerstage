package org.firstinspires.ftc.teamcode.Deprecated.VeryDeprecated;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;
import static java.lang.Thread.sleep;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotConstants;
import org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotState;
import org.firstinspires.ftc.teamcode.drive.Robotv8.Abstract;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Deprecated()
public class Robotv8_FullstackTesting_v2 extends OpMode {
    public Abstract handler;

    public RobotState state = RobotState.IDLE;
    public OuttakeState outtakeState = OuttakeState.IDLE;

    public DcMotorEx backLM = null;
    public DcMotorEx backRM = null;
    public DcMotorEx frontLM = null;
    public DcMotorEx frontRM = null;
    public Servo servoFlap = null;
    public Servo servoClaw = null;
    public Servo servoWrist = null;
    public Servo servoElbowR = null;
    public Servo servoElbowL = null;
    public Servo servoPlane = null;
    public CRServo servoHangR = null;
    public CRServo servoHangL = null;
    public DcMotorEx armR = null;
    public DcMotorEx armL = null;
    public IMU imu = null;
    public DcMotorEx intake = null;

    public final ElapsedTime encoderRuntime = new ElapsedTime();
    public final ElapsedTime armRuntime = new ElapsedTime();
    public final ElapsedTime resetTimer = new ElapsedTime();

    public double targetClawPosition = RobotConstants.CLAW_OPEN;
    public double targetWristPosition = RobotConstants.WRIST_PICKUP;
    public double targetElbowPosition = RobotConstants.ELBOW_STANDBY;
    public double targetPlanePosition = RobotConstants.PLANE_STANDBY;
    public double targetFlapPosition = RobotConstants.FLAP_CLOSE;
    public int targetOuttakePosition = 0;

    public boolean planeTriggered = false;
    public boolean clawOpen = false;
    public boolean wristActive = false;
    public boolean elbowActive = false;
    public boolean transferStageDeployed = false;

    public double current_v1 = 0;
    public double current_v2 = 0;
    public double current_v3 = 0;
    public double current_v4 = 0;

    public double driveSpeedModifier = 1;
    public boolean adjustmentAllowed = true;
    public boolean fieldCentricRed = true;

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

    public void MoveElbow(double targetPos) {
        servoElbowR.setPosition(targetPos);
        servoElbowL.setPosition(1 - targetPos); // Set to the opposite position
        Delay(50);
    }

    public void InitializeBlock() {
        driveSpeedModifier = RobotConstants.BASE_DRIVE_SPEED_MODIFIER;

        backLM = hardwareMap.get(DcMotorEx.class, RobotConstants.BACK_LEFT);
        backRM = hardwareMap.get(DcMotorEx.class, RobotConstants.BACK_RIGHT);

        frontLM = hardwareMap.get(DcMotorEx.class, RobotConstants.FRONT_LEFT);
        frontRM = hardwareMap.get(DcMotorEx.class, RobotConstants.FRONT_RIGHT); //frontRM.setDirection(DcMotorSimple.Direction.REVERSE); // weird workaround Stanley put in
        intake = hardwareMap.get(DcMotorEx.class, RobotConstants.INTAKE_MOTOR);

        backLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backLM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRM.setDirection(DcMotorSimple.Direction.REVERSE);
        backRM.setDirection(DcMotorSimple.Direction.REVERSE);

        armR = hardwareMap.get(DcMotorEx.class, RobotConstants.ARM_R);
        armL = hardwareMap.get(DcMotorEx.class, RobotConstants.ARM_L);

        armR.setDirection(DcMotorSimple.Direction.REVERSE);
        armL.setDirection(DcMotorSimple.Direction.FORWARD);
        armR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armR.setTargetPosition(0);
        armL.setTargetPosition(0);
        armR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        servoFlap = hardwareMap.get(Servo.class, RobotConstants.SERVO_FLAP);
        servoElbowR = hardwareMap.get(Servo.class, RobotConstants.SERVO_ELBOW_R);
        servoElbowL = hardwareMap.get(Servo.class, RobotConstants.SERVO_ELBOW_L);
        servoClaw = hardwareMap.get(Servo.class, RobotConstants.SERVO_CLAW);
        servoWrist = hardwareMap.get(Servo.class, RobotConstants.SERVO_WRIST);
        servoPlane = hardwareMap.get(Servo.class, RobotConstants.SERVO_PLANE);

        servoHangR = hardwareMap.get(CRServo.class, RobotConstants.SERVO_HANG_R);
        servoHangR.setDirection(DcMotorSimple.Direction.FORWARD);
        servoHangR.setPower(0);

        servoHangL = hardwareMap.get(CRServo.class, RobotConstants.SERVO_HANG_L);
        servoHangL.setDirection(DcMotorSimple.Direction.REVERSE);
        servoHangL.setPower(0);

        clawOpen = true;
        transferStageDeployed = false;
        servoFlap.setPosition(RobotConstants.FLAP_CLOSE);
        servoClaw.setPosition(RobotConstants.CLAW_OPEN);
        servoWrist.setPosition(RobotConstants.WRIST_STANDBY);
        servoPlane.setPosition(RobotConstants.PLANE_STANDBY);
        MoveElbow(RobotConstants.ELBOW_STANDBY); // special function for inverted servos*/

        // -------------------------------------------------------------- IMU INIT

        telemetry.addData("Status", "CALIBRATING IMU...");
        telemetry.addData("Important Information", "PLACE ROBOT FACING AWAY FROM ALLIANCE BOX!");
        telemetry.update();

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));

        imu = hardwareMap.get(IMU.class, RobotConstants.HUB_IMU);
        imu.initialize(parameters);
        imu.resetYaw();

        Delay(100);
    }

    public void init() {
        telemetry.addData("Status", "INITIALIZING ROBOT...");
        telemetry.update();

        Delay(100);

        InitializeBlock();
        MainInit();

        telemetry.addData("Status", "INITIALIZATION COMPLETE!");
        telemetry.update();
    }
    public void start() {
        MainStart();
    }

    public void loop() {
        StatusTelemetry();

        // Optional, to see position output
        //handler.update();
        MainLoop();
    }
    public void stop() {
        MainStop();
        handler.stop();
    }

    public void MainInit() {

    }

    public void MainStart() {

    }

    public void MainLoop() {

    }

    public void MainStop() {

    }

    public void StatusTelemetry() {
        // TELEMETRY
        telemetry.addData("Arm Left: ", armL.getCurrentPosition());
        telemetry.addData("Arm Right: ", armR.getCurrentPosition());
        telemetry.addData("FrontRM Encoder Value: ", frontRM.getCurrentPosition());
        telemetry.addData("FrontLM Encoder Value: ", frontLM.getCurrentPosition());
        telemetry.addData("BackRM Encoder Value: ", backRM.getCurrentPosition());
        telemetry.addData("BackLM Encoder Value: ", backLM.getCurrentPosition());
        telemetry.addData("Target Flap Position: ", targetFlapPosition);
        telemetry.addData("Target Wrist Position: ", targetWristPosition);
        telemetry.addData("Target Elbow Position: ", targetElbowPosition);
        telemetry.addData("Target Outtake Position: ", targetOuttakePosition);
        telemetry.addData("Target Claw Position: ", targetClawPosition);
        telemetry.addData("Adjustment Allowed: ", adjustmentAllowed);
        telemetry.addData("Field Centric Mode : ", fieldCentricRed ? "RED" : "BLUE");
        telemetry.addData("Current Speed Mode: ", driveSpeedModifier == RobotConstants.BASE_DRIVE_SPEED_MODIFIER ? "BASE SPEED" : "PRECISION MODE");
        //telemetry.addData("IMU Yaw: ", GetHeading());

        telemetry.update();
    }

    public void RuntimeConfig() {
        // -------------------------------------------------------------- MANUAL ARM CONTROL (directly effects bot)
        if (adjustmentAllowed) { // lining up arm for topmost cone
            // slow down driving with analog trigger
            if (gamepad1.right_trigger >= 0.2) {
                driveSpeedModifier = gamepad1.right_trigger + 1.3;
            } else {
                driveSpeedModifier = 1;
            }

            if (gamepad2.right_trigger >= 0.6 && ((armL.getCurrentPosition() < RobotConstants.MAX_OUTTAKE_HEIGHT - RobotConstants.ARM_ADJUSTMENT_INCREMENT) && (armR.getCurrentPosition() < RobotConstants.MAX_OUTTAKE_HEIGHT - RobotConstants.ARM_ADJUSTMENT_INCREMENT))) {
                if (targetOuttakePosition < RobotConstants.MAX_OUTTAKE_HEIGHT - RobotConstants.ARM_ADJUSTMENT_INCREMENT) {
                    targetOuttakePosition += RobotConstants.ARM_ADJUSTMENT_INCREMENT;
                    UpdateOuttake(false, 0);
                }
            } else if (gamepad2.left_trigger >= 0.6 && ((armL.getCurrentPosition() > RobotConstants.MIN_OUTTAKE_HEIGHT + RobotConstants.ARM_ADJUSTMENT_INCREMENT) && (armR.getCurrentPosition() > RobotConstants.MIN_OUTTAKE_HEIGHT + RobotConstants.ARM_ADJUSTMENT_INCREMENT))) {
                if (targetOuttakePosition > RobotConstants.MIN_OUTTAKE_HEIGHT + RobotConstants.ARM_ADJUSTMENT_INCREMENT) {
                    targetOuttakePosition -= RobotConstants.ARM_ADJUSTMENT_INCREMENT;
                    UpdateOuttake(false, 0);
                }
            } else if (gamepad2.dpad_down) {
                targetOuttakePosition = 30;
                UpdateOuttake(true, 0);
            } else if (gamepad2.dpad_up) {
                targetOuttakePosition = RobotConstants.MAX_OUTTAKE_HEIGHT;
                UpdateOuttake(false, 0);
            }

            if (gamepad2.square) {
                targetClawPosition -= 0.02;
                servoClaw.setPosition(targetClawPosition);
                Delay(50);
            } else if (gamepad2.circle) {
                targetClawPosition += 0.02;
                servoClaw.setPosition(targetClawPosition);
                Delay(50);
            }

            // todo: relinquish wrist control in favour of hanging
            if (gamepad2.right_bumper) {
                targetWristPosition += 0.02;
                servoWrist.setPosition(targetWristPosition);
                Delay(50);
            } else if (gamepad2.left_bumper) {
                targetWristPosition -= 0.02;
                servoWrist.setPosition(targetWristPosition);
                Delay(50);
            }

            /*if (gamepad2.right_bumper) {
                servoHangR.setPower(1);
                servoHangL.setPower(1);
            } else if (gamepad2.left_bumper) {
                servoHangR.setPower(-1);
                servoHangL.setPower(-1);
            } else {
                servoHangR.setPower(0);
                servoHangL.setPower(0);
            }*/

            // FLAP (FOR TUNING VALUES) -----------------------------------------------
            if (gamepad2.dpad_right) {
                targetFlapPosition += 0.02;
                servoFlap.setPosition(targetFlapPosition);
            } else if (gamepad2.dpad_left) {
                targetFlapPosition -= 0.02;
                servoFlap.setPosition(targetFlapPosition);
            }

            // PLANE -------------------------------------------------------------------
            if (gamepad2.cross) {
                if (!planeTriggered) {
                    planeTriggered = true;
                    servoPlane.setPosition(RobotConstants.PLANE_ACTIVE);
                } else {
                    planeTriggered = false;
                    servoPlane.setPosition(RobotConstants.PLANE_STANDBY);
                }
                Delay(250);
            }

            // ELBOW ------------------------------------------------------------------
            if (gamepad1.dpad_up) {
                targetElbowPosition += 0.02;
                MoveElbow(targetElbowPosition);
            } else if (gamepad1.dpad_down) {
                targetElbowPosition -= 0.02;
                MoveElbow(targetElbowPosition);
            }
        }

        // -------------------------------------------------------------- CONFIGURATION (don't directly move the bot)
        if (gamepad1.start) { // re-calibrate field centric drive
            imu.resetYaw();
        }
    }

    private volatile boolean drivebaseMacroActive = false;
    public void TurnToHeadingAsync(double targetHeading) {
        if (drivebaseMacroActive) {
            return; // A macro is already in progress
        }

        drivebaseMacroActive = true;

        double frontLeftPower;
        double backLeftPower;
        double frontRightPower;
        double backRightPower;

        double currentHeading;
        double error;
        double Kp = 0.03; // Adjust this gain as needed

        currentHeading = GetHeading();
        error = targetHeading - currentHeading;

        while (Math.abs(error) > 0.02) {
            currentHeading = GetHeading();
            error = targetHeading - currentHeading;

            // Determine the direction of rotation (clockwise or counterclockwise)
            double rotatePower;
            if (error > 180) {
                // If error is more than 180 degrees, rotate in the opposite direction
                rotatePower = -Math.abs(error) * Kp;
            } else {
                rotatePower = Math.abs(error) * Kp;
            }

            // Apply the rotation power to the robot
            frontLeftPower = -rotatePower;
            backLeftPower = -rotatePower;
            frontRightPower = rotatePower;
            backRightPower = rotatePower;

            // Update motor powers
            frontLM.setPower(frontLeftPower);
            frontRM.setPower(frontRightPower);
            backLM.setPower(backLeftPower);
            backRM.setPower(backRightPower);

            // Allow other controls to run in parallel
            // For example, you can continue reading gamepad input here

            // Sleep for a short duration to avoid busy-waiting
            Delay(20); // Adjust the sleep duration as needed
        }

        // Stop the robot
        frontLM.setPower(0);
        frontRM.setPower(0);
        backLM.setPower(0);
        backRM.setPower(0);

        drivebaseMacroActive = false;
    }

    public void Mecanum() {
        double frontLeftPower;
        double backLeftPower;
        double frontRightPower;
        double backRightPower;

        double yAxis;
        double xAxis;
        double rotateAxis;

        int dir = fieldCentricRed ? 1 : -1;

        // all negative when field centric red
        yAxis = gamepad1.left_stick_y * dir;
        xAxis = -gamepad1.left_stick_x * 1.1 * dir;
        rotateAxis = -gamepad1.right_stick_x * dir;

        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = xAxis * Math.cos(-heading) - yAxis * Math.sin(-heading);
        double rotY = xAxis * Math.sin(-heading) + yAxis * Math.cos(-heading);

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rotateAxis), 1);
        frontLeftPower = (rotY + rotX + rotateAxis) / denominator;
        backLeftPower = (rotY - rotX + rotateAxis) / denominator;
        frontRightPower = (rotY - rotX - rotateAxis) / denominator;
        backRightPower = (rotY + rotX - rotateAxis) / denominator;

        double stable_v1 = Stabilize(backLeftPower, current_v1);
        double stable_v2 = Stabilize(frontRightPower, current_v2);
        double stable_v3 = Stabilize(frontLeftPower, current_v3);
        double stable_v4 = Stabilize(backRightPower, current_v4);

        current_v1 = stable_v1;
        current_v2 = stable_v2;
        current_v3 = stable_v3;
        current_v4 = stable_v4;

        /*if (gamepad1.dpad_right) {
            double targetHeading = 270;
            TurnToHeadingAsync(targetHeading);
        } else if (gamepad1.dpad_left) {
            double targetHeading = 90;
            TurnToHeadingAsync(targetHeading);
        }*/
        frontLM.setPower(stable_v3 / driveSpeedModifier);
        frontRM.setPower(stable_v2 / driveSpeedModifier);
        backLM.setPower(stable_v1 / driveSpeedModifier);
        backRM.setPower(stable_v4 / driveSpeedModifier);

        // TODO: test if deadzone works
        if (Math.abs(gamepad1.left_stick_y) >= RobotConstants.JOYSTICK_DEADZONE && Math.abs(gamepad1.left_stick_x) >= RobotConstants.JOYSTICK_DEADZONE) {

        }
    }

    public void Macros() {
        // INTAKE
        if (gamepad1.left_trigger > 0.2 || gamepad2.triangle) {
            intake.setPower(RobotConstants.MAX_MANUAL_INTAKE_POWER);
        } else if (gamepad1.square) {
            intake.setPower(-RobotConstants.MAX_MANUAL_INTAKE_POWER);
        } else {
            intake.setPower(0);
        }

        // GRAB
        if (gamepad1.left_bumper && outtakeState == OuttakeState.IDLE) {
            intake.setPower(0);
            GrabAndReady();
            outtakeState = OuttakeState.GRABBED_AND_READY;
        }

        // RESET
        if (gamepad1.right_bumper && !(outtakeState == OuttakeState.PRIMED_FOR_DEPOSIT)) {
            DropAndReset();
            outtakeState = OuttakeState.IDLE;
        }

        // DEPLOY & RESET DEPENDING ON STATE
        if (outtakeState == OuttakeState.GRABBED_AND_READY || outtakeState == OuttakeState.PRIMED_FOR_DEPOSIT) {
            if (gamepad1.cross) {
                DepositSequence(RobotConstants.JUNCTION_LOW);
            } else if (gamepad1.circle) {
                DepositSequence(RobotConstants.JUNCTION_MID);
            } else if (gamepad1.triangle) {
                DepositSequence(RobotConstants.JUNCTION_HIGH);
            }
        }

        Delay(5); // debounce
    }

    // INTAKE/OUTTAKE SEQUENCE FUNCTIONS --------------------------------------------------
    public void DepositSequence(int height) {
        if (outtakeState == OuttakeState.GRABBED_AND_READY) {
            intake.setPower(0); // make sure intake is not running
            RaiseAndPrime(height);
            outtakeState = OuttakeState.PRIMED_FOR_DEPOSIT;

        } else {
            DropAndReset();
            outtakeState = OuttakeState.IDLE;
        }
        Delay(50);
    }

    public void GrabAndReady() {
        servoFlap.setPosition(RobotConstants.FLAP_OPEN);
        Delay(700);

        // transfer stage sequence
        servoWrist.setPosition(RobotConstants.WRIST_PICKUP);
        MoveElbow(RobotConstants.ELBOW_STANDBY); // moves it up a little to avoid tubes
        Delay(200);
        MoveElbow(RobotConstants.ELBOW_PICKUP);

        Delay(200);
        servoClaw.setPosition(RobotConstants.CLAW_CLOSE);
        Delay(250);

        // primes the elbow
        MoveElbow(RobotConstants.ELBOW_STANDBY);
        Delay(100);
        servoWrist.setPosition(RobotConstants.WRIST_STANDBY);
    }

    public void RaiseAndPrime(int height) {
        servoFlap.setPosition(RobotConstants.FLAP_CLOSE);
        servoClaw.setPosition(RobotConstants.CLAW_CLOSE);
        servoWrist.setPosition(RobotConstants.WRIST_ACTIVE);

        MoveElbow(RobotConstants.ELBOW_ACTIVE);

        Delay(100);

        targetOuttakePosition = height;
        UpdateOuttake(false, 0);
    }

    public void DropAndReset() {
        servoClaw.setPosition(RobotConstants.CLAW_OPEN);
        Delay(300); // wait for claw to open

        servoWrist.setPosition(RobotConstants.WRIST_STANDBY);
        MoveElbow(RobotConstants.ELBOW_STANDBY);

        Delay(350); // elbow should come down after the slide is near done

        servoFlap.setPosition(RobotConstants.FLAP_CLOSE);
        targetOuttakePosition = 10;
        UpdateOuttake(true, 0);
    }


    // HELPER FUNCTIONS ------------------------------------------------------------------
    public void UpdateOuttake(boolean reset, double delay) { // test new function
        if (reset) {
            Delay(delay);
            armR.setTargetPosition(10);
            armL.setTargetPosition(10);
            targetOuttakePosition = 10;
            armRuntime.reset();
            armR.setVelocity(RobotConstants.MAX_OUTTAKE_SPEED);
            armL.setVelocity(RobotConstants.MAX_OUTTAKE_SPEED);
            /*while (armM.getCurrentPosition() >= 50 || armRuntime.seconds() <= ARM_RESET_TIMEOUT) {
                armM.setVelocity((double)2100 / ARM_BOOST_MODIFIER);

                if (armM.getCurrentPosition() <= 50 || armRuntime.seconds() >= ARM_RESET_TIMEOUT) {
                    break;
                }
            }*/

            if ((armL.getCurrentPosition() <= 20 || armR.getCurrentPosition() <= 20) || armRuntime.seconds() >= RobotConstants.ARM_RESET_TIMEOUT) {
                armR.setVelocity(0);
                armL.setVelocity(0);
            }

            telemetry.update();
        }

        else {
            Delay(delay);
            armR.setTargetPosition(targetOuttakePosition);
            armL.setTargetPosition(targetOuttakePosition);
            armRuntime.reset();
            armR.setVelocity(RobotConstants.MAX_OUTTAKE_SPEED);
            armL.setVelocity(RobotConstants.MAX_OUTTAKE_SPEED); // velocity used to be 1800, could be faster
        }
    }

    public void PassiveArmResetCheck() {
        if (targetOuttakePosition <= 30) {
            if ((armL.getCurrentPosition() <= 10 && armR.getCurrentPosition() <= 10) && (armL.getCurrentPosition() >= 0 && armR.getCurrentPosition() <= 0)) {
                armR.setVelocity(0);
                armL.setVelocity(0);
            } else if ((armL.getCurrentPosition() <= 210 && armR.getCurrentPosition() <= 210) && (armL.getCurrentPosition() >= -100 && armR.getCurrentPosition() >= -100)) {
                armR.setTargetPosition(10);
                armL.setTargetPosition(10);
                targetOuttakePosition = 10;

                armR.setVelocity(800);
                armL.setVelocity(800);
            }
        }
    }

    public static class AutoMecanumDrive extends MecanumDrive {
        public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(0, 0, 0);
        public static PIDCoefficients HEADING_PID = new PIDCoefficients(0, 0, 0);

        public static double LATERAL_MULTIPLIER = 1;

        public static double VX_WEIGHT = 1;
        public static double VY_WEIGHT = 1;
        public static double OMEGA_WEIGHT = 1;

        private TrajectorySequenceRunner trajectorySequenceRunner;

        private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
        private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL);

        private TrajectoryFollower follower;

        private DcMotorEx leftFront, leftRear, rightRear, rightFront;
        private List<DcMotorEx> motors;

        private IMU imu;
        private VoltageSensor batteryVoltageSensor;

        private List<Integer> lastEncPositions = new ArrayList<>();
        private List<Integer> lastEncVels = new ArrayList<>();

        public AutoMecanumDrive(Abstract handler, HardwareMap hardwareMap, DcMotorEx frontLM, DcMotorEx frontRM, DcMotorEx backLM, DcMotorEx backRM, IMU i) {
            super(DriveConstants.kV, DriveConstants.kA, DriveConstants.kStatic,
                    TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER);
            follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                    new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);
            LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

            batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

            for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
                module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
            }

            // TODO: adjust the names of the following hardware devices to match your configuration
            imu = i;

            leftFront = frontLM;
            leftRear = backLM;
            rightRear = backRM;
            rightFront = frontRM;

            motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

            for (DcMotorEx motor : motors) {
                MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
                motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
                motor.setMotorType(motorConfigurationType);
            }

            if (RUN_USING_ENCODER) {
                setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
                setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
            }

            // TODO: reverse any motors using DcMotor.setDirection()
            rightFront.setDirection(DcMotorEx.Direction.REVERSE);
            rightRear.setDirection(DcMotorEx.Direction.REVERSE);

            List<Integer> lastTrackingEncPositions = new ArrayList<>();
            List<Integer> lastTrackingEncVels = new ArrayList<>();

            setLocalizer(handler.localizer);

            trajectorySequenceRunner = new TrajectorySequenceRunner(
                    follower, HEADING_PID, batteryVoltageSensor,
                    lastEncPositions, lastEncVels, lastTrackingEncPositions, lastTrackingEncVels
            );
        }
        public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
            return new TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
        }

        public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
            return new TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
        }

        public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
            return new TrajectoryBuilder(startPose, startHeading, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
        }

        public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
            return new TrajectorySequenceBuilder(
                    startPose,
                    VEL_CONSTRAINT, ACCEL_CONSTRAINT,
                    MAX_ANG_VEL, MAX_ANG_ACCEL
            );
        }

        public void turnAsync(double angle) {
            trajectorySequenceRunner.followTrajectorySequenceAsync(
                    trajectorySequenceBuilder(getPoseEstimate())
                            .turn(angle)
                            .build()
            );
        }

        public void turn(double angle) {
            turnAsync(angle);
            waitForIdle();
        }

        public void followTrajectoryAsync(Trajectory trajectory) {
            trajectorySequenceRunner.followTrajectorySequenceAsync(
                    trajectorySequenceBuilder(trajectory.start())
                            .addTrajectory(trajectory)
                            .build()
            );
        }

        public void followTrajectory(Trajectory trajectory) {
            followTrajectoryAsync(trajectory);
            waitForIdle();
        }

        public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
            trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
        }

        public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
            followTrajectorySequenceAsync(trajectorySequence);
            waitForIdle();
        }

        public Pose2d getLastError() {
            return trajectorySequenceRunner.getLastPoseError();
        }

        public void update() {
            updatePoseEstimate();
            DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
            if (signal != null) setDriveSignal(signal);
        }

        public void waitForIdle() {
            while (!Thread.currentThread().isInterrupted() && isBusy())
                update();
        }

        public boolean isBusy() {
            return trajectorySequenceRunner.isBusy();
        }

        public void setMode(DcMotor.RunMode runMode) {
            for (DcMotorEx motor : motors) {
                motor.setMode(runMode);
            }
        }

        public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
            for (DcMotorEx motor : motors) {
                motor.setZeroPowerBehavior(zeroPowerBehavior);
            }
        }

        public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
            PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                    coefficients.p, coefficients.i, coefficients.d,
                    coefficients.f * 12 / batteryVoltageSensor.getVoltage()
            );

            for (DcMotorEx motor : motors) {
                motor.setPIDFCoefficients(runMode, compensatedCoefficients);
            }
        }

        public void setWeightedDrivePower(Pose2d drivePower) {
            Pose2d vel = drivePower;

            if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                    + Math.abs(drivePower.getHeading()) > 1) {
                // re-normalize the powers according to the weights
                double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                        + VY_WEIGHT * Math.abs(drivePower.getY())
                        + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

                vel = new Pose2d(
                        VX_WEIGHT * drivePower.getX(),
                        VY_WEIGHT * drivePower.getY(),
                        OMEGA_WEIGHT * drivePower.getHeading()
                ).div(denom);
            }

            setDrivePower(vel);
        }

        @NonNull
        @Override
        public List<Double> getWheelPositions() {
            lastEncPositions.clear();

            List<Double> wheelPositions = new ArrayList<>();
            for (DcMotorEx motor : motors) {
                int position = motor.getCurrentPosition();
                lastEncPositions.add(position);
                wheelPositions.add(encoderTicksToInches(position));
            }
            return wheelPositions;
        }

        @Override
        public List<Double> getWheelVelocities() {
            lastEncVels.clear();

            List<Double> wheelVelocities = new ArrayList<>();
            for (DcMotorEx motor : motors) {
                int vel = (int) motor.getVelocity();
                lastEncVels.add(vel);
                wheelVelocities.add(encoderTicksToInches(vel));
            }
            return wheelVelocities;
        }

        @Override
        public void setMotorPowers(double v, double v1, double v2, double v3) {
            leftFront.setPower(v);
            leftRear.setPower(v1);
            rightRear.setPower(v2);
            rightFront.setPower(v3);
        }

        @Override
        public double getRawExternalHeading() {
            return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        }

        @Override
        public Double getExternalHeadingVelocity() {
            return (double) imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;
        }

        public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
            return new MinVelocityConstraint(Arrays.asList(
                    new AngularVelocityConstraint(maxAngularVel),
                    new MecanumVelocityConstraint(maxVel, trackWidth)
            ));
        }

        public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
            return new ProfileAccelerationConstraint(maxAccel);
        }
    }
}
