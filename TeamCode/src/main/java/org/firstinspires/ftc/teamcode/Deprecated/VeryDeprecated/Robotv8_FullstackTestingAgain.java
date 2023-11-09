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
public class Robotv8_FullstackTestingAgain extends OpMode {
    public NewRobot_v8_AbstractTesting handler;

    public RobotState state = RobotState.IDLE;

    public DcMotorEx backLM = null;
    public DcMotorEx backRM = null;
    public DcMotorEx frontLM = null;
    public DcMotorEx frontRM = null;
    public Servo servoClaw = null;
    public Servo servoWrist = null;
    public Servo servoElbowR = null;
    public Servo servoElbowL = null;
    public Servo servoPlane = null;
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

    public AutoMecanumDrive drive;

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

        servoElbowR = hardwareMap.get(Servo.class, RobotConstants.SERVO_ELBOW_R);
        servoElbowL = hardwareMap.get(Servo.class, RobotConstants.SERVO_ELBOW_L);
        servoClaw = hardwareMap.get(Servo.class, RobotConstants.SERVO_CLAW);
        servoWrist = hardwareMap.get(Servo.class, RobotConstants.SERVO_WRIST);
        servoPlane = hardwareMap.get(Servo.class, RobotConstants.SERVO_PLANE);

        clawOpen = true;
        wristActive = false;
        elbowActive = false;
        transferStageDeployed = false;
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

        drive = new AutoMecanumDrive(hardwareMap, frontLM, frontRM, backLM, backRM, imu, handler);

        Delay(500);
    }

    public void init() {
        telemetry.addData("Status", "INITIALIZING ROBOT...");
        telemetry.update();

        Delay(2000);

        handler = new NewRobot_v8_AbstractTesting(this, hardwareMap, telemetry);
        InitializeBlock();

        telemetry.addData("Status", "INITIALIZATION COMPLETE!");
        telemetry.update();
    }

    public void loop() {
        // TELEMETRY
        telemetry.addData("Arm Left: ", armL.getCurrentPosition());
        telemetry.addData("Arm Right: ", armR.getCurrentPosition());
        /*telemetry.addData("FrontRM Encoder Value: ", frontRM.getCurrentPosition());
        telemetry.addData("FrontLM Encoder Value: ", frontLM.getCurrentPosition());
        telemetry.addData("BackRM Encoder Value: ", backRM.getCurrentPosition());
        telemetry.addData("BackLM Encoder Value: ", backLM.getCurrentPosition());*/
        telemetry.addData("Target Wrist Position: ", targetWristPosition);
        telemetry.addData("Target Elbow Position: ", targetElbowPosition);
        telemetry.addData("Target Outtake Position: ", targetOuttakePosition);
        telemetry.addData("Target Claw Position: ", targetClawPosition);
        telemetry.addData("Adjustment Allowed: ", adjustmentAllowed);
        telemetry.addData("Field Centric Mode : ", fieldCentricRed ? "RED" : "BLUE");
        telemetry.addData("Current Speed Mode: ", driveSpeedModifier == RobotConstants.BASE_DRIVE_SPEED_MODIFIER ? "BASE SPEED" : "PRECISION MODE");
        //telemetry.addData("IMU Yaw: ", GetHeading());

        telemetry.update();

        // Optional, to see position output
        handler.update();
        MainLoop();
    }
    public void stop() {
        handler.stop();
    }

    public void MainLoop() {

    }

    public void RuntimeConfig() {
        // -------------------------------------------------------------- MANUAL ARM CONTROL (directly effects bot)
        if (adjustmentAllowed) { // lining up arm for topmost cone
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
            } /*else if (gamepad1.right_bumper || gamepad1.left_bumper) {
                armR.setVelocity(0);
                armL.setVelocity(0);
            }*/

            if (gamepad2.square) {
                targetClawPosition -= 0.02;
                servoClaw.setPosition(targetClawPosition);
                Delay(50);
            } else if (gamepad2.circle) {
                targetClawPosition += 0.02;
                servoClaw.setPosition(targetClawPosition);
                Delay(50);
            }

            if (gamepad2.right_bumper) {
                targetWristPosition += 0.02;
                servoWrist.setPosition(targetWristPosition);
                Delay(50);
            } else if (gamepad2.left_bumper) {
                targetWristPosition -= 0.02;
                servoWrist.setPosition(targetWristPosition);
                Delay(50);
            }

            if (gamepad1.triangle) {
                targetElbowPosition += 0.02;
                MoveElbow(targetElbowPosition);
            } else if (gamepad1.cross) {
                targetElbowPosition -= 0.02;
                MoveElbow(targetElbowPosition);
            }

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

            if (gamepad1.dpad_up) {
                intake.setPower(0.5);
            } else {
                intake.setPower(0);
            }
        }

        // -------------------------------------------------------------- CONFIGURATION (don't directly move the bot)

        /*if (gamepad1.x && gamepad1.back) { // toggle red / blue alliance for FCD
            fieldCentricRed = !fieldCentricRed;
            Delay(50);
        }*/

        if (gamepad1.start) { // re-calibrate field centric drive
            imu.resetYaw();
        }
    }

    public void ArmStandby() {
        servoClaw.setPosition(RobotConstants.CLAW_OPEN);
        targetOuttakePosition = RobotConstants.MIN_OUTTAKE_HEIGHT;
        UpdateOuttake(true, 300);
        Delay(350); // elbow should come down after the slide is near done
        MoveElbow(RobotConstants.ELBOW_STANDBY);
        servoWrist.setPosition(RobotConstants.WRIST_STANDBY);
    }

    public void Grab() {
        servoWrist.setPosition(RobotConstants.WRIST_PICKUP);
        MoveElbow(RobotConstants.ELBOW_PICKUP);
        Delay(200);
        //MoveElbow(RobotConstants.ELBOW_STANDBY);
        //Delay(200);
        servoClaw.setPosition(RobotConstants.CLAW_CLOSE);
        Delay(300);
    }

    public void Deposit(int height) {
        // TODO: test if this reinforcement actually works
        //servoClaw.setPosition(RobotConstants.CLAW_CLOSE);
        servoWrist.setPosition(RobotConstants.WRIST_ACTIVE);

        MoveElbow(RobotConstants.ELBOW_ACTIVE);

        Delay(100);

        targetOuttakePosition = height;
        UpdateOuttake(false, 0);
    }

    public void GrabAndDeposit(int height) {
        if (!transferStageDeployed) {
            Grab();
            Deposit(height);
            transferStageDeployed = true;
        } else {
            ArmStandby();
            transferStageDeployed = false;
        }
    }

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
        if ((armL.getCurrentPosition() <= 30 && armR.getCurrentPosition() <= 30) && targetOuttakePosition <= 30) {
            armR.setVelocity(0);
            armL.setVelocity(0);
            resetTimer.reset();
        }
    }
    public void Macros(Abstract handler) {
        // test transfer stage macro
        if (gamepad1.dpad_left) {
            if (!wristActive) {
                wristActive = true;
                servoClaw.setPosition(RobotConstants.CLAW_CLOSE);
                Delay(600);
                servoClaw.setPosition(RobotConstants.CLAW_CLOSE);
                servoWrist.setPosition(RobotConstants.WRIST_ACTIVE);

            } else {
                wristActive = false;
                servoClaw.setPosition(RobotConstants.CLAW_OPEN);
                Delay(200);
                servoWrist.setPosition(RobotConstants.WRIST_STANDBY);
                servoClaw.setPosition(RobotConstants.CLAW_OPEN);
            }
            Delay(200);
        }

        else if (gamepad1.dpad_right && gamepad1.left_bumper) {
            if (!transferStageDeployed) {
                transferStageDeployed = true;
                servoClaw.setPosition(RobotConstants.CLAW_CLOSE);
                Delay(500);
                // TODO: test if this reinforcement actually works
                servoClaw.setPosition(RobotConstants.CLAW_CLOSE);
                servoWrist.setPosition(RobotConstants.WRIST_ACTIVE);

                targetOuttakePosition = RobotConstants.MAX_OUTTAKE_HEIGHT;
                MoveElbow(RobotConstants.ELBOW_ACTIVE);

                Delay(100);

                UpdateOuttake(false, 0);
            } else {
                transferStageDeployed = false;
                servoClaw.setPosition(RobotConstants.CLAW_OPEN);
                Delay(300);

                targetOuttakePosition = RobotConstants.MIN_OUTTAKE_HEIGHT + 1;
                UpdateOuttake(false, 0);
                MoveElbow(RobotConstants.ELBOW_STANDBY);
                servoWrist.setPosition(RobotConstants.WRIST_STANDBY);
            }
        } else if (gamepad1.right_stick_button) {
            //drive.followTrajectory(handler.TILE_TO_BACKDROP);
        } else if (gamepad1.left_stick_button) {
            //drive.followTrajectory(handler.BACKDROP_TO_TILE);
        } else if (gamepad1.dpad_down && gamepad1.left_bumper) {
            GrabAndDeposit(RobotConstants.JUNCTION_LOW);
        } else if (gamepad1.dpad_left && gamepad1.left_bumper) {
            GrabAndDeposit(RobotConstants.JUNCTION_MID);
        } else if (gamepad1.dpad_up && gamepad1.left_bumper) {
            GrabAndDeposit(RobotConstants.JUNCTION_HIGH);
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

        public AutoMecanumDrive(HardwareMap hardwareMap, DcMotorEx frontLM, DcMotorEx frontRM, DcMotorEx backLM, DcMotorEx backRM, IMU i, NewRobot_v8_AbstractTesting handler) {
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
