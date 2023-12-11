package org.firstinspires.ftc.teamcode.drive.Robotv8;

import static java.lang.Thread.sleep;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.FSM_Drivetrain;
import org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.FSM_Outtake;
import org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.FSM_PlaneLauncher;
import org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotConstants;
import org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotState;

public class FSM_TeleOp_Fullstack extends OpMode {
    public RobotState state = RobotState.IDLE;
    public FSM_Outtake outtakeState = FSM_Outtake.ACTIVATED;
    public FSM_PlaneLauncher planeLauncherState = FSM_PlaneLauncher.IDLE;
    public FSM_Drivetrain drivetrainState = FSM_Drivetrain.MANUAL;

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
    public Servo servoWhateverTheFuckThatThingIs = null;
    public CRServo servoHangR = null;
    public CRServo servoHangL = null;
    public DcMotorEx armR = null;
    public DcMotorEx armL = null;
    public IMU imu = null;
    public DcMotorEx intake = null;

    public final ElapsedTime encoderRuntime = new ElapsedTime();
    public final ElapsedTime armRuntime = new ElapsedTime();
    public final ElapsedTime resetTimer = new ElapsedTime();
    public final ElapsedTime drivetrainTimer = new ElapsedTime();
    public final ElapsedTime outtakeFSMTimer = new ElapsedTime();
    public final ElapsedTime planeLauncherFSMTimer = new ElapsedTime();

    public double targetClawPosition = RobotConstants.CLAW_OPEN;
    public double targetWristPosition = RobotConstants.WRIST_PICKUP;
    public double targetElbowPosition = RobotConstants.ELBOW_STANDBY;
    public double targetPlanePosition = RobotConstants.PLANE_STANDBY;
    public double targetFlapPosition = RobotConstants.FLAP_CLOSE;
    public int targetOuttakePosition = 0;

    public boolean clawOpen = false;
    public boolean wristActive = false;
    public boolean elbowActive = false;
    public boolean transferStageDeployed = false;
    public boolean hangStabilizationDeployed = false;

    public double current_v1 = 0;
    public double current_v2 = 0;
    public double current_v3 = 0;
    public double current_v4 = 0;

    public double driveSpeedModifier = 1;
    public boolean adjustmentAllowed = true;
    public boolean fieldCentricRed = true;

    public boolean DANGER_MANUAL_OUTTAKE = false;

    public void InitializeBlock() {
        // NOTE: giant initialization block stored here instead of directly in init.
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
        servoWhateverTheFuckThatThingIs = hardwareMap.get(Servo.class, RobotConstants.SERVO_WHATEVER_THE_FUCK_THAT_THING_IS);

        servoHangR = hardwareMap.get(CRServo.class, RobotConstants.SERVO_HANG_R);
        servoHangR.setDirection(DcMotorSimple.Direction.FORWARD);
        servoHangR.setPower(0);

        servoHangL = hardwareMap.get(CRServo.class, RobotConstants.SERVO_HANG_L);
        servoHangL.setDirection(DcMotorSimple.Direction.REVERSE);
        servoHangL.setPower(0);

        clawOpen = true;
        transferStageDeployed = false;
/*        MoveElbow(RobotConstants.ELBOW_STANDBY);
        servoFlap.setPosition(RobotConstants.FLAP_CLOSE);
        servoClaw.setPosition(RobotConstants.CLAW_OPEN);
        servoWrist.setPosition(RobotConstants.WRIST_STANDBY);
        servoPlane.setPosition(RobotConstants.PLANE_STANDBY);*/

        servoWhateverTheFuckThatThingIs.setPosition(RobotConstants.WHATEVER_THE_FUCK_THAT_THING_IS_ON);

        // -------------------------------------------------------------- IMU INIT

        telemetry.addData("Status", "CALIBRATING IMU...");
        telemetry.addData("Important Information", "PLACE ROBOT FACING AWAY FROM ALLIANCE BOX!");
        telemetry.update();

        //InitCameras();

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));

        imu = hardwareMap.get(IMU.class, RobotConstants.HUB_IMU);
        imu.initialize(parameters);
        imu.resetYaw();

        Delay(100);
    }

    // NOTE: SYSTEM METHODS ------------------------------------------------------------------
    public void init() {
        telemetry.addData("Status", "INITIALIZING ROBOT...");
        telemetry.update();

        InitializeBlock();
        MainInit();

        telemetry.addData("Status", "INITIALIZATION COMPLETE!"); // note: may be overwritten by camera opening slower
        telemetry.update();
    }
    public void start() {

        MoveElbow(RobotConstants.ELBOW_STANDBY);
        servoFlap.setPosition(RobotConstants.FLAP_CLOSE);
        servoClaw.setPosition(RobotConstants.CLAW_OPEN);
        servoWrist.setPosition(RobotConstants.WRIST_STANDBY);
        servoPlane.setPosition(RobotConstants.PLANE_STANDBY);

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
        //handler.stop();
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
        // NOTE: Basic robot telemetry is handled here, instead of child classes.
        telemetry.addData("DANGER_OUTTAKE_MODE", DANGER_MANUAL_OUTTAKE);
        telemetry.addData("Arm Left: ", armL.getCurrentPosition());
        telemetry.addData("Arm Right: ", armR.getCurrentPosition());
        telemetry.addData("IMU Raw: ", GetHeadingRaw());
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

    // NOTE: CUSTOM BEHAVIOUR ----------------------------------------------------------------
    public void Mecanum() {
        // NOTE: field centric Mecanum drive, tuned for acceleration curves and manual control.
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

        frontLM.setPower(stable_v3 / driveSpeedModifier);
        frontRM.setPower(stable_v2 / driveSpeedModifier);
        backLM.setPower(stable_v1 / driveSpeedModifier);
        backRM.setPower(stable_v4 / driveSpeedModifier);
    }

    public void DrivetrainSubsystem() {
        switch (drivetrainState) {
            case MANUAL:
                Mecanum();
                if (gamepad1.dpad_right) {
                    Delay(100);

                    drivetrainTimer.reset();
                    drivetrainState = FSM_Drivetrain.ALIGNING_WITH_BACKDROP;

                } else if (gamepad1.dpad_left) {
                    Delay(100);

                    drivetrainTimer.reset();
                    drivetrainState = FSM_Drivetrain.ALIGNING_WITH_OUTER_WALL;
                }
                break;
            case ALIGNING_WITH_BACKDROP:
                // TODO: this might break, test later
                Mecanum();
                TurnToDirection(0.5, 90); // note: automatically switches drivetrainState back to manual when done
                HandleDrivetrainOverride(); // note: also resets to manual if overridden
            case ALIGNING_WITH_OUTER_WALL:
                Mecanum();
                TurnToDirection(0.5, 35);
                HandleDrivetrainOverride();
        }
    }

    public void OuttakeSubsystem() {
        // NOTE: statemachine for outtake sequences
        switch (outtakeState) {
            case ACTIVATED:
                if (gamepad1.left_bumper) {
                    servoFlap.setPosition(RobotConstants.FLAP_OPEN);
                    outtakeFSMTimer.reset();

                    outtakeState = FSM_Outtake.FLAP_OPENING;
                }
                break;
            case FLAP_OPENING:
                // amount of time the servo takes to activate from the previous state
                if (outtakeFSMTimer.milliseconds() >= 300) {
                    servoWrist.setPosition(RobotConstants.WRIST_PICKUP);
                    MoveElbow(RobotConstants.ELBOW_STANDBY);
                    outtakeFSMTimer.reset();

                    outtakeState = FSM_Outtake.WRIST_PICKING;
                }
                break;
            case WRIST_PICKING:
                if (outtakeFSMTimer.milliseconds() >= 150) {
                    MoveElbow(RobotConstants.ELBOW_PICKUP);
                    outtakeFSMTimer.reset();

                    outtakeState = FSM_Outtake.ELBOW_PICKING;
                }
                break;
            case ELBOW_PICKING:
                if (outtakeFSMTimer.milliseconds() >= 100) {
                    servoClaw.setPosition(RobotConstants.CLAW_CLOSE);
                    outtakeFSMTimer.reset();

                    outtakeState = FSM_Outtake.CLAW_CLOSING;
                }
                break;
            case CLAW_CLOSING:
                if (outtakeFSMTimer.milliseconds() >= 150) {
                    outtakeFSMTimer.reset();
                    //servoWrist.setPosition(RobotConstants.WRIST_STANDBY);
                    servoFlap.setPosition(RobotConstants.FLAP_OPEN);
                    MoveElbow(RobotConstants.ELBOW_STANDBY);
                    outtakeState = FSM_Outtake.GRABBED_AND_READY;
                }
                break;
            case GRABBED_AND_READY:
                HandleDeposit(); // different deposit heights, the outtake progression is set within this function
                break;
            case PRIMED_FOR_DEPOSIT:
                // acts as a stopper to suspend the statemachine until a button is pressed
                if (gamepad1.cross || gamepad1.circle || gamepad1.triangle) {
                    outtakeState = FSM_Outtake.CLAW_OPENING;
                    Delay(100); // debounce input
                }
                break;
            case CLAW_OPENING:
                servoClaw.setPosition(RobotConstants.CLAW_OPEN);
                servoWrist.setPosition(RobotConstants.WRIST_ACTIVE * 1.12); // note: wrist down to normalize deposit
                Delay(100);
                outtakeFSMTimer.reset();
                outtakeState = FSM_Outtake.OUTTAKE_RESET;
                break;
            case OUTTAKE_RESET:
                if (outtakeFSMTimer.milliseconds() >= 600 && outtakeFSMTimer.milliseconds() <= 1500) {
                    servoFlap.setPosition(RobotConstants.FLAP_CLOSE);
                    servoWrist.setPosition(RobotConstants.WRIST_STANDBY);
                    MoveElbow(RobotConstants.ELBOW_STANDBY);

                    Delay(100);
                    servoFlap.setPosition(RobotConstants.FLAP_CLOSE);
                    targetOuttakePosition = 10;
                    UpdateOuttake(true, 0);

                    outtakeState = FSM_Outtake.ACTIVATED;
                }
                break;
        }
    }

    public void PlaneLauncherSubsystem() {
        switch (planeLauncherState) {
            case IDLE:
                if (gamepad2.cross) {
                    servoWhateverTheFuckThatThingIs.setPosition(RobotConstants.WHATEVER_THE_FUCK_THAT_THING_IS_OFF);
                    Delay(200);
                    servoPlane.setPosition(RobotConstants.PLANE_ACTIVE);
                    planeLauncherFSMTimer.reset();

                    planeLauncherState = FSM_PlaneLauncher.ACTIVE;
                }
                break;
            case ACTIVE:
                if (planeLauncherFSMTimer.seconds() >= 2) {
                    servoWhateverTheFuckThatThingIs.setPosition(RobotConstants.WHATEVER_THE_FUCK_THAT_THING_IS_ON);
                    servoPlane.setPosition(RobotConstants.PLANE_STANDBY);

                    planeLauncherState = FSM_PlaneLauncher.IDLE;
                }
                break;
        }
    }

    public void RuntimeConfig() {
        // NOTE: hanging stabilization macro
        if (gamepad1.right_trigger > 0.1 && gamepad1.triangle) {
            HandleHangStabilization();
        }

        // NOTE: manual control logic. slides / claw / hanging / flap / plane / elbow
        else if (adjustmentAllowed) {
            // slow down driving with analog trigger
            if (gamepad1.right_trigger >= 0.3) {
                driveSpeedModifier = gamepad1.right_trigger + 1.6;
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
                targetWristPosition -= 0.02;
                servoWrist.setPosition(targetWristPosition);
                Delay(50);
            } else if (gamepad2.dpad_up) {
                targetWristPosition += 0.02;
                servoWrist.setPosition(targetWristPosition);
                Delay(50);
            }

            // claw
            if (gamepad2.square) {
                targetClawPosition -= 0.02;
                servoClaw.setPosition(targetClawPosition);
                Delay(50);
            } else if (gamepad2.circle) {
                targetClawPosition += 0.02;
                servoClaw.setPosition(targetClawPosition);
                Delay(50);
            }

            // note: relinquish wrist control in favour of hanging
            /*if (gamepad2.right_bumper) {
                targetWristPosition += 0.02;
                servoWrist.setPosition(targetWristPosition);
                Delay(50);
            } else if (gamepad2.left_bumper) {
                targetWristPosition -= 0.02;
                servoWrist.setPosition(targetWristPosition);
                Delay(50);
            }*/

            if (gamepad2.right_bumper) {
                servoHangR.setPower(1);
                servoHangL.setPower(1);
            } else if (gamepad2.left_bumper) {
                servoHangR.setPower(-1);
                servoHangL.setPower(-1);
            } else {
                servoHangR.setPower(0);
                servoHangL.setPower(0);
            }

            // FLAP (FOR TUNING VALUES) -----------------------------------------------
            if (gamepad2.dpad_right) {
                targetFlapPosition += 0.02;
                servoFlap.setPosition(targetFlapPosition);
            } else if (gamepad2.dpad_left) {
                targetFlapPosition -= 0.02;
                servoFlap.setPosition(targetFlapPosition);
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

        /*if (gamepad2.start && gamepad2.back) {
            if (DANGER_MANUAL_OUTTAKE) {
                armR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                targetOuttakePosition = 10;

                armR.setVelocity(0);
                armL.setVelocity(0);

                UpdateOuttake(true, 0);

                DANGER_MANUAL_OUTTAKE = false;
            } else {
                DANGER_MANUAL_OUTTAKE = true;
            }
            Delay(200);
        }*/

        // NOTE: INTAKE
        if (gamepad1.left_trigger > 0.2 || gamepad2.triangle) {
            intake.setPower(RobotConstants.MAX_MANUAL_INTAKE_POWER);
        } else if (gamepad1.square) {
            intake.setPower(-RobotConstants.MAX_MANUAL_INTAKE_POWER);
        } else {
            intake.setPower(0);
        }

        // NOTE: MANUAL OUTTAKE RESET
        if (gamepad1.right_bumper && !(outtakeState == FSM_Outtake.PRIMED_FOR_DEPOSIT) && !hangStabilizationDeployed) {
            intake.setPower(0);
            servoClaw.setPosition(RobotConstants.CLAW_OPEN);
            MoveElbow(RobotConstants.ELBOW_STANDBY);
            Delay(350);
            outtakeState = FSM_Outtake.CLAW_OPENING; // state to open claw and completely reset the outtake
        }

        // NOTE: FIELD CENTRIC IMU YAW RESET
        if (gamepad1.start) { // re-calibrate field centric drive
            imu.resetYaw();
        }
    }

    public void HandleDeposit() {
        if (gamepad1.cross) {
            RaiseAndPrime(RobotConstants.JUNCTION_LOW);
            outtakeState = FSM_Outtake.PRIMED_FOR_DEPOSIT;
            Delay(100);
        } else if (gamepad1.circle) {
            RaiseAndPrime(RobotConstants.JUNCTION_MID);
            outtakeState = FSM_Outtake.PRIMED_FOR_DEPOSIT;
            Delay(100);
        } else if (gamepad1.triangle) {
            RaiseAndPrime(RobotConstants.JUNCTION_HIGH);
            outtakeState = FSM_Outtake.PRIMED_FOR_DEPOSIT;
            Delay(100);
        }
    }

    public void RaiseAndPrime(int height) {
        intake.setPower(0); // make sure intake is not running

        MoveElbow(RobotConstants.ELBOW_ACTIVE);

        targetOuttakePosition = height;
        UpdateOuttake(false, 0);

        servoWrist.setPosition(RobotConstants.WRIST_ACTIVE);
        servoClaw.setPosition(RobotConstants.CLAW_CLOSE);

        outtakeState = FSM_Outtake.PRIMED_FOR_DEPOSIT;
        Delay(50); // debounce

        //servoFlap.setPosition(RobotConstants.FLAP_CLOSE); // note: moved into the subsystem - this has to go later, because axons are fast
    }

    public void HandleDrivetrainOverride() {
        // note: override in case things go die die
        if (gamepad1.dpad_left || gamepad1.dpad_right) {
            backLM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontLM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontRM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            backLM.setPower(0);
            backRM.setPower(0);
            frontLM.setPower(0);
            frontRM.setPower(0);

            drivetrainTimer.reset();
            drivetrainState = FSM_Drivetrain.MANUAL;
        }
    }

    // NOTE: HELPER METHODS ------------------------------------------------------------------
    public void HandleHangStabilization() {
        if (hangStabilizationDeployed) {
            servoClaw.setPosition(RobotConstants.CLAW_OPEN);
            servoWrist.setPosition(RobotConstants.WRIST_STANDBY_BACK);
            hangStabilizationDeployed = false;
            Delay(500);
        } else {
            servoClaw.setPosition(RobotConstants.CLAW_OPEN);
            MoveElbow(RobotConstants.ELBOW_HANG_STABILIZATION);
            servoWrist.setPosition(RobotConstants.WRIST_HANG_STABILIZATION);
            Delay(800);
            servoClaw.setPosition(RobotConstants.CLAW_CLOSE);
            hangStabilizationDeployed = true;
            Delay(500);
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

    public void MoveElbow(double targetPos) {
        servoElbowR.setPosition(targetPos);
        servoElbowL.setPosition(1 - targetPos); // Set to the opposite position
    }

    public void TurnToDirection(double speed, double desiredHeading) {
        backLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double currentHeading = imu.getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
        double error = desiredHeading - currentHeading;

        // Ensure that the error is within the range -180 to 180 degrees
        if (error > 180) {
            error -= 360;
        } else if (error < -180) {
            error += 360;
        }

        double power = error > 0 ? speed : -speed; // which turning direction is closest?

        if (Math.abs(error) < 1.0) {
            backLM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontLM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontRM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            backLM.setPower(0);
            backRM.setPower(0);
            frontLM.setPower(0);
            frontRM.setPower(0);

            drivetrainState = FSM_Drivetrain.MANUAL;
            return;
        }

        if (drivetrainTimer.seconds() <= 6) {
            backLM.setPower(-power);
            backRM.setPower(power);
            frontLM.setPower(-power);
            frontRM.setPower(power);
        } else {
            backLM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontLM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontRM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            drivetrainState = FSM_Drivetrain.MANUAL;
        }
    }

    public void PassiveArmResetCheck() {
        if (!DANGER_MANUAL_OUTTAKE) {
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

    }

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

    public double GetHeadingRaw() {
        return imu.getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
    }
}
