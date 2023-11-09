package org.firstinspires.ftc.teamcode.Deprecated.VeryDeprecated;

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

@Deprecated()
public class NewRobot_v8_TransferServoTest_v1 extends OpMode {
    // -------------------------------------------------------------- SYSTEM VAR
    private DcMotorEx backLM = null;
    private DcMotorEx backRM = null;
    private DcMotorEx frontLM = null;
    private DcMotorEx frontRM = null;
    private Servo servoClaw = null;
    private Servo servoWrist = null;
    private IMU imu = null;

    private final ElapsedTime encoderRuntime = new ElapsedTime();
    private final ElapsedTime armRuntime = new ElapsedTime();
    private final ElapsedTime resetTimer = new ElapsedTime();

    private double targetClawPosition = 0.4;
    private double targetWristPosition = 0;
    private boolean clawOpen = false;
    private boolean transferStageActive = false;

    private double current_v1 = 0;
    private double current_v2 = 0;
    private double current_v3 = 0;
    private double current_v4 = 0;

    private double driveSpeedModifier = 1;

    private boolean adjustmentAllowed = true;

    private boolean fieldCentricRed = true;

    // -------------------------------------------------------------- ROBOT CONFIG

    private static final String FRONT_LEFT = "frontL";
    private static final String FRONT_RIGHT = "frontR";
    private static final String BACK_LEFT = "backL";
    private static final String BACK_RIGHT = "backR";
    private static final String ARM_R = "armR";
    private static final String ARM_L = "armL";
    private static final String SERVO_CLAW = "claw";
    private static final String SERVO_WRIST = "wrist";
    private static final String HUB_IMU = "imu";

    private static final int MAX_ARM_HEIGHT = 4000; // TODO: CHANGE THIS BACK TO 4300 when stable
    private static final int MIN_ARM_HEIGHT = 0;
    private static final int ARM_ADJUSTMENT_INCREMENT = 50;
    private static final int ARM_BOOST_MODIFIER = 1;
    private static final int ARM_RESET_TIMEOUT = 3;

    private static final double CLAW_CLOSE = 0;
    private static final double CLAW_OPEN = 0.2;
    private static final double WRIST_STANDBY = 0.8;
    private static final double WRIST_ACTIVE = 0.4;

    private static final double MAX_ACCELERATION_DEVIATION = 10; // higher = less smoothing
    private static final double BASE_DRIVE_SPEED_MODIFIER = 1; // higher = less speed
    private static final double PRECISION_DRIVE_SPEED_MODIFIER = 3.35;

    private static final double PPR = 537.7; // gobuilda motor 85203 Series

    // -------------------------------------------------------------- JUNCTION PRESETS

    private static final int JUNCTION_OFF = 30; // will change
    private static final int JUNCTION_LOW = 1650;
    private static final int JUNCTION_MID = 2700;
    private static final int JUNCTION_STANDBY = 3200;
    private static final int JUNCTION_HIGH = 4000;

    // -------------------------------------------------------------- ROBOT OPERATION

    private void Mecanum() {
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

    private void RuntimeConfig() {
        // -------------------------------------------------------------- MANUAL ARM CONTROL (directly effects bot)

        if (adjustmentAllowed) { // lining up arm for topmost cone
            /*if (gamepad1.left_bumper) {
                if (clawOpen) {
                    clawOpen = false;
                    servoClaw.setPosition(CLAW_CLOSE);
                } else {
                    clawOpen = true;
                    servoClaw.setPosition(CLAW_OPEN);
                }

                Delay(100);
            }*/

            if (gamepad1.left_bumper) {
                targetClawPosition -= 0.05;
                servoClaw.setPosition(targetClawPosition);
                Delay(50);
            } else if (gamepad1.right_bumper) {
                targetClawPosition += 0.05;
                servoClaw.setPosition(targetClawPosition);
                Delay(50);
            }

            if (gamepad1.right_trigger >= 0.6) {
                targetWristPosition += 0.05;
                servoWrist.setPosition(targetWristPosition);
                Delay(50);
            } else if (gamepad1.left_trigger >= 0.6) {
                targetWristPosition -= 0.05;
                servoWrist.setPosition(targetWristPosition);
                Delay(50);
            }


        }

        // -------------------------------------------------------------- CONFIGURATION (don't directly move the bot)

        if (gamepad1.x && gamepad1.back) { // toggle red / blue alliance for FCD
            fieldCentricRed = !fieldCentricRed;
            Delay(50);
        }

        if (gamepad1.start) { // re-calibrate field centric drive
            imu.resetYaw();
        }

        else {
            driveSpeedModifier = BASE_DRIVE_SPEED_MODIFIER;
        }
    }

    private void Macros() {
        // test transfer stage macro
        if (gamepad1.cross) {
            if (!transferStageActive) {
                transferStageActive = true;
                servoClaw.setPosition(CLAW_CLOSE);
                Delay(800);
                servoClaw.setPosition(CLAW_CLOSE);
                servoWrist.setPosition(WRIST_ACTIVE);

            } else {
                transferStageActive = false;
                servoClaw.setPosition(CLAW_OPEN);
                Delay(200);
                servoWrist.setPosition(WRIST_STANDBY);
                servoClaw.setPosition(CLAW_OPEN);
            }
            Delay(200);
        }
    }

    // -------------------------------------------------------------- USER FUNCTIONS

    public static boolean IsPositive(double d) { return !(Double.compare(d, 0.0) < 0); }

    private void Delay(double time) {
        try { sleep((long)time); } catch (Exception e) { System.out.println("interrupted"); }
    }

    private double Stabilize(double new_accel, double current_accel) {
        double dev = new_accel - current_accel;
        return Math.abs(dev) > MAX_ACCELERATION_DEVIATION ? current_accel + MAX_ACCELERATION_DEVIATION * dev / Math.abs(dev) : new_accel;
    }

    private double GetHeading() {
        double currentHeading = imu.getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
        double rot = (double)(Math.round(-currentHeading + 720) % 360);
        rot = rot == 0 ? 360 : rot;
        return rot;
    }

    private void EncoderMove(double power, double left, double right, boolean strafe, boolean strafeRight, double safetyTimeout) {

        int backLMTarget;
        int frontLMTarget;
        int backRMTarget;
        int frontRMTarget;

        if (!strafe) {
            backLMTarget = backLM.getCurrentPosition() - (int)(left * PPR);
            frontLMTarget = frontLM.getCurrentPosition() - (int)(left * PPR);
            backRMTarget = backRM.getCurrentPosition() - (int)(right * PPR);
            frontRMTarget = frontRM.getCurrentPosition() - (int)(right * PPR);
        }

        else {
            int dir = strafeRight ? 1 : -1;
            backLMTarget = backLM.getCurrentPosition() + (int)(left * PPR * dir);
            frontLMTarget = frontLM.getCurrentPosition() - (int)(left * PPR * dir);
            backRMTarget = backRM.getCurrentPosition() - (int)(right * PPR * dir);
            frontRMTarget = frontRM.getCurrentPosition() + (int)(right * PPR * dir);
        }

        backLM.setTargetPosition(backLMTarget);
        frontLM.setTargetPosition(frontLMTarget);
        backRM.setTargetPosition(backRMTarget);
        frontRM.setTargetPosition(frontRMTarget);

        backLM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRM.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        backLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        encoderRuntime.reset();
        backLM.setPower(Math.abs(power));
        frontLM.setPower(Math.abs(power));
        backRM.setPower(Math.abs(power));
        frontRM.setPower(Math.abs(power));

        while ((encoderRuntime.seconds() <= safetyTimeout) && (backRM.isBusy() && backLM.isBusy())) {
            telemetry.clear();
            telemetry.addData("CURRENT COORDINATE: ",  "%7d :%7d", backLM.getCurrentPosition(), backRM.getCurrentPosition());
            telemetry.update();
        }

        backLM.setPower(0);
        frontLM.setPower(0);
        backRM.setPower(0);
        frontRM.setPower(0);

        backLM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Delay(50);
    }

    private void EncoderTransform(double power, double left, double right, boolean useIMU, double absoluteTargetRot, double safetyTimeout) {
        if (useIMU) { // use the IMU for accurate rotations
            backLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // if this floats, will overshoot
            backLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            encoderRuntime.reset();

            while (encoderRuntime.seconds() <= safetyTimeout) {
                // funny kelvin code fixed spastic robot
                double margin = (absoluteTargetRot - GetHeading() + 360 * 10) % 360;
                double dir = (margin > 180) ? 1 : -1;

                if (Math.abs(margin) <= 8) break;

                backLM.setPower(power * dir);
                frontLM.setPower(power * dir);
                backRM.setPower(-power * dir);
                frontRM.setPower(-power * dir);

                margin = (absoluteTargetRot - GetHeading() + 360 * 10) % 360;
                if (Math.abs(margin) <= 5) break;

                telemetry.clear();
                telemetry.addData("Current Rotation: ", GetHeading());
                telemetry.update();
            }

            backLM.setPower(0);
            frontLM.setPower(0);
            backRM.setPower(0);
            frontRM.setPower(0);

            backLM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontLM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontRM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            Delay(55);

            backLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            frontLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            backRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            frontRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        else {
            // new transformation algorithm with IMU turning
            int backLMTarget = backLM.getCurrentPosition() - (int)(left * PPR);
            int frontLMTarget = frontLM.getCurrentPosition() - (int)(left * PPR);
            int backRMTarget = backRM.getCurrentPosition() - (int)(right * PPR);
            int frontRMTarget = frontRM.getCurrentPosition() - (int)(right * PPR);

            backLM.setTargetPosition(backLMTarget);
            frontLM.setTargetPosition(frontLMTarget);
            backRM.setTargetPosition(backRMTarget);
            frontRM.setTargetPosition(frontRMTarget);

            backLM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontLM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRM.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            encoderRuntime.reset();
            backLM.setPower(Math.abs(power));
            frontLM.setPower(Math.abs(power));
            backRM.setPower(Math.abs(power));
            frontRM.setPower(Math.abs(power));

            while ((encoderRuntime.seconds() <= safetyTimeout) && (backRM.isBusy() && backLM.isBusy())) {
                telemetry.clear();
                telemetry.addData("CURRENT COORDINATE: ",  "%7d :%7d", backLM.getCurrentPosition(), backRM.getCurrentPosition());
                telemetry.update();
            }
        }

        backLM.setPower(0);
        frontLM.setPower(0);
        backRM.setPower(0);
        frontRM.setPower(0);

        backLM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Delay(50);
    }

    // -------------------------------------------------------------- MAIN INIT & LOOP

    private void InitializeBlock() {
        driveSpeedModifier = BASE_DRIVE_SPEED_MODIFIER;

        /*backLM = hardwareMap.get(DcMotorEx.class, BACK_LEFT);
        backRM = hardwareMap.get(DcMotorEx.class, BACK_RIGHT);

        frontLM = hardwareMap.get(DcMotorEx.class, FRONT_LEFT);
        frontRM = hardwareMap.get(DcMotorEx.class, FRONT_RIGHT); //frontRM.setDirection(DcMotorSimple.Direction.REVERSE); // weird workaround Stanley put in

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
        backRM.setDirection(DcMotorSimple.Direction.REVERSE);*/

        servoClaw = hardwareMap.get(Servo.class, SERVO_CLAW);
        servoWrist = hardwareMap.get(Servo.class, SERVO_WRIST);

        clawOpen = true;
        transferStageActive = false;
        servoClaw.setPosition(CLAW_OPEN);
        servoWrist.setPosition(WRIST_STANDBY);

        // -------------------------------------------------------------- IMU INIT

        telemetry.addData("Status", "CALIBRATING IMU...");
        telemetry.addData("Important Information", "PLACE ROBOT FACING AWAY FROM ALLIANCE BOX!");
        telemetry.update();

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));

        imu = hardwareMap.get(IMU.class, HUB_IMU);
        imu.initialize(parameters);
        imu.resetYaw();

        Delay(1000);
    }

    public void init() {
        telemetry.addData("Status", "INITIALIZING ROBOT...");    //
        telemetry.update();

        InitializeBlock();

        telemetry.setAutoClear(false);
        telemetry.addData("Status", "INITIALIZATION COMPLETE!");
        telemetry.update();
    }

    public void loop() {
        telemetry.clear();

        // -------------------------------------------------------------- DRIVE AND MANUAL OVERRIDES

        RuntimeConfig();
        //PassiveArmResetCheck();
        Macros();
        //Mecanum();

        // -------------------------------------------------------------- TELEMETRY

        /*telemetry.addData("FrontRM Encoder Value: ", frontRM.getCurrentPosition());
        telemetry.addData("FrontLM Encoder Value: ", frontLM.getCurrentPosition());
        telemetry.addData("BackRM Encoder Value: ", backRM.getCurrentPosition());
        telemetry.addData("BackLM Encoder Value: ", backLM.getCurrentPosition());*/
        telemetry.addData("target claw pos: ", targetClawPosition);
        telemetry.addData("Claw Open? ", clawOpen ? "YES" : "NO");
        telemetry.addData("Target Wrist Position: ", targetWristPosition);
        telemetry.addData("Adjustment Allowed: ", adjustmentAllowed);
        telemetry.addData("Field Centric Mode : ", fieldCentricRed ? "RED" : "BLUE");
        telemetry.addData("Current Speed Mode: ", driveSpeedModifier == BASE_DRIVE_SPEED_MODIFIER ? "BASE SPEED" : "PRECISION MODE");
        telemetry.addData("IMU Yaw: ", GetHeading());

        telemetry.update();
    }
}