package org.firstinspires.ftc.teamcode.drive;

import static org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotConstants.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotConstants;
import org.firstinspires.ftc.teamcode.drive.Robotv8.Robotv8_Fullstack;
import org.firstinspires.ftc.teamcode.drive.Robotv8.Robotv8_FullstackTesting;

@Autonomous(name="Autonomous Testing", group="Final")
public class NewRobot_v8_AutoTesting_v1 extends Robotv8_FullstackTesting {

    private double CMtoTicks(double in) {
        return PPR / WHEEL_CIRCUMFERENCE * in;
    }

    private void EncoderMove(double power, double left, double right, boolean strafe, boolean strafeRight, double safetyTimeout) {

        int backLMTarget;
        int frontLMTarget;
        int backRMTarget;
        int frontRMTarget;

        if (!strafe) {
            backLMTarget = backLM.getCurrentPosition() - (int)(CMtoTicks(left));
            frontLMTarget = frontLM.getCurrentPosition() - (int)(CMtoTicks(left));
            backRMTarget = backRM.getCurrentPosition() - (int)(CMtoTicks(right));
            frontRMTarget = frontRM.getCurrentPosition() - (int)(CMtoTicks(right) * PPR);
        }

        else {
            int dir = strafeRight ? 1 : -1;
            backLMTarget = backLM.getCurrentPosition() + (int)(CMtoTicks(left) * dir);
            frontLMTarget = frontLM.getCurrentPosition() - (int)(CMtoTicks(left) * dir);
            backRMTarget = backRM.getCurrentPosition() - (int)(CMtoTicks(right) * dir);
            frontRMTarget = frontRM.getCurrentPosition() + (int)(CMtoTicks(right) * dir);
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

        Delay(5);
    }

    public void MainInit() {

    }

    public void MainStart() {
        //handler.initTask(2);
        // Go back to starting position

        GrabAndReady();
        EncoderMove(0.8, 87, 87, false, false, 5);
        //Turn(90);
        EncoderMove(0.8, -35, 35, false, false, 5);
        EncoderMove(0.8, -85, -85, false, false, 6);
        //RaiseAndPrime(JUNCTION_LOW);
        Delay(1000);
        DropAndReset();
    }

    public void MainLoop() {
        // Get Pixel
        // Go to
    }
}
