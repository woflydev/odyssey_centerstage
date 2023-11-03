package org.firstinspires.ftc.teamcode.drive;

import static org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotConstants.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.FSM_Outtake;
import org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotConstants;

@Autonomous(name="Autonomous Testing", group="Final")
public class NewRobot_v8_Auto_v1 extends NewRobot_v8_FSM_FullRobot_v3 {

    private double TilesToTicks(double input) {
        return ENCODER_TICKS_PER_TILE * input;
    }

    // this function is removed from the FSM
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

    // runs on start press, only once
    public void MainStart() {
        //handler.initTask(2);

        EncoderMove(0.6, 2.3, 2.3, false, false, 4);
        Delay(100);
        EncoderMove(0.8, -1, 1, false, false, 2);
        Delay(100);
        EncoderMove(0.8, 3, 3, false, false, 5);
        Delay(100);

        intake.setPower(0.6);
        Delay(2000);
        intake.setPower(0);

        EncoderMove(0.8, -4.8, -4.8, false, false, 5);

        /*GrabAndReady();
        EncoderMove(0.8, 1, 1, false, false, 5);
        Delay(50);
        EncoderMove(0.8, -1, 1, false, false, 4);
        Delay(100);
        EncoderMove(1, -1.65, -1.65, false, false, 4);
        Delay(100);

        RaiseAndPrime(100);

        //RaiseAndPrime(JUNCTION_LOW);
        Delay(500);
        DropAndReset();
        Delay(500);

        //EncoderMove(1, 1, 1, true, false, 5);
        EncoderMove(0.7, -0.3, -0.3, false, false, 2);
        //EncoderMove(1, 1.7, 1.7, false, false, 5); //NOTE: added onto next one
        EncoderMove(0.75, 4.4, 4.4, false, false, 5);

        intake.setPower(0.5);
        Delay(2000);
        intake.setPower(0);*/


    }

    public void MainLoop() {
        // Get Pixel
        // Go to
    }
}
