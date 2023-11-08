package org.firstinspires.ftc.teamcode.drive;

import static org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotConstants.ENCODER_TICKS_PER_TILE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.FSM_Outtake;
import org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotAlliance;
import org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotConstants;
import org.firstinspires.ftc.teamcode.drive.Robotv8.Robotv8_FSM_Fullstack;

@Autonomous(name="AutoBlue1", group="Final")
public class Robotv8_AutoPixelScoreParkBlue extends Robotv8_AutoPixelScorePark {
    public ElapsedTime autoTimer = new ElapsedTime();

    public void MainInit() {
        alliance = RobotAlliance.BLUE;
    }
}
