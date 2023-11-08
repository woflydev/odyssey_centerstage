package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotAlliance;

@Autonomous(name="AutoBlue1", group="Final")
public class AutoPixelScoreParkBlue extends AutoPixelScorePark {
    public ElapsedTime autoTimer = new ElapsedTime();

    public void MainInit() {
        alliance = RobotAlliance.BLUE;
    }
}
