package org.firstinspires.ftc.teamcode.drive.Deprecated;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.Deprecated.AC2301A_AutoPixelScorePark;
import org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotAlliance;

//@Autonomous(name="AutoBlue1", group="Final")
@Deprecated
public class AC2301B_AutoPixelScoreParkBlue extends AC2301A_AutoPixelScorePark {
    public ElapsedTime autoTimer = new ElapsedTime();

    public void MainInit() {
        alliance = RobotAlliance.BLUE;
    }
}
