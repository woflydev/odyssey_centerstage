package org.firstinspires.ftc.teamcode.Deprecated;

import com.qualcomm.robotcore.util.ElapsedTime;

//@Autonomous(name="AutoBlue1", group="Final")
@Deprecated
public class AC2301B_AutoPixelScoreParkBlue extends AC2301A_AutoPixelScorePark {
    public ElapsedTime autoTimer = new ElapsedTime();

    public void MainInit() {
        alliance = RobotAlliance.BLUE;
    }
}
