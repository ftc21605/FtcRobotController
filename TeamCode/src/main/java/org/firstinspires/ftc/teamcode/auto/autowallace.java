package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.hardware.navx;

@Autonomous(name = "Autonomous Wallace", group = "Wallace")
@Disabled
public class autowallace extends LinearOpMode {

    DriveTrain drive = new DriveTrain(this);

    @Override
    public void runOpMode() {
        drive.init();
	//	navx.init();
        // telemetry.addData("Starting at", "%7d :%7d",
        //         leftFrontDrive.getCurrentPosition(),
        //         rightFrontDrive.getCurrentPosition());
        //BlueFinder.Selected selected;
        // here is what happens after we hit start
        telemetry.addData(">", "Press Start to run");
        telemetry.update();
        while (!isStarted() && !isStopRequested()) {
        }
        drive.setDrivePower(0.2, 0.2, 0.2, 0.2);
        while (drive.getCurrentLeftFrontPosition() < 1000) {
            sleep(10);
            telemetry.addData("currpos:", "%10d", drive.getCurrentLeftFrontPosition());
            telemetry.update();

        }
        drive.off();
    }

}


