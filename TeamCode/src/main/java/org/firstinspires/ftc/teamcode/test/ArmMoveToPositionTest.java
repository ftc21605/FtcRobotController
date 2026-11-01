package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Arm;

@TeleOp(name = "Test: Arm Move Position Test", group = "Test")
//@Disabled
public class ArmMoveToPositionTest extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();
    Arm arm = new Arm(this);
    double savepower = 0.;
    boolean apushed = false;
    boolean bpushed = false;

    @Override
    public void runOpMode() {

        arm.init();
        telemetry.addData(">", "Press Start to run.");
        telemetry.update();
        waitForStart();

        double slidepower = 0;

        // Scan servo till stop pressed.
        while (opModeIsActive()) {
            double armpower = 0;
            int armposition = arm.getCurrentPosition();
	    if (gamepad1.a)
		{
		    if (!apushed)
			{
		    arm.MoveTo(arm.getCurrentPosition()+1000);
		    apushed = true;
			}
		    
		}
	    else
		{
		    apushed = false;
		}
	    if (gamepad1.b)
		{
		    if (!bpushed)
			{
		    arm.MoveTo(arm.getCurrentPosition()-1000);
		    bpushed = true;
			}
		    
		}
	    else
		{
		    bpushed = false;
		}
            telemetry.addData("Arm Position", "%10d", armposition);

	    if (arm.isBusy())
		{
            telemetry.addData("current armpower", "%5.2f", armpower);
		}
	    else
		{
            telemetry.addData(">", "not busy");
		}
            telemetry.addData(">", "Press A to move forward");
            telemetry.addData(">", "Press B to move backward");
		    
            telemetry.update();
        }
        // Set the servo to the new position and pause;
        //            rotator_servo.setPosition(position);
        //            sleep(CYCLE_MS);
        //idle();


    }
}
