package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Grabber;
import org.firstinspires.ftc.teamcode.hardware.Rotator;

@TeleOp(name = "Test: Grabber Test", group = "Test")
//@Disabled
public class GrabberTest extends LinearOpMode {

    Rotator rotator = new Rotator(this);
    Grabber grabber = new Grabber(this);
    boolean apushed = false;
    boolean bpushed = false;
    boolean xpushed = false;
    boolean ypushed = false;
    boolean lbpushed = false;
    boolean rbpushed = false;
    @Override
    public void runOpMode() {
        grabber.init();
        rotator.init();
	sleep(1000);
	rotator.initpos();
        telemetry.addData(">", "Press Start to test Grabber");
        telemetry.update();
        waitForStart();


        // Scan servo till stop pressed.
        while (opModeIsActive()) {

            if (gamepad1.y) {
		if (!ypushed)
		    {
			                grabber.grab();
					rotator.initpos();
					ypushed = true;
		    }
	    }
	    else
		{
		    ypushed = false;
		}
            if (gamepad1.x) {
		if (!xpushed)
		    {
			                grabber.release();
		xpushed = true;
		    }
	    }
	    else
		{
		    xpushed = false;
		}
            if (gamepad1.a) {
                if (!apushed) {
                    rotator.rotate_left();
                    apushed = true;
                }
            } else {
                apushed = false;
            }
            if (gamepad1.b) {
                if (!bpushed) {
                    rotator.rotate_right();
                    bpushed = true;
                }
            } else {
                bpushed = false;
            }
	    if (gamepad1.right_bumper)
		{
		    if (!rbpushed)
			{
			rotator.rotate_all_right();
			rbpushed = true;
			}
		}
	    else
		{
		    rbpushed = false;
		}
	    if (gamepad1.left_bumper)
		{
		    if (!lbpushed)
			{
			rotator.rotate_all_left();
			lbpushed = true;
			}
		}
	    else
		{
		    lbpushed = false;
		}
		    
            telemetry.addData("currpos:", "%5.2f", rotator.currpos());
            telemetry.addData(">", "Press a to turn left.");
            telemetry.addData(">", "Press b to turn right.");
            telemetry.addData(">", "Press X to grab.");
            telemetry.addData(">", "Press y to release.");
            telemetry.addData(">", "Press left bumper to move all left.");
            telemetry.addData(">", "Press right bumper to move all right.");
            telemetry.update();

            // Set the servo to the new position and pause;
            //            rotator_servo.setPosition(position);
            //            sleep(CYCLE_MS);
            //idle();
        }

        // Signal done;
        telemetry.addData(">", "Done");
        telemetry.update();
    }
}
