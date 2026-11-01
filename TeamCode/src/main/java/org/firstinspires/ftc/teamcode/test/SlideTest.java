
package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Slide;

@TeleOp(name = "Test: Slide Test", group = "Test")
//@Disabled
public class SlideTest extends LinearOpMode {

    Arm arm = new Arm(this);
    Slide slide = new Slide(this);
	    double savepowerarm = 0.;
	    double savepowerslide = 0.;
   private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

	arm.init();
	slide.init();
        telemetry.addData(">", "Press Start to run." );
        telemetry.update();
        waitForStart();


        // Scan servo till stop pressed.
        while(opModeIsActive()){
double powerslide =  -gamepad1.right_stick_y;  // Note: pushing stick forward gives negative value
double powerarm =  -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
if (Math.abs(powerarm) > 0.05)
    {
	savepowerarm = powerarm;
    }
if (Math.abs(powerslide) > 0.05)
    {
	savepowerslide = powerslide;
    }
long armposition = arm.getCurrentPosition();
long slideposition = slide.getCurrentPosition();
if (armposition >= 1900)
{
    powerarm = Math.min(powerarm,0);
}
if (armposition <= 60)
{
    powerarm = Math.max(powerarm,0);
}
arm.move(powerarm);
if (slideposition >= 3000)
    {
	powerslide = Math.min(powerslide,0);
    }
if (slideposition <= 60)
    {
	powerslide = Math.max(powerslide,0);
    }
	
	slide.move(powerslide);

// Display the current value
telemetry.addData("Arm Position", "%10d", armposition);
telemetry.addData("last powerarm", "%5.2f", savepowerarm);
telemetry.addData("Slide Position", "%10d", slideposition);
telemetry.addData("last powerslide", "%5.2f", savepowerslide);
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
