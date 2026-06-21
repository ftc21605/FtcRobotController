
package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Slide;

@TeleOp(name = "Test: Arm Position", group = "Test")
//@Disabled
public class ArmPosition extends LinearOpMode {

    Arm arm = new Arm(this);
    Slide slide = new Slide(this);
    private ElapsedTime runtime = new ElapsedTime();
    boolean p1Xpushed = false;

    @Override
    public void runOpMode() {

        arm.init();
	slide.init();
        telemetry.addData(">", "Press Start to run.");
        telemetry.update();
        waitForStart();


        // Scan servo till stop pressed.
        while (opModeIsActive()) {
	   if (gamepad1.x){
	       if (! p1Xpushed)
		   {
		       		       // arm.Brake();
		       		       // slide.Brake();
	       arm.Reset();
	       slide.Reset();
	       p1Xpushed = true;
		   }
	   }
            telemetry.addData("Arm Position", "%10d", arm.getCurrentPosition());
            telemetry.addData("Slide Position", "%10d", slide.getCurrentPosition());
            telemetry.addData(">","Push X to reset");
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
