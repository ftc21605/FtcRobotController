
package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Slide;

@TeleOp(name = "Test: Arm Turn Test", group = "Test")
//@Disabled
public class ArmTurnTest extends LinearOpMode {

    Slide slide = new Slide(this);
    Arm arm = new Arm(this);


    @Override
    public void runOpMode() {

        slide.init();
        arm.init();
        // Connect to servo (Assume Robot Left Hand)
        // Wait for the start button
        telemetry.addData(">", "Press Start.");
        telemetry.update();
        waitForStart();


        // Scan servo till stop pressed.
        while (opModeIsActive()) {

            long armposition = arm.getCurrentPosition();
            long slideposition = slide.getCurrentPosition();
            double power = gamepad1.left_stick_y;
            arm.move(power);
            double powerext = gamepad1.left_stick_x;
            slide.move(powerext);
            telemetry.addData("Power", "%5.2f", power);
            telemetry.addData("armpos:", "%10d", armposition);
            telemetry.addData("slidepos:", "%10df", slideposition);
            telemetry.update();

        }

        // Signal done;
        telemetry.addData(">", "Done");
        telemetry.update();
    }
}
