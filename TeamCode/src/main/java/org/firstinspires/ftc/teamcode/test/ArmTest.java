
package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Arm;

@TeleOp(name = "Test: Arm Test", group = "Test")
//@Disabled
public class ArmTest extends LinearOpMode {

    Arm arm = new Arm(this);
    double savepower = 0.;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        arm.init();
        telemetry.addData(">", "Press Start to run.");
        telemetry.update();
        waitForStart();


        // Scan servo till stop pressed.
        while (opModeIsActive()) {
            double power = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            if (Math.abs(power) > 0.05) {
                savepower = power;
            }
            long armposition = arm.getCurrentPosition();
            if (armposition >= 1900) {
                power = Math.min(power, 0);
            }
            if (armposition <= 60) {
                power = Math.max(power, 0);
            }

            arm.move(power);
            // Display the current value
            telemetry.addData("Arm Position", "%10d", armposition);
            telemetry.addData("last power", "%5.2f", savepower);
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
