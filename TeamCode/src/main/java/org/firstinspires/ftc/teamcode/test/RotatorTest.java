package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Rotator;

@TeleOp(name = "Test: Rotator Test", group = "Test")
//@Disabled
public class RotatorTest extends LinearOpMode {

    Rotator rotator = new Rotator(this);
    boolean apushed = false;
    boolean bpushed = false;

    @Override
    public void runOpMode() {
        rotator.init();
        telemetry.addData(">", "Press Start to test Rotator Servo.");
        telemetry.update();
        waitForStart();

        int acalls = 0;
        int bcalls = 0;
        // Scan servo till stop pressed.
        while (opModeIsActive()) {

            if (gamepad1.a) {
                if (!apushed) {
                    acalls++;
                    rotator.rotate_left();
                    apushed = true;
                }
            } else {
                apushed = false;
            }
            if (gamepad1.b) {
                if (!bpushed) {
                    bcalls++;
                    rotator.rotate_right();
                    bpushed = true;
                }
            } else {
                bpushed = false;
            }
            if (gamepad1.x) {
                acalls = 0;
                rotator.rotate_all_left();
            }
            if (gamepad1.y) {
                bcalls = 0;
                rotator.rotate_all_right();
            }
            telemetry.addData("calls A:", "%10d", acalls);
            telemetry.addData("calls B:", "%10d", bcalls);
            telemetry.addData("currpos:", "%5.2f", rotator.currpos());
            telemetry.addData(">", "Press a to turn left.");
            telemetry.addData(">", "Press b to turn right.");
            telemetry.addData(">", "Press x to turn all left.");
            telemetry.addData(">", "Press y to turn all right.");
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
