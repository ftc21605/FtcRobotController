package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Slide;

@TeleOp(name = "Test: Arm Slide Auto Move Position", group = "Test")
//@Disabled
public class ArmSlideMoveToPositionTest extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();
    Arm arm = new Arm(this);
    Slide slide = new Slide(this);
    double savepower = 0.;
    boolean apushed = false;
    boolean bpushed = false;
    boolean xpushed = false;
    boolean ypushed = false;
    boolean armdown = false;
    boolean slidedown = false;
    boolean slidedownonly = false;
    boolean leftbumper = false;
    boolean rightbumper = false;
    boolean armup = false;

    @Override
    public void runOpMode() {

        arm.init();
        slide.init();
        telemetry.addData(">", "Press Start to run.");
        telemetry.update();
        waitForStart();


        // Scan servo till stop pressed.
        while (opModeIsActive()) {
            int armposition = arm.getCurrentPosition();
            int slideposition = slide.getCurrentPosition();
            if (gamepad1.left_bumper) {
                if (!leftbumper) {
                    slide.Float();
                    slide.move(-0.4);
                    leftbumper = true;
                    slidedown = true;
                }

            } else {
                leftbumper = false;
            }

            if (gamepad1.right_bumper) {
                if (!rightbumper) {
                    arm.Brake();
                    arm.MoveTo(arm.getArmDropPosition(), 1.);
                    rightbumper = true;
                    armup = true;
                }

            } else {
                rightbumper = false;
            }
            if (armup && armposition > 1100) {
                slide.MoveTo(slide.maxSlidePosition(arm.getArmDropPosition()), 1.);
                armup = false;
            telemetry.addData(">", "should move slide Press dpad_up to continue");

            telemetry.update();

		// while(!gamepad1.dpad_up)
		//     {
		// 	sleep(1);
		//     }
            }

            if (gamepad1.a) {
                if (!apushed) {
                    arm.Brake();
                    arm.MoveTo(arm.getArmDropPosition(), 1.);
                    apushed = true;
                }

            } else {
                apushed = false;
            }
            if (gamepad1.b) {
                if (!bpushed) {
                    slide.MoveTo(slide.maxSlidePosition(arm.getArmDropPosition()), 1.);
                    bpushed = true;
                }

            } else {
                bpushed = false;
            }
            if (gamepad1.x) {
                if (!xpushed) {
                    arm.Float();
                    arm.move(-0.4);
                    xpushed = true;
                    armdown = true;
                }

            } else {
                xpushed = false;
            }
            if (gamepad1.y) {
                if (!ypushed) {
                    slide.Float();
                    slide.move(-0.4);
                    ypushed = true;
                    slidedownonly = true;
                }

            } else {
                ypushed = false;
            }
            if (armdown && armposition < 100) {
                arm.Stop();
                armdown = false;
            }
            if (slidedown && slideposition < 60) {
                slide.Stop();
                slidedown = false;
                arm.Float();
                arm.move(-0.4);
                armdown = true;
            }
            if (slidedownonly && slideposition < 60) {
                slide.Stop();
                slidedownonly = false;
            }
            telemetry.addData("Arm Position", "%10d", armposition);
            telemetry.addData("Slide Position", "%10d", slideposition);

            if (arm.isBusy()) {
                telemetry.addData("current armpower", "%5.2f", arm.getPower());
            } else {
                telemetry.addData(">", "arm not busy");
            }
            if (slide.isBusy()) {
                telemetry.addData("current slide power", "%5.2f", slide.getPower());
            } else {
                telemetry.addData(">", "slide not busy");
            }
            telemetry.addData(">", "Press A to move arm");
            telemetry.addData(">", "Press B to move slide");

            telemetry.update();
        }
        // Set the servo to the new position and pause;
        //            rotator_servo.setPosition(position);
        //            sleep(CYCLE_MS);
        //idle();


    }
}
