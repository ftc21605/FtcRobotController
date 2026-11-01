package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.hardware.Grabber;
import org.firstinspires.ftc.teamcode.hardware.Rotator;
import org.firstinspires.ftc.teamcode.hardware.Slide;

@TeleOp(name = "TeleOp", group = "AWallace")
@Disabled
public class OurTeleOP extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();
    //    Grabber grabber = new Grabber(this);
    DriveTrain drive = new DriveTrain(this);
    Arm arm = new Arm(this);
    Rotator rotator = new Rotator(this);
    Slide slide = new Slide(this);
    Grabber grabber = new Grabber(this);
    boolean apushed = false;
    boolean bpushed = false;
    double savearmpower = 0.;
    double savepowerslide = 0.;
    double maxslidepos = 2980;
    double maxarmpos = 1900; // 1550 for dropping things
    double slowaxial = 0.2;
    double slowlateral = 0.15;
    double slowyaw = 0.15;
    boolean p1Xpushed = false;
    boolean p1Ypushed = false;
    boolean p1bpushed = false;
    boolean override_arm_safety = false;
    // boolean bumper_right_pushed_arm = false;
    // boolean bumper_right_pushed_slide = false;

    boolean armdown = false;
    boolean slidedown = false;
    boolean slidedownonly = false;
    boolean leftbumper = false;
    boolean rightbumper = false;
    boolean armup = false;


    @Override
    public void runOpMode() {
        drive.init();
        arm.init();
        grabber.init();
        rotator.init();
        //sleep(10);
        //	rotator.initpos();
        slide.init();
        telemetry.addData(">", "Press Start to run");
        telemetry.update();
        waitForStart();


        // Scan servo till stop pressed.
        while (opModeIsActive()) {
            boolean slowbot = false;
            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = 0;
            double left_stick_val = gamepad1.left_stick_x;
            if (Math.abs(left_stick_val) > 0.2) {
                lateral = gamepad1.left_stick_x * 0.8;
            }
            double yaw = gamepad1.right_stick_x * 0.6;
            double armpower = -gamepad2.left_stick_y;  // Note: pushing stick forward gives negative value
            int armposition = arm.getCurrentPosition();
            int slideposition = slide.getCurrentPosition();
            // if (Math.abs(gamepad1.left_stick_x) > 0.8) {
            //     if (Math.abs(gamepad1.left_stick_y) < 0.3) {
            //         lateral = gamepad1.left_stick_x*0.8;
            //     }
            // }
            drive.driveRobot(axial, lateral, yaw);
            if (gamepad1.b) {
                if (!p1bpushed) {
                    arm.Float();
                    p1bpushed = true;
                }
            } else {
                p1bpushed = false;
            }

            if (gamepad1.x) {
                if (!p1Xpushed) {
                    arm.Brake();
                    arm.Reset();
                    p1Xpushed = true;
                }
            } else {
                p1Xpushed = false;
            }
            if (gamepad1.y) {
                if (!p1Ypushed) {
                    slide.Reset();
                    p1Ypushed = true;
                }
            } else {
                p1Ypushed = false;
            }
            if (gamepad1.dpad_down) {
                override_arm_safety = true;
                armpower = -0.4;
            } else {
                override_arm_safety = false;
            }

            if (gamepad2.y) {
                grabber.grab();
            }
            if (gamepad2.x) {
                grabber.release();
            }
            if (gamepad2.a) {
                if (!apushed) {
                    //                    rotator.rotate_left();
                    rotator.setposition(0.45);
                    apushed = true;
                }
            } else {
                apushed = false;
            }
            if (gamepad2.b) {
                if (!bpushed) {
                    rotator.rotate_right();
                    bpushed = true;
                }
            } else {
                bpushed = false;
            }
            if (gamepad2.dpad_right) {
                lateral = drive.getDpadLateralPower();
                telemetry.addData("Status", "Dpad right pushed ");
                slowbot = true;
            }
            if (gamepad2.dpad_left) {
                lateral = -drive.getDpadLateralPower();
                slowbot = true;
            }
            if (gamepad2.dpad_up) {
                axial = drive.getDpadAxialPower();
                slowbot = true;
            }
            if (gamepad2.dpad_down) {
                axial = -drive.getDpadAxialPower();
                telemetry.addData("Status", "Dpad down pushed ");
                slowbot = true;
            }
            if (gamepad2.right_stick_x > 0.5) {
                yaw = -drive.getDpadYawPower();
                slowbot = true;
            }
            if (gamepad2.right_stick_x < -0.5) {
                yaw = drive.getDpadYawPower();
                slowbot = true;
            }

            if (gamepad2.left_bumper) {
                if (!leftbumper) {
                    slide.Float();
                    slide.move(-0.7);
                    leftbumper = true;
                    slidedown = true;
                }

            } else {
                leftbumper = false;
            }

            if (gamepad2.right_bumper) {
                if (!rightbumper) {
                    arm.Brake();
                    arm.MoveTo(arm.getArmDropPosition(), 1.);
                    rotator.setposition(0.45); // rotate sample horizontal
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

            }


            //     yaw = slowyaw;  // Note: pushing stick forward gives negative value
            //     slowbot = true;
            // }
            if (slowbot) {
                drive.driveRobotSlow(axial, lateral, yaw);
            }
            double powerslide = -gamepad2.right_stick_y;  // Note: pushing stick forward gives negative value
            if (Math.abs(armpower) > 0.05) {
                savearmpower = armpower;
            }

            if (armposition >= maxarmpos) {
                armpower = Math.min(armpower, 0);
            }
            if (armposition <= 60 && !override_arm_safety) {
                armpower = Math.max(armpower, 0);
            }

            if (Math.abs(powerslide) > 0.05) {
                savepowerslide = powerslide;
            }
            if (slideposition >= slide.maxSlidePosition(armposition)) {
                powerslide = Math.min(powerslide, 0);
            }
            if (slideposition <= 60) {
                powerslide = Math.max(powerslide, 0);
            }

            if (gamepad2.right_trigger > 0) {
                powerslide = 0.07;
                if (armposition > arm.getArmDropPosition() + 50) {
                    armpower = -0.1;
                }
                if (armposition < arm.getArmDropPosition() - 50) {
                    armpower = 0.1;
                }
            }
            if (gamepad2.left_trigger > 0) {
                if (armposition < arm.getArmDropPosition()) {
                    armpower = 0.4;
                }
            }
            if (armposition > arm.getArmSlowPosition()) {
                armpower = Math.min(armpower, 0.2);
            }
            if (armdown && armposition < 100) {
                arm.Stop();
                armdown = false;
            }
            if (slidedown && slideposition < 60) {
                slide.Stop();
                slidedown = false;
                arm.Float();
                arm.move(-0.7);
                armdown = true;
            }
            //            slide.move(powerslide);
            //arm.move(armpower);

            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.addData("rotator pos:", "%5.2f", rotator.currpos());
            telemetry.addData("armpos:", "%10d", armposition);
            telemetry.addData("slidepos:", "%10df", slideposition);
            telemetry.addData(">", "Press a to turn left.");
            telemetry.addData(">", "Press b to turn right.");
            telemetry.addData(">", "Press X to grab.");
            telemetry.addData(">", "Press y to release.");
            telemetry.addData(">", "Press left trigger to position arm");
            telemetry.addData(">", "Press right trigger to lock slide");
            telemetry.addData(">", "Use DPad for fine positioning of robot");
            telemetry.addData(">", "Use Bumpers to fine rotation of robot");
            telemetry.addData(">", "Press gamepad1 x to reset Arm encoder");
            telemetry.addData(">", "Press gamepad1 y to reset Slide encoder");


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
