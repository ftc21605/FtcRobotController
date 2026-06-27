package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.hardware.Grabber;
import org.firstinspires.ftc.teamcode.hardware.Rotator;
import org.firstinspires.ftc.teamcode.hardware.Slide;

@TeleOp(name = "Faire TeleOp", group = "AWallace")
//@Disabled
public class FaireTeleOP extends LinearOpMode {

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
    boolean p1Xpushed = false;
    boolean p1Ypushed = false;
    boolean p1bpushed = false;
    boolean p1apushed = false;
    boolean p2Xpushed = false;
    boolean override_arm_safety = false;
    boolean override_slide_safety = false;
    boolean auto_arm_slide = false;
    boolean auto_arm_slide_up = false;
    boolean auto_arm_slide_down = false;

    boolean armdown = false;
    boolean slidedown = false;
    boolean slidedownonly = false;
    boolean leftbumper = false;
    boolean rightbumper = false;
    boolean armup = false;
    boolean no_move_arm = false;

    boolean enable_robot = true;

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
            boolean slowbot = true;
            boolean debugdrive = false;
            double lateral = 0;
            double axial = 0;
            double yaw = 0;
            double armpower = 0;  // Note: pushing stick forward gives negative value
            int armposition = arm.getCurrentPosition();
            int slideposition = slide.getCurrentPosition();
            double powerslide = 0;//-gamepad1.right_stick_y;  // Note: pushing stick forward gives negative value
            if (gamepad2.a) {
                enable_robot = false;
            }
            if (gamepad2.b) {
                enable_robot = true;
            }

            if (enable_robot) {
                axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
                double left_stick_val = gamepad1.left_stick_x;
                if (Math.abs(left_stick_val) > 0.2) {
                    lateral = gamepad1.left_stick_x * 0.8;
                }
                yaw = gamepad1.right_stick_x * 0.6;
            } else {
                axial = -gamepad2.left_stick_y;
                double left_stick_val = gamepad2.left_stick_x;
                if (Math.abs(left_stick_val) > 0.2) {
                    lateral = gamepad2.left_stick_x * 0.8;
                }
                yaw = gamepad2.right_stick_x * 0.6;
            }

            if ((gamepad1.right_trigger > 0 || gamepad1.left_trigger > 0) && enable_robot) {
                if (gamepad1.right_trigger > 0) {
                    if (slideposition < 1000) {
                        powerslide = Math.min(gamepad1.right_trigger, 0.5);
                    }
                } else {
                    powerslide = -Math.min(gamepad1.left_trigger, 0.5);
                }
            }
            drive.driveRobot(axial * 0.7, lateral, yaw * 0.9);
            if (gamepad1.y && enable_robot) {
                grabber.grab();
            }
            if (gamepad1.x && enable_robot) {
                grabber.release();
            }
            if (gamepad1.a && enable_robot) {
                if (!apushed) {
                    //                    rotator.rotate_left();
                    rotator.setposition(0.45);
                    apushed = true;
                }
            } else {
                apushed = false;
            }
            if (gamepad1.b && enable_robot) {
                if (!bpushed) {
                    rotator.rotate_right();
                    bpushed = true;
                }
            } else {
                bpushed = false;
            }
            if (enable_robot) {
                if (gamepad1.left_bumper) {
                    if (!leftbumper) {
                        slide.Float();
                        slide.move(-0.7);
                        leftbumper = true;
                        slidedown = true;
                        auto_arm_slide = true;
                        auto_arm_slide_down = true;
                        auto_arm_slide_up = false;
                    }

                } else {
                    leftbumper = false;
                }


                if (gamepad1.right_bumper) {
                    if (!rightbumper) {
                        arm.Brake();
                        arm.MoveTo(arm.getArmDropPosition() - 400, 1.);
                        rotator.setposition(0.45); // rotate sample horizontal
                        rightbumper = true;
                        armup = true;
                        auto_arm_slide_down = false;
                        auto_arm_slide = true;
                        auto_arm_slide_up = true;
                    }

                } else {
                    rightbumper = false;
                }
            }


            if (armup && armposition > 500) {
                slide.MoveTo(1800, 1.);
                armup = false;
                telemetry.addData(">", "should move slide Press dpad_up to continue");

                telemetry.update();

            }


            if (slowbot) {
                drive.driveRobotSlow(axial, lateral, yaw);
            }

            if (Math.abs(armpower) > 0.05) {
                savearmpower = armpower;
            }

            if (armposition >= arm.getArmMaxPosition()) {
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
            if (slideposition <= 60 && !override_slide_safety) {
                powerslide = Math.max(powerslide, 0);
            }

            if (armposition > arm.getArmSlowPosition() && slideposition > 2000) {
                armpower = Math.min(armpower, 0.2);
            }
            if (armdown && armposition < 100) {
                arm.Stop();
                armdown = false;
                auto_arm_slide_down = false;
            }
            if (slidedown && slideposition < 60) {
                slide.Stop();
                slidedown = false;
                arm.Float();
                arm.move(-0.7);
                armdown = true;
            }
            if (auto_arm_slide && !slide.isBusy() && !arm.isBusy() && !auto_arm_slide_up && !auto_arm_slide_down) {
                auto_arm_slide = false;
            }
            if (!auto_arm_slide && !no_move_arm) {
                slide.move(powerslide);
                arm.move(armpower);
            }
            // move arm and slide with gamepad2 without safety to reset them
            if (!enable_robot) {
                slide.move(-Math.min(gamepad2.right_trigger, 0.4));
                arm.move(-Math.min(gamepad2.left_trigger, 0.4));
                if (gamepad2.x && !p2Xpushed) {
                    p2Xpushed = true;
                    arm.Reset();
                    slide.Reset();
                } else {
                    p2Xpushed = false;
                }

            }
            // if (gamepad2.right_trigger > 0)
            // 	{
            //
            // 	}
            // else
            // 	{
            // 	    slide.Stop();
            // 	}
            // if (gamepad2.left_trigger > 0)
            // 	{
            // 	    slide.move(gamepad2.right_trigger);
            // 	}
            // else
            // 	{
            // 	    slide.Stop();
            // 	}
            if (enable_robot) {
                telemetry.addData("Status", " Enabled");
            } else {
                telemetry.addData("Status", " Disabled (gamepad2 A was pressed), press B to re-enable");
            }
            telemetry.addData("Active", "Run Time: " + runtime);
            if (debugdrive) {
                telemetry.addData("axial:", "%5.2f", axial);
                telemetry.addData("lateral:", "%5.2f", lateral);
                telemetry.addData("yaw:", "%5.2f", yaw);
            }
            telemetry.addData("rotator pos:", "%5.2f", rotator.currpos());
            telemetry.addData("armpos:", "%10d", armposition);
            telemetry.addData("slidepos:", "%10d", slideposition);
            if (enable_robot) {
                telemetry.addData(">", "Press A to reset grabber to straight position");
                telemetry.addData(">", "Press B to reset grabber left");
                telemetry.addData(">", "Press X to release");
                telemetry.addData(">", "Press Y to grab");
                telemetry.addData(">", "Press left trigger to retract arm");
                telemetry.addData(">", "Press right trigger for drop position");
            } else {
                telemetry.addData(">", "Press right trigger to retract slide");
                telemetry.addData(">", "Press left to retract arm");
                telemetry.addData(">", "Press X to reset arm and slide encoder");
            }


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
