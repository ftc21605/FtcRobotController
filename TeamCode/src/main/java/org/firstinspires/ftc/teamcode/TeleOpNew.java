/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.OurLinearOpBase;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "New TeleOp", group = "AWallace")
//@Disabled
public class TeleOpNew extends OurLinearOpBase {

     boolean pad1_x_pushed = false;
     boolean pad1_a_pushed = false;

    boolean pad1_y_pushed = false;
    boolean pad1_b_pushed = false;

    boolean pad2_leftbumper_pressed = false;
    boolean pad2_rightbumper_pressed = false;
    boolean pad2_dpad_pressed = false;

    @Override
    public void runOpMode() {
	setup_drive_motors();
	setup_planeservo();
	setup_elevator();
	setup_intake();
	setup_hanger();
	setup_pixel_lift();
	setup_pixel_bucket();
	setup_pixel_transport();
 	setup_bucketback();
	setup_bucketfront();
        bucketback_release();

	// Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData(">", "left trigger intake in");
            telemetry.addData(">", "right trigger intake out");
            telemetry.addData(">", "left bumper + right stick: hang");
            telemetry.addData(">", "Press X for plane launch");
            telemetry.addData(">", "Press X again for load plane");
            telemetry.addData(">", "Press Y for hook deploy");
            telemetry.addData(">", "Press B for hook retract");
            telemetry.addData(">", "Press A for emergency hook stop");
            telemetry.addData(">", "Press DPad Up for hanging");
            telemetry.addData(">", "Press DPad Left for moving sideways to the left");
            telemetry.addData(">", "Press DPad Right for moving sideways to the right");
            telemetry.addData(">", "Press Right Bumper for Intake continuous in");
            telemetry.addData(">", "Press Left Bumper for Intake continuous out");

            telemetry.addData(">", "Pad2: Press A tilt bucket forward");
            telemetry.addData(">", "Pad2: Press B tile bucket backward");
            telemetry.addData(">", "Pad2: Press left bumper toggle back lock");
            telemetry.addData(">", "Pad2: Press right bumper toggle front lock");

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            if (Math.abs(axial) < 0.15) {
                axial = 0.;
            }
            axial = axial * 0.7;  // Note: pushing stick forward gives negative value
            double lateral = 0.;
            if (Math.abs(gamepad1.left_stick_x) > 0.8) {
                if (Math.abs(gamepad1.left_stick_y) < 0.3) {
                    lateral = gamepad1.left_stick_x;
                }
            }
            double yaw = 0;
	    if (Math.abs(gamepad1.right_stick_x) > 0.15)
	    {
		yaw = gamepad1.right_stick_x * 0.6;
	    }
            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;
            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            if (gamepad1.dpad_right) {
                leftFrontPower = 0.3;
                rightFrontPower = -0.3;
                leftBackPower = -0.3;
                rightBackPower = 0.3;
            }
            if (gamepad1.dpad_left) {
                leftFrontPower = -0.3;
                rightFrontPower = 0.3;
                leftBackPower = 0.3;
                rightBackPower = -0.3;
            }
            if (gamepad1.dpad_down) {
                PixelLift.setPower(-0.2);
            } else {
                PixelLift.setPower(0);
            }

            // Send calculated power to wheels
            //    leftFrontDrive.setPower(leftFrontPower);
            //    rightFrontDrive.setPower(rightFrontPower);
            //    leftBackDrive.setPower(leftBackPower);
            //    rightBackDrive.setPower(rightBackPower);

            double intake_power = 0;
            if (gamepad1.right_trigger > 0 && gamepad1.left_trigger == 0) {
                intake_power = gamepad1.right_trigger;
            }
            if (gamepad1.left_trigger > 0 && gamepad1.right_trigger == 0) {
                intake_power = -gamepad1.left_trigger;
            }
            Intake.setPower(intake_power);

            if (gamepad1.x && !pad1_x_pushed) {
                pad1_x_pushed = true;
                launch_plane();
            }
            if (!gamepad1.x) {
                pad1_x_pushed = false;
            }

//            telemetry.addData(">", "Y Button" + String.valueOf(gamepad1.y));
            //           telemetry.addData(">", "pad1_y_pushed " + String.valueOf(pad1_y_pushed));
            //         telemetry.addData(">", "moving_up " + String.valueOf(moving_up));
            //       telemetry.addData(">", "moving_down "+ String.valueOf(moving_down));
            //     telemetry.addData(">", "elevator time " + ElevatorTimer.toString());
            if (gamepad1.y && !pad1_y_pushed) {

                pad1_y_pushed = true;

                ElevatorServo.setPosition(MOVE_UP);
                moving_up = true;
                moving_down = false;
            }
            if (!gamepad1.y) {
                pad1_y_pushed = false;
            }

            if (gamepad1.b && !pad1_b_pushed) {

                pad1_b_pushed = true;

                moving_down = true;
                moving_up = false;
                ElevatorServo.setPosition(MOVE_DOWN);
                ElevatorTimer.reset();


            }
            if (!gamepad1.b) {
                pad1_b_pushed = false;
            }

            if (ElevatorLimit.isPressed() && !moving_down) {
                telemetry.addData("Touch Sensor", "Is Pressed");
                ElevatorServo.setPosition(MOVE_STOP);
            }
            if (moving_down && ElevatorTimer.milliseconds() > 1500) {
                ElevatorServo.setPosition(MOVE_STOP);
            }
            if (gamepad1.a) {
                ElevatorServo.setPosition(MOVE_STOP);
                moving_up = false;
                moving_down = false;
            }
            if (gamepad1.dpad_up) {
                // set hanger power
                hang_the_bot();
            } else {
                Hanger.setPower(0);
            }

            if (gamepad1.right_bumper) {
                PixelTransport.setPower(-0.5);
            }
            if (gamepad1.left_bumper) {
                PixelTransport.setPower(0.5);
            }

            if (!gamepad1.left_bumper && !gamepad1.right_bumper) {
                PixelTransport.setPower(0);
            }
            if (gamepad2.a) {
                bucket_tilt_forward();
            }


            // HERE IS GAMEPAD 2
            if (gamepad2.right_bumper) {
                if (!pad2_rightbumper_pressed) {
                    pad2_rightbumper_pressed = true;
                    bucketfront_toggle();
                }
            } else {
                pad2_rightbumper_pressed = false;
            }

            if (gamepad2.left_bumper) {
                if (!pad2_leftbumper_pressed) {
                    pad2_leftbumper_pressed = true;
                    bucketback_toggle();
                }
            } else {
                pad2_leftbumper_pressed = false;
            }


            if (gamepad2.b) {
                bucket_tilt_backward();
            }
            if (gamepad2.x) {
                bucketfront_lock();
                bucketback_release();
            }
            if (gamepad2.y) {
                bucketback_release();
            }
            //grabber.setPosition(grabber_position);
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
            telemetry.addData(">", "Pad2: left bumper toggles bucket back lock servo");
            telemetry.addData(">", "Pad2: right bumper toggles bucket front lock servo");
            if (gamepad2.right_bumper) {
                if (!pad2_rightbumper_pressed) {
                    pad2_rightbumper_pressed = true;
                    bucketfront_toggle();
                }
            } else {
                pad2_rightbumper_pressed = false;
            }

            if (gamepad2.left_bumper) {
                if (!pad2_leftbumper_pressed) {
                    pad2_leftbumper_pressed = true;
                    bucketback_toggle();
                }
            } else {
                pad2_leftbumper_pressed = false;
            }
            double Power;
            if (gamepad2.right_trigger > 0 && gamepad2.left_trigger == 0) {
                Power = gamepad2.right_trigger;
            } else if (gamepad2.left_trigger > 0 && gamepad2.right_trigger == 0) {
                Power = -gamepad2.left_trigger;
            } else {
                Power = 0;
            }

            PixelLift.setPower(Power);

            telemetry.addData(">", "Press Stop to end test.");
            telemetry.update();

            if (gamepad2.a) {
                bucket_tilt_forward();
            }
            if (gamepad2.b) {
                bucket_tilt_backward();
            }
            pad2_dpad_pressed = false;
            if (gamepad2.dpad_right) {
                pad2_dpad_pressed = true;
                leftFrontPower = 0.3;
                rightFrontPower = -0.3;
                leftBackPower = -0.3;
                rightBackPower = 0.3;
            }
            if (gamepad2.dpad_left) {
                pad2_dpad_pressed = true;
                leftFrontPower = -0.3;
                rightFrontPower = 0.3;
                leftBackPower = 0.3;
                rightBackPower = -0.3;
            }
            if (gamepad2.dpad_down) {
                pad2_dpad_pressed = true;
                leftFrontPower = -0.3;
                rightFrontPower = -0.3;
                leftBackPower = -0.3;
                rightBackPower = -0.3;

            }
            if (gamepad2.dpad_up) {
                pad2_dpad_pressed = true;
                leftFrontPower = 0.3;
                rightFrontPower = 0.3;
                leftBackPower = 0.3;
                rightBackPower = 0.3;

            }
            if (Math.abs(gamepad2.right_stick_x) > 0.1 ) {
                yaw = gamepad2.right_stick_x * 0.3;

                // Combine the joystick requests for each axis-motion to determine each wheel's power.
                // Set up a variable for each drive wheel to save the power level for telemetry.
                leftFrontPower = +yaw;
                rightFrontPower = -yaw;
                leftBackPower = +yaw;
                rightBackPower = -yaw;
            }

                if (!pad2_dpad_pressed) {
                    //    drivemotors_off();
                }
                leftFrontDrive.setPower(leftFrontPower);
                rightFrontDrive.setPower(rightFrontPower);
                leftBackDrive.setPower(leftBackPower);
                rightBackDrive.setPower(rightBackPower);


        }
    }
    public void pixel_release() {
        double Power = 0.2;
        int tics = 8;
        Intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Intake.setPower(Power);
        while (Intake.getCurrentPosition() < tics) {
            sleep(1);
        }
        Intake.setPower(0);


    }
    public void pixel_lock() {
        double Power = -0.2;
        int tics = 8;
        Intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Intake.setPower(Power);
        while (Intake.getCurrentPosition() < tics) {
            sleep(1);
        }
        Intake.setPower(0);


    }

}


