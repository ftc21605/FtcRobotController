/* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This OpMode scans a single servo back and forward until Stop is pressed.
 * The code is structured as a LinearOpMode
 * INCREMENT sets how much to increase/decrease the servo position each cycle
 * CYCLE_MS sets the update period.
 *
 * This code assumes a Servo configured with the name "left_hand" as is found on a Robot.
 *
 * NOTE: When any servo position is set, ALL attached servos are activated, so ensure that any other
 * connected servos are able to move freely before running this test.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
@TeleOp(name = "Test: Pixel Release", group = "ZTest")
//@Disabled
public class PixelRelease extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor Intake = null;

    @Override
    public void runOpMode() {


        Intake = hardwareMap.get(DcMotor.class, "intake");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        Intake.setDirection(DcMotor.Direction.FORWARD);
        Intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        waitForStart();


        // Scan servo till stop pressed.
        while (opModeIsActive()) {
            double Power = 0.2;
            int tics = 8;
            telemetry.addData(">", "Rotations Tics %7d", Intake.getCurrentPosition());
            telemetry.update();
            if (gamepad1.x) {
                Intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            }
            if (gamepad1.a && Intake.getCurrentPosition() < tics) {
                Intake.setPower(Power);
                while (Intake.getCurrentPosition() < tics) {
                sleep(1);}
                telemetry.addData(">", "Rotations Tics in loop %7d", Intake.getCurrentPosition());
                telemetry.update();
                Intake.setPower(0);
            }
            //Intake.setPower(0);
            //Intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            if (gamepad1.b) {
                Intake.setPower(-Power);
                while (Intake.getCurrentPosition() > -tics) {
                    sleep(1);
                }
                Intake.setPower(0);
             }

        }

        // Signal done;
        // telemetry.addData(">", "Done");
        // telemetry.update();
    }
    }

