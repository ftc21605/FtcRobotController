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
import com.qualcomm.robotcore.hardware.Servo;

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
@TeleOp(name = "Test: Pixel Lift and Drop Test Button", group = "ZTest")
//@Disabled
public class PixelLiftAndDropTestButton extends LinearOpMode {

    static final double MAX_POS = 0.25;     // Maximum rotational position
    static final double MIN_POS = 0.5;     // Minimum rotational position

    // Define class members
    Servo CRservo;

    private DcMotor PixelLift = null;

    @Override
    public void runOpMode() {

        // Connect to servo (Assume Robot Left Hand)
        // Change the text in quotes to match any servo name on your robot.
        CRservo = hardwareMap.get(Servo.class, "pixelbucket");
        PixelLift  = hardwareMap.get(DcMotor.class, "pixellift");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        PixelLift.setDirection(DcMotor.Direction.REVERSE);
        PixelLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        PixelLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // Wait for the start button
        telemetry.addData(">", "Press X for max pos.");
        telemetry.addData(">", "Press Y for min pos.");
        telemetry.update();
        waitForStart();


        // Scan servo till stop pressed.
        while (opModeIsActive()) {

            // slew the servo, according to the rampUp (direction) variable.
            if (gamepad1.x) {
                CRservo.setPosition(MAX_POS);
            }
            if (gamepad1.y) {
                //PixelLift.setPower(0);
                CRservo.setPosition(MIN_POS);
            }

            double Power;
            if (gamepad1.right_trigger > 0 && gamepad1.left_trigger == 0)
            {
                Power = gamepad1.right_trigger;
            }
            else if (gamepad1.left_trigger > 0 && gamepad1.right_trigger == 0)
            {
                Power = -gamepad1.left_trigger;
            }
            else {
                Power = 0;
            }
            if (gamepad1.a) {
                PixelLift.setTargetPosition(1100);
                PixelLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                PixelLift.setPower(0.5);
                while(PixelLift.isBusy()){
                    sleep(10);
                }
                sleep(100);
                CRservo.setPosition(MAX_POS);
                sleep(3000);
                CRservo.setPosition(MIN_POS);
                sleep(100);
                PixelLift.setTargetPosition(100);
                PixelLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                PixelLift.setPower(0.2);
                while(PixelLift.isBusy()){
                    sleep(10);
                }
                PixelLift.setPower(0);


                //    while (PixelLift.getCurrentPosition() < 1100) {
               //     telemetry.addData("Pixel Lift", "position (%d)", PixelLift.getCurrentPosition());
//
             //       sleep(1);
             //   }
             //   PixelLift.setPower(0);
            }
            if (gamepad1.b) {
                PixelLift.setTargetPosition(300);
                PixelLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                PixelLift.setPower(0.2);
            }

            //PixelLift.setPower(Power);

            telemetry.addData(">", "Press X for max pos.");
            telemetry.addData(">", "Press Y for min pos.");
            telemetry.addData(">", "Press right trigger for lift up");
            telemetry.addData(">", "Press left trigger for lift down");
            telemetry.addData("Pixel Lift", "position (%d)", PixelLift.getCurrentPosition());

            telemetry.update();

             // Set the servo to the new position and pause;
         }

        // Signal done;
        telemetry.addData(">", "Done");
        telemetry.update();
    }
}

