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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
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
@TeleOp(name = "Test: Hook Test", group = "ZTest")
//@Disabled
public class HookTest extends LinearOpMode {

    static final double MOVE_UP     =  0.8;     // move up (cont servo posistion = 1)
    static final double MOVE_DOWN     =  0.0;     // move down (cont servo position = 0)
    static final double MOVE_STOP     =  0.5;     // stop moving (cont servo position 0.5)
    // Define class members
    Servo   CRservo;
    TouchSensor ElevatorLimit;  // Touch sensor Object

boolean moving_up = false;
boolean moving_down = false;
    @Override
    public void runOpMode() {

        // Connect to servo (Assume Robot Left Hand)
        // Change the text in quotes to match any servo name on your robot.
        CRservo = hardwareMap.get(Servo.class, "hook");
        ElevatorLimit = hardwareMap.get(TouchSensor.class, "elevatorlimit");
        ElapsedTime timer = new ElapsedTime();
        // Wait for the start button
        telemetry.addData(">", "Press Start to scan Servo." );
        telemetry.update();
        waitForStart();


        // Scan servo till stop pressed.
        while(opModeIsActive()){

            // slew the servo, according to the rampUp (direction) variable.


            telemetry.addData(">", "Press X for moving up");
            telemetry.addData(">", "Press Y for moving down");
            telemetry.addData(">", "Press A for stop");
            if (gamepad1.x) {
                CRservo.setPosition(MOVE_UP);
                moving_up = true;
moving_down = false;
            }

            if (gamepad1.y) {
                CRservo.setPosition(MOVE_DOWN);
                timer.reset();
                moving_up = false;
                moving_down = true;
            }
            if (gamepad1.a) {
                CRservo.setPosition(MOVE_STOP);
                moving_up = false;
                moving_down = false;
            }
            if (ElevatorLimit.isPressed() && moving_up) {
                telemetry.addData("Touch Sensor", "Is Pressed");
                moving_up = false;
                CRservo.setPosition(MOVE_STOP);
            }

if (moving_down && timer.seconds() > 1)
{
    CRservo.setPosition(MOVE_STOP);
    moving_up = false;
    moving_down = false;
}
            // Display the current value
            telemetry.addData("Servo Power", "%5.2f", CRservo.getPosition());
            telemetry.addData(">", "Press Stop to end test.");
            telemetry.update();


            // Display the current value

        }


    }
}
