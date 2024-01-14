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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 * <p>
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 * <p>
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 * <p>
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 * <p>
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 * <p>
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 * <p>
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "AWallace")
//@Disabled
public class TeleOp extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    Servo PlaneServo;
    static final double PLANE_LAUNCH = 1.;
    static final double PLANE_LOAD = 0.;
    boolean plane_launched = false;
    boolean x_pushed = false;

    Servo ElevatorServo;
    TouchSensor ElevatorLimit; // magnet limit switch
    static final double MOVE_UP = 1.0;     // move up (cont servo posistion = 1)
    static final double MOVE_DOWN = 0.0;     // move down (cont servo position = 0)
    static final double MOVE_STOP = 0.5;     // stop moving (cont servo position 0.5)
    boolean moving_up = false;
    boolean moving_down = false;
    boolean y_pushed = false;
    boolean b_pushed = false;

    // Intake
    private DcMotor Intake = null;

    private DcMotor Hanger = null;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "frontleft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "frontright");
        leftBackDrive = hardwareMap.get(DcMotor.class, "backleft");
        rightBackDrive = hardwareMap.get(DcMotor.class, "backright");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        PlaneServo = hardwareMap.get(Servo.class, "plane");

        ElevatorServo = hardwareMap.get(Servo.class, "hook");
        ElevatorLimit = hardwareMap.get(TouchSensor.class, "elevatorlimit");
        ElapsedTime ElevatorTimer = new ElapsedTime();

        Intake = hardwareMap.get(DcMotor.class, "intake");
        Intake.setDirection(DcMotor.Direction.FORWARD);
        Hanger = hardwareMap.get(DcMotor.class, "hanger");
        Hanger.setDirection(DcMotor.Direction.FORWARD);

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
            telemetry.addData(">", "Press A again for emergency hook stop");
            telemetry.addData(">", "Press DPad Up again for hanging");
            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -gamepad1.left_stick_y / 2.;  // Note: pushing stick forward gives negative value
            double lateral = 0.;
            if (Math.abs(gamepad1.left_stick_x) > 0.8) {
                lateral = gamepad1.left_stick_x / 2.;
            }
            double yaw = gamepad1.right_stick_x / 2.;

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


            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            double intake_power = 0;
            if (gamepad1.right_trigger > 0 && gamepad1.left_trigger == 0) {
                intake_power = gamepad1.right_trigger;
            }
            if (gamepad1.left_trigger > 0 && gamepad1.right_trigger == 0) {
                intake_power = -gamepad1.left_trigger;
            }
            Intake.setPower(intake_power);

            if (gamepad1.x && !x_pushed) {
                x_pushed = true;
                if (!plane_launched) {
                    PlaneServo.setPosition(PLANE_LAUNCH);
                    plane_launched = true;
                } else {
                    PlaneServo.setPosition(PLANE_LOAD);
                    plane_launched = false;
                }
            }
            if (!gamepad1.x) {
                x_pushed = false;
            }

//            telemetry.addData(">", "Y Button" + String.valueOf(gamepad1.y));
            //           telemetry.addData(">", "y_pushed " + String.valueOf(y_pushed));
            //         telemetry.addData(">", "moving_up " + String.valueOf(moving_up));
            //       telemetry.addData(">", "moving_down "+ String.valueOf(moving_down));
            //     telemetry.addData(">", "elevator time " + ElevatorTimer.toString());
            if (gamepad1.y && !y_pushed) {

                y_pushed = true;

                    ElevatorServo.setPosition(MOVE_UP);
                    moving_up = true;
                    moving_down = false;
            }
            if (!gamepad1.y) {
                y_pushed = false;
            }

            if (gamepad1.b && !b_pushed) {

                b_pushed = true;

                    moving_down = true;
                moving_up = false;
                    ElevatorServo.setPosition(MOVE_DOWN);
                    ElevatorTimer.reset();



            }
            if (!gamepad1.b) {
                b_pushed = false;
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
                double hangerpower = 0.5;
                Hanger.setPower(hangerpower);
            } else {
                Hanger.setPower(0);
            }
            //grabber.setPosition(grabber_position);
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}


