package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.DriveTrain;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "DpadDrive", group = "Test")
//@Disabled
public class DpadDrive extends LinearOpMode {

    DriveTrain drive = new DriveTrain(this);
    // Declare OpMode members for each of the 4 motors.
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        // Wait for the game to start (driver presses PLAY)
        drive.init();
        telemetry.addData(">", "Press Start to run");
        telemetry.update();
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            double axial = 0;
            double lateral = 0;
            double yaw = 0;
            if (gamepad1.dpad_right) {
                lateral = drive.getDpadLateralPower();  // Note: pushing stick forward gives negative value
                telemetry.addData("Status", "Dpad right pushed ");
            }
            if (gamepad1.dpad_left) {
                lateral = -drive.getDpadLateralPower();
                telemetry.addData("Status", "Dpad left pushed ");
            }
            if (gamepad1.dpad_up) {
                axial = drive.getDpadAxialPower();
                telemetry.addData("Status", "Dpad up pushed ");
            }
            if (gamepad1.dpad_down) {
                axial = -drive.getDpadAxialPower();
                telemetry.addData("Status", "Dpad down pushed ");
            }
            if (gamepad1.right_stick_x < -0.5) {
                yaw = drive.getDpadYawPower();  // Note: pushing stick forward gives negative value
                telemetry.addData("Status", "left bumper pushed ");
            }
            if (gamepad1.right_stick_x > 0.5) {
                yaw = -drive.getDpadYawPower();
                telemetry.addData("Status", "right bumper pushed ");
            }
            telemetry.addData("axial:", "%5.2f", axial);
            telemetry.addData("lateral:", "%5.2f", lateral);
            telemetry.addData("yaw:", "%5.2f", yaw);

            //double lateral = gamepad1.left_stick_x;
            drive.driveRobotSlow(axial, lateral, yaw);
            telemetry.update();
        }
    }
}


