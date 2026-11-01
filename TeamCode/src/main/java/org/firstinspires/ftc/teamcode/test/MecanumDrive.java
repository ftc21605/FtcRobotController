package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.DriveTrain;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "MecanumDrive", group = "Test")
//@Disabled
public class MecanumDrive extends LinearOpMode {

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
            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x * 0.6;
	    telemetry.addData("axial:", "%5.2f", axial);
	    telemetry.addData("lateral:", "%5.2f", lateral);
	    telemetry.addData("yaw:", "%5.2f", yaw);
            drive.driveRobot(axial, lateral, yaw);

	    if (gamepad1.a)
		{
		    telemetry.addData("left front", "%10d", drive.getLeftFrontDrive().getCurrentPosition());
		    drive.getLeftFrontDrive().setPower(0.5);
		}
	    if (gamepad1.b)
		{
		    drive.getLeftBackDrive().setPower(0.5);
		    telemetry.addData("left back", "%10d", drive.getLeftBackDrive().getCurrentPosition());
		}
	    if (gamepad1.x)
		{
		    telemetry.addData("right front", "%10d", drive.getRightFrontDrive().getCurrentPosition());
		    drive.getRightFrontDrive().setPower(0.5);
		}
	    if (gamepad1.y)
		{
		    telemetry.addData("right back", "%10d", drive.getRightBackDrive().getCurrentPosition());
		    drive.getRightBackDrive().setPower(0.5);
		}
	    if (gamepad1.y)
		{
		    telemetry.addData("right back", "%10d", drive.getRightBackDrive().getCurrentPosition());
		    drive.getRightBackDrive().setPower(0.5);
		}
	    if (gamepad1.left_bumper)
		{
		    drive.left_turn_angle(90.);
		}
	    if (gamepad1.right_bumper)
		{
		    drive.right_turn_angle(90.);
		}
	    telemetry.addData(">", "Press left bumper for 90deg left turn");
	    telemetry.addData(">", "Press right bumper for 90deg right turn");
	    telemetry.addData("left ticks", "%10d", drive.getCurrentLeftFrontPosition());
	    telemetry.update();
        }
    }
}


