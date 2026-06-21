package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


public class DriveTrain {

    /* Declare OpMode members. */
    final private LinearOpMode myOpMode;   // gain access to methods in the calling OpMode.

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    double TURN_SPEED = -0.2;
    static final double DPAD_POWER_LATERAL = 0.3;
    static final double DPAD_POWER_AXIAL = 0.2;
    static final double DPAD_POWER_YAW = 0.2;
    // Define a constructor that allows the OpMode to pass a reference to itself.
    public DriveTrain(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void init() {
        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        leftFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "frontleft"); // port 0
        rightFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "frontright"); // port 1
        leftBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "backleft"); // port 2
        rightBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "backright"); // port 3

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        myOpMode.telemetry.addData(">", "Drive Train Initialized");
    }

    /**
     * Calculates the left/right motor powers required to achieve the requested
     * robot motions using the input values from the controler.
     * Then sends these power levels to the motors.
     */
    public void driveRobot(double axial, double lateral, double yaw) {
        // Combine drive and turn for blended motion.
        if (Math.abs(axial) < 0.15) {
            axial = 0.;
        }
        axial = axial * 0.7;  // Note: pushing stick forward gives negative value
        if (Math.abs(lateral) < 0.8) {
            lateral = 0;
        }
        yaw = yaw * 0.6;
        double leftFrontPower = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower = axial - lateral + yaw;
        double rightBackPower = axial + lateral - yaw;
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));
        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }
        // Send calculated power to wheels
        setDrivePower(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
    }

    public void driveRobotSlow(double axial, double lateral, double yaw) {
        double leftFrontPower = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower = axial - lateral + yaw;
        double rightBackPower = axial + lateral - yaw;
        // Send calculated power to wheels
        setDrivePower(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
    }

    public void setDrivePower(double leftFrontPower, double rightFrontPower, double leftBackPower, double rightBackPower) {
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    public long getCurrentLeftFrontPosition() {
        return leftFrontDrive.getCurrentPosition();
    }

    public void moveRobot(double x, double yaw) {
        // Calculate left and right wheel powers.
        myOpMode.telemetry.addData("Manual","Drive %5.2f, Turn %5.2f", x, yaw);
        double leftPower = -(x + yaw);
        double rightPower = -(x - yaw);
        myOpMode.telemetry.addData("Manual","Drive %5.2f, Turn %5.2f", leftPower, rightPower);

        // Normalize wheel powers to ensure they are between -1 and 1.
        double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (max > 1.0) {
            leftPower /= max;
            rightPower /= max;
        }

        // Apply the calculated power to all four motors.
        leftFrontDrive.setPower(leftPower);
        rightFrontDrive.setPower(rightPower);
        leftBackDrive.setPower(leftPower);
        rightBackDrive.setPower(rightPower);
    }

    public void off() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    public void Stop() {
	off();
    }
    public DcMotor getLeftFrontDrive()
    {
	return leftFrontDrive;
    }
    public DcMotor getLeftBackDrive()
    {
	return leftBackDrive;
    }
    public DcMotor getRightFrontDrive()
    {
	return rightFrontDrive;
    }
    public DcMotor getRightBackDrive()
    {
	return rightBackDrive;
    }
    
    public void left_turn_counter(int ticks) {
	left_turn_counter(ticks,TURN_SPEED);
	return;
    }
	public void left_turn_counter(int ticks, double power) {
	long toposition = leftFrontDrive.getCurrentPosition() - ticks;
        leftFrontDrive.setPower(power);
        rightFrontDrive.setPower(-power);
        leftBackDrive.setPower(power);
        rightBackDrive.setPower(-power);
	while(	leftFrontDrive.getCurrentPosition() > toposition)
	    {
		myOpMode.sleep(1);
	    }
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
	return;
    }

    public void right_turn_counter(int ticks) {
	right_turn_counter(ticks,TURN_SPEED);
	return;
    }

    public void right_turn_angle(double angle) {
	int ticks = (int) (angle*4.*770./360.);
	right_turn_counter(ticks,TURN_SPEED);
	return;
    }

    public void left_turn_angle(double angle) {
	int ticks = (int)(angle*4.*740./360.);
	left_turn_counter(ticks,TURN_SPEED);
	return;
    }

    public void right_turn_counter(int ticks, double power) {
	long toposition = leftFrontDrive.getCurrentPosition() + ticks;
        leftFrontDrive.setPower(-power);
        rightFrontDrive.setPower(power);
        leftBackDrive.setPower(-power);
        rightBackDrive.setPower(power);
	while(	leftFrontDrive.getCurrentPosition() < toposition)
	    {
		myOpMode.sleep(1);
	    }
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
	return;
    }
    public void coast()
    {
    leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
	return;
    }
    
    public void brake()
    {
    leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
	return;
    }
    public double getDpadLateralPower()
    {
	return DPAD_POWER_LATERAL;
    }
    public double getDpadAxialPower()
    {
	return DPAD_POWER_AXIAL;
    }
    public double getDpadYawPower()
    {
	return DPAD_POWER_YAW;
    }
    
}
