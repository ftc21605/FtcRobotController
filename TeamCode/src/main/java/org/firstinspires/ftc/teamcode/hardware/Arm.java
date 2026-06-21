package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

public class Arm {
    /* Declare OpMode members. */
    private final LinearOpMode myOpMode;   // gain access to methods in the calling OpMode.
    int arm_drop_position = 1650;
    int arm_slow_position = 1500;
    int arm_min_position = 100; // then power off and drop
    int startposition = 818;
    boolean movepos = false; // state of arm MOVE_TO_POSTION -> true
    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    private DcMotor Arm = null;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public Arm(LinearOpMode opmode) {
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
        Arm = myOpMode.hardwareMap.get(DcMotor.class, "arm");

        Arm.setDirection(DcMotor.Direction.FORWARD);
        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //	Arm.setMode(DcMotor.STOP_AND_RESET_ENCODER);

        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        myOpMode.telemetry.addData(">", "Arm Initialized");
    }

    /**
     * Calculates the left/right motor powers required to achieve the requested
     * robot motions using the input values from the controler.
     * Then sends these power levels to the motors.
     */
    public void move(double power) {
        RunWithoutEncoder();
        Arm.setPower(power);
    }
    public void Float()
    {
		        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
    public void Brake()
    {
		        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public int getCurrentPosition() {
        return Arm.getCurrentPosition();
    }

    public int getArmDropPosition() {
        return arm_drop_position;
    }

    public int getArmSlowPosition() {
	return arm_slow_position;
    }
    public int getArmMinPosition() {
	return arm_min_position;
    }
    public void Reset(){
	movepos = false;
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void MoveTo(int ticks)
    {
	MoveTo(ticks,0.3);
    }
    public void MoveTo(int ticks, double power)
    {
	movepos = true;
	Arm.setTargetPosition(ticks);
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       	Arm.setPower(power);
    }
    public boolean isBusy()
    {
	return Arm.isBusy();
    }
    public void Stop()
    {
	Arm.setPower(0);
    }
	    
    public void RunWithoutEncoder()
    {
	movepos = false;
        Arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public double getPower()
    {
	return Arm.getPower();
    }
	
}
