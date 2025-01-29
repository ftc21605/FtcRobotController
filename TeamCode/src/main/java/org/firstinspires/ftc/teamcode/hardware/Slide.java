package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


public class Slide {

    /* Declare OpMode members. */
    private final LinearOpMode myOpMode;   // gain access to methods in the calling OpMode.

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    private DcMotor Slide = null;
    // limits
    int maxslideposition = 2980;
    int maxslidehorizontalposition = 2000;
    double slideholdpower = 0.07; // holds the slide in place
    // Define a constructor that allows the OpMode to pass a reference to itself.
    public Slide(LinearOpMode opmode) {
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
        Slide = myOpMode.hardwareMap.get(DcMotor.class, "slide");

        Slide.setDirection(DcMotor.Direction.REVERSE);
        Slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        myOpMode.telemetry.addData(">", "Slide Initialized");
    }

    public void move(double power) {
        Slide.setPower(power);
    }

    public int getCurrentPosition() {
        return Slide.getCurrentPosition();
    }

    public int maxSlidePosition(int armposition)
    {
	if (armposition > 1000)
	    {
		return maxslideposition;
	    }
	else
	    {
		return maxslidehorizontalposition;
	    }
    }
    public double SlideHoldPower()
    {
	return slideholdpower;
    }
    public void Reset(){
            Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void Stop()
    {
	Slide.setPower(0);
    }
    public void MoveToPosition(int position)
    {
        Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
	Slide.setTargetPosition(position);
	Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void MoveTo(int ticks)
    {
	//        Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
	Slide.setTargetPosition(ticks);
        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       	Slide.setPower(0.3);
    }
    public void MoveTo(int ticks, double power)
    {
	//        Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
	Slide.setTargetPosition(ticks);
        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       	Slide.setPower(power);
    }
    public boolean isBusy()
    {
	return Slide.isBusy();
    }
    public double getPower()
    {
	return Slide.getPower();
    }
    public void RunWithoutEncoder()
    {
	Slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
