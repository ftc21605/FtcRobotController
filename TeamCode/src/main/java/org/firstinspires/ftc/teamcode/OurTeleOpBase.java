package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class OurTeleOpBase extends LinearOpMode {
    public Servo Grabber0;
    @Override
    public void runOpMode() {
    }
    public Servo setup_grabber() {
        Grabber0 = hardwareMap.get(Servo.class, "grabber0");
	return Grabber0;
}
}
