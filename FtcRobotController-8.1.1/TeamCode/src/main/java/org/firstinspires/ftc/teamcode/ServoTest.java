package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
@Disabled
public class ServoTest extends LinearOpMode {
    private Servo grabberServo = null;
    @Override

    public void runOpMode() throws InterruptedException{
        grabberServo = hardwareMap.get(Servo.class, "Grabber_Servo");

        grabberServo.setPosition(0);

        waitForStart();

        grabberServo.setPosition(1);
        sleep(1000);
        grabberServo.setPosition(0);

    }
}


