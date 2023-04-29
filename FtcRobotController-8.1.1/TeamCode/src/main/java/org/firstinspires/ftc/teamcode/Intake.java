package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
@Disabled
public class Intake extends LinearOpMode{
    private DcMotor intakemotor = null;

    @Override

    public void runOpMode() throws InterruptedException{
        intakemotor = hardwareMap.get(DcMotor.class, "Motor");
        waitForStart();

        intakemotor.setPower(1);
        sleep(90000000);

    }

}
