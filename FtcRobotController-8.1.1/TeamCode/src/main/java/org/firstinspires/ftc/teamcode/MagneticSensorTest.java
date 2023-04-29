package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class MagneticSensorTest extends LinearOpMode {
    // Define variables for our touch sensor and motor
    TouchSensor limit;
    DcMotor liftMotorA;
    DcMotor liftMotorB;

    @Override
    public void runOpMode() {
        // Get the touch sensor and motor from hardwareMap
        limit = hardwareMap.get(TouchSensor.class, "Magnet_Sensor");
        liftMotorA = hardwareMap.get(DcMotor.class, "Lift_Motor_A");
        liftMotorB = hardwareMap.get(DcMotor.class, "Lift_Motor_B");

        // Wait for the play button to be pressed
        waitForStart();

        // Loop while the Op Mode is running
        while (opModeIsActive()) {

            boolean up = gamepad1.dpad_up;
            // If the Magnetic Limit Swtch is pressed, stop the motor
            if (limit.isPressed()) {
                liftMotorA.setPower(0.05);
                liftMotorB.setPower(-0.05);
                telemetry.addData("LimitSensor", "Pressed");
                telemetry.update();
            } else { // Otherwise, run the motor
                telemetry.addData("LimitSensor", "Not Pressed");
                telemetry.update();
                if(up == true) {
                    liftMotorA.setPower(0.3);
                    liftMotorB.setPower(-0.3);
                } else {
                    liftMotorA.setPower(0.05);
                    liftMotorB.setPower(-0.05);
                }
            }

            telemetry.addData("Left Arm Motor Power:", liftMotorA.getPower());
            telemetry.addData("Right Arm Motor Power:", liftMotorB.getPower());
            telemetry.update();
        }
    }
}