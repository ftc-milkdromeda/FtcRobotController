package org.firstinspires.ftc.teamcode.ControlSystem;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class MotorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor motor = super.hardwareMap.dcMotor.get("motor1");
        int motorPosition = 0;
        int incrementAmount = 50;

        boolean startTest = false;

        super.waitForStart();

        while(super.opModeIsActive()) {
            if(super.gamepad1.dpad_up)
                motorPosition+= incrementAmount;
            if(super.gamepad1.dpad_down)
                motorPosition -= incrementAmount;

            if(super.gamepad1.dpad_left)
                incrementAmount -= 5;
            if(super.gamepad1.dpad_right)
                incrementAmount += 5;

            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setTargetPosition(motorPosition);
            motor.setPower(0.3);

            super.telemetry.addData("Servo Position", "%f", motorPosition);
            super.telemetry.addData("Increment Amount", "%f", incrementAmount);
            super.telemetry.update();
        }
    }
}
