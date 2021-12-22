package org.firstinspires.ftc.teamcode.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp(name="MotorSpeedTest")
public class MotorSpeedTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor motor = super.hardwareMap.dcMotor.get("motor");
        double motorPosition = 0.0;
        double incrementAmount = 0.05;

        boolean startTest = false;

        super.waitForStart();

        while(!startTest && super.opModeIsActive()) {
            startTest = super.gamepad1.a;

            if(super.gamepad1.dpad_up)
                motorPosition = (motorPosition + incrementAmount) >= 1 ? 1 : motorPosition + incrementAmount;
            else if(super.gamepad1.dpad_down)
                motorPosition = (motorPosition - incrementAmount) <= -1 ? -1: motorPosition -incrementAmount;
            else if(super.gamepad1.dpad_left)
                incrementAmount -= 0.001;
            else if(super.gamepad1.dpad_right)
                incrementAmount += 0.001;

            while((super.gamepad1.dpad_up == true || super.gamepad1.dpad_down || super.gamepad1.dpad_left || super.gamepad1.dpad_right) && super.opModeIsActive());

            super.telemetry.addData("Motor Position", "%f", motorPosition);
            super.telemetry.addData("Increment Amount", "%f", incrementAmount);
            super.telemetry.update();
        }

        while(super.opModeIsActive()) {
            if(super.gamepad1.dpad_up)
                motorPosition = (motorPosition + incrementAmount) >= 1 ? 1 : motorPosition + incrementAmount;
            else if(super.gamepad1.dpad_down)
                motorPosition = (motorPosition - incrementAmount) <= -1 ? -1: motorPosition -incrementAmount;
            else if(super.gamepad1.dpad_left)
                incrementAmount -= 0.001;
            else if(super.gamepad1.dpad_right)
                incrementAmount += 0.001;

            motor.setPower(motorPosition);

            while((super.gamepad1.dpad_up == true || super.gamepad1.dpad_down || super.gamepad1.dpad_left || super.gamepad1.dpad_right) && super.opModeIsActive());

            super.telemetry.addData("Motor Position", "%f", motorPosition);
            super.telemetry.addData("Increment Amount", "%f", incrementAmount);
            super.telemetry.update();
        }
    }
}
