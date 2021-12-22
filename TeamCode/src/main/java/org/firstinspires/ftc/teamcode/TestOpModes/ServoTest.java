package org.firstinspires.ftc.teamcode.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp(name="ServoTest")
public class ServoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo servo = super.hardwareMap.servo.get("servo");
        double servoPosition = 0.0;
        double incrementAmount = 0.05;

        boolean startTest = false;

        super.waitForStart();

        while(!startTest && super.opModeIsActive()) {
            startTest = super.gamepad1.a;

            if(super.gamepad1.dpad_up)
                servoPosition = (servoPosition + incrementAmount) >= 1 ? 1 : servoPosition + incrementAmount;
            else if(super.gamepad1.dpad_down)
                servoPosition = (servoPosition - incrementAmount) <= -1 ? -1: servoPosition -incrementAmount;
            else if(super.gamepad1.dpad_left)
                incrementAmount -= 0.001;
            else if(super.gamepad1.dpad_right)
                incrementAmount += 0.001;

            while((super.gamepad1.dpad_up == true || super.gamepad1.dpad_down || super.gamepad1.dpad_left || super.gamepad1.dpad_right) && super.opModeIsActive());

            super.telemetry.addData("Servo Position", "%f", servoPosition);
            super.telemetry.addData("Increment Amount", "%f", incrementAmount);
            super.telemetry.update();
        }

        while(super.opModeIsActive()) {
            if(super.gamepad1.dpad_up)
                servoPosition = (servoPosition + incrementAmount) >= 1 ? 1 : servoPosition + incrementAmount;
            else if(super.gamepad1.dpad_down)
                servoPosition = (servoPosition - incrementAmount) <= -1 ? -1: servoPosition -incrementAmount;
            else if(super.gamepad1.dpad_left)
                incrementAmount -= 0.001;
            else if(super.gamepad1.dpad_right)
                incrementAmount += 0.001;

            servo.setPosition(servoPosition);

            while((super.gamepad1.dpad_up == true || super.gamepad1.dpad_down || super.gamepad1.dpad_left || super.gamepad1.dpad_right) && super.opModeIsActive());

            super.telemetry.addData("Servo Position", "%f", servoPosition);
            super.telemetry.addData("Increment Amount", "%f", incrementAmount);
            super.telemetry.update();
        }
    }
}
