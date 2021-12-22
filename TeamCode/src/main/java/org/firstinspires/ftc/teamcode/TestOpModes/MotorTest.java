package org.firstinspires.ftc.teamcode.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp(name="MotorTest")
public class MotorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor motor = super.hardwareMap.dcMotor.get("motor");

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /*
        int motorPosition = 0;
        int incrementAmount = 50;

        boolean startTest = false;

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        super.waitForStart();

        while(super.opModeIsActive()) {
            if(super.gamepad1.dpad_up)
                motorPosition+= incrementAmount;
            else if(super.gamepad1.dpad_down)
                motorPosition -= incrementAmount;
            else if(super.gamepad1.dpad_left)
                incrementAmount -= 5;
            else if(super.gamepad1.dpad_right)
                incrementAmount += 5;

            while((super.gamepad1.dpad_up == true || super.gamepad1.dpad_down || super.gamepad1.dpad_left || super.gamepad1.dpad_right) && super.opModeIsActive());

            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setTargetPosition(motorPosition);
            motor.setPower(0.3);

            super.telemetry.addData("Motor Position", "%f", motorPosition);
            super.telemetry.addData("Increment Amount", "%f", incrementAmount);
            super.telemetry.update();
        }

         */
        waitForStart();

        while(super.opModeIsActive()) {
            super.telemetry.addData("Motor Position","%d", motor.getCurrentPosition());
            super.telemetry.update();
        }
    }
}
