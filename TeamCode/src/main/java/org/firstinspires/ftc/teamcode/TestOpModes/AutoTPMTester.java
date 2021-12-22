package org.firstinspires.ftc.teamcode.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Disabled
@TeleOp(name="TPM Tests")
public class AutoTPMTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        boolean startTest = false;
        Type type = Type.FORWARD;

        DcMotor motor0 = super.hardwareMap.dcMotor.get("motor0");
        DcMotor motor1 = super.hardwareMap.dcMotor.get("motor1");
        DcMotor motor2 = super.hardwareMap.dcMotor.get("motor2");
        DcMotor motor3 = super.hardwareMap.dcMotor.get("motor3");

        motor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor0.setDirection(DcMotorSimple.Direction.REVERSE);
        motor1.setDirection(DcMotorSimple.Direction.REVERSE);

        super.waitForStart();

        do {
            startTest = false;

            while (!startTest && super.opModeIsActive()) {
                if (super.gamepad1.right_trigger >= .75)
                    startTest = true;
                if (super.gamepad1.x)
                    type = Type.FORWARD;
                if (super.gamepad1.a)
                    type = Type.STRAFE;
                if (super.gamepad1.b)
                    type = Type.ROTATE;

                super.telemetry.addData("Type", type == Type.FORWARD ? "FORWARD" : type == Type.STRAFE ? "STRAFE" : "ROTATE");
                super.telemetry.update();
            }

            motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            if(type == Type.FORWARD) {
                motor0.setTargetPosition(3000);
                motor1.setTargetPosition(3000);
                motor2.setTargetPosition(3000);
                motor3.setTargetPosition(3000);
            }
            if(type == Type.STRAFE) {
                motor0.setTargetPosition(-3000);
                motor1.setTargetPosition(3000);
                motor2.setTargetPosition(3000);
                motor3.setTargetPosition(-3000);
            }
            if(type == Type.ROTATE) {
                motor0.setTargetPosition(500);
                motor1.setTargetPosition(500);
                motor2.setTargetPosition(-500);
                motor3.setTargetPosition(-500);
            }

            motor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            motor0.setPower(0.5);
            motor1.setPower(0.5);
            motor2.setPower(0.5);
            motor3.setPower(0.5);

            while(
                    motor0.isBusy() ||
                    motor1.isBusy() ||
                    motor2.isBusy() ||
                    motor3.isBusy()
            ) {
                super.telemetry.addData("0", "%d", motor0.getCurrentPosition());
                super.telemetry.addData("1", "%d", motor1.getCurrentPosition());
                super.telemetry.addData("2", "%d", motor2.getCurrentPosition());
                super.telemetry.addData("3", "%d", motor3.getCurrentPosition());
                super.telemetry.update();
            }

            super.telemetry.addData("", "DONE");
            super.telemetry.update();
        }while(super.opModeIsActive());


    }

    private enum Type {
        ROTATE,
        STRAFE,
        FORWARD;
    }
}
