package org.firstinspires.ftc.teamcode.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import Framework.MecanumAlg.DriveTrainDriver;
import Framework.MecanumAlg.MecanumAlg;
import Framework.Units.AngleUnits;

@Disabled
@TeleOp(name = "TestDriveTrain")
public class DriveTrainTestOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor motor0 = super.hardwareMap.dcMotor.get("motor0");
        DcMotor motor1 = super.hardwareMap.dcMotor.get("motor1");
        DcMotor motor2 = super.hardwareMap.dcMotor.get("motor2");
        DcMotor motor3 = super.hardwareMap.dcMotor.get("motor3");

        motor0.setDirection(DcMotorSimple.Direction.REVERSE);
        motor1.setDirection(DcMotorSimple.Direction.REVERSE);

        motor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        MecanumAlg alg = new MecanumAlg();

        super.waitForStart();

        while(super.opModeIsActive()) {
            double x = super.gamepad1.left_stick_x;
            double y = super.gamepad1.left_stick_y;
            double w = super.gamepad1.right_stick_x;

            double mag = Math.sqrt(x*x + y*y);

            //calculate linear direction
            DriveTrainDriver.MotorSettingList max_setting = alg.getMotorSettingsNORM(new DriveTrainDriver.NormalizedMovementParameter(
                    1, Math.acos((mag == 0 ? 0 : x / mag)),  AngleUnits.RAD, 0
            ));
            DriveTrainDriver.MotorSettingList settings = alg.getMotorSettingsNORM(new DriveTrainDriver.NormalizedMovementParameter(x, y, 0));

            settings = new DriveTrainDriver.MotorSettingList(
                    Math.abs(settings.w0 / max_setting.w0) * Math.signum(settings.w0),
                    Math.abs(settings.w1 / max_setting.w1) * Math.signum(settings.w1),
                    Math.abs(settings.w2 / max_setting.w2) * Math.signum(settings.w2),
                    Math.abs(settings.w3 / max_setting.w3) * Math.signum(settings.w3)
                    );

            //calculate pivot
            double pivot_difference = (1 - Math.max(Math.abs(settings.w0), Math.abs(settings.w3))) * w;

            settings = new DriveTrainDriver.MotorSettingList(
                    settings.w0 + pivot_difference,
                    settings.w1 + pivot_difference,
                    settings.w2 - pivot_difference,
                    settings.w3 - pivot_difference
            );


            //set motor
            motor0.setPower(settings.w0);
            motor1.setPower(settings.w1);
            motor2.setPower(settings.w2);
            motor3.setPower(settings.w3);

            super.telemetry.addData("motor0", "%f", settings.w0);
            super.telemetry.addData("motor1", "%f", settings.w1);
            super.telemetry.addData("motor2", "%f", settings.w2);
            super.telemetry.addData("motor3", "%f", settings.w3);

            super.telemetry.addData("X", "%f", x);
            super.telemetry.addData("Y", "%f", y);
            super.telemetry.addData("W", "%f", w);
            super.telemetry.addData("dPivot", "%f", pivot_difference);
            super.telemetry.update();
        }

    }
}
