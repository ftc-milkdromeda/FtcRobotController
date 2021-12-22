package org.firstinspires.ftc.teamcode.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import Framework.MecanumAlg.DriveTrainDriver;
import Framework.MecanumAlg.MecanumAlg;
import Framework.Units.AngleUnits;

@Disabled
@TeleOp(name = "DriveTrainT test")
public class DriveTrainTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor motor0 = super.hardwareMap.dcMotor.get("motor0");
        DcMotor motor1 = super.hardwareMap.dcMotor.get("motor1");
        DcMotor motor2 = super.hardwareMap.dcMotor.get("motor2");
        DcMotor motor3 = super.hardwareMap.dcMotor.get("motor3");

        MecanumAlg alg = new MecanumAlg();

        super.waitForStart();

        motor0.setPower(0.5);
        motor1.setPower(0.5);
        motor2.setPower(0.5);
        motor3.setPower(0.5);

        while(super.opModeIsActive());

    }
}

