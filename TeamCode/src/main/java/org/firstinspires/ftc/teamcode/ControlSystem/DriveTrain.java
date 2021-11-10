package org.firstinspires.ftc.teamcode.ControlSystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


import Framework.MecanumAlg.DriveTrainDriver;
import Framework.MecanumAlg.MecanumAlg;
import Framework.Units.AngleUnits;

public class DriveTrain {
    public DriveTrain(HardwareMap map) {
        motor0 = map.dcMotor.get("motor0");
        motor1 = map.dcMotor.get("motor1");
        motor2 = map.dcMotor.get("motor2");
        motor3 = map.dcMotor.get("motor3");

        motor0.setDirection(DcMotorSimple.Direction.REVERSE);
        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);
        motor3.setDirection(DcMotorSimple.Direction.REVERSE);

        motor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.alg = new MecanumAlg();
    }

    public void gamepad_setDriveTrain(double x, double y, double w) {
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
    }

    public void auto_setPosition(double x, double y, double w) {

    }

    private DcMotor motor0;
    private DcMotor motor1;
    private DcMotor motor2;
    private DcMotor motor3;

    private MecanumAlg alg;
}
