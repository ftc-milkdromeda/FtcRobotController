package org.firstinspires.ftc.teamcode.ControlSystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import Framework.Drivers.Driver;
import Framework.Drivers.DriverType;
import Framework.MecanumAlg.MecanumAlg;
import Framework.Units.AngleUnits;
import Framework.Units.LengthUnits;

public class DriveTrainAuto extends Driver {
    public DriveTrainAuto(HardwareMap map) {
        super(DriverDirectory.DRIVE_TRAIN_AUTO);

        motor0 = (DcMotorEx) map.dcMotor.get("motor0");
        motor1 = (DcMotorEx) map.dcMotor.get("motor1");
        motor2 = (DcMotorEx) map.dcMotor.get("motor2");
        motor3 = (DcMotorEx) map.dcMotor.get("motor3");

        this.motor0.setDirection(DcMotorSimple.Direction.REVERSE);
        this.motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        this.motor2.setDirection(DcMotorSimple.Direction.FORWARD);
        this.motor3.setDirection(DcMotorSimple.Direction.FORWARD);

        motor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void auto_moveForward(double power, double y, LengthUnits units)  {
        final double tickpermeter = 1685.393;
        final double tolerance = 0.01*tickpermeter;

        y = units.toBase(y);

        this.motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.motor0.setTargetPositionTolerance((int)(tolerance));
        this.motor1.setTargetPositionTolerance((int)(tolerance));
        this.motor2.setTargetPositionTolerance((int)(tolerance));
        this.motor3.setTargetPositionTolerance((int)(tolerance));

        this.motor0.setTargetPosition((int)(tickpermeter*y));
        this.motor1.setTargetPosition((int)(tickpermeter*y));
        this.motor2.setTargetPosition((int)(tickpermeter*y));
        this.motor3.setTargetPosition((int)(tickpermeter*y));

        this.motor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        this.motor0.setPower(power);
        this.motor1.setPower(power);
        this.motor2.setPower(power);
        this.motor3.setPower(power);

        while(
                !this.isFinished &&
                        (this.motor0.isBusy() ||
                                this.motor1.isBusy() ||
                                this.motor2.isBusy() ||
                                this.motor3.isBusy())
        );
    }
    public void auto_strafe(double power, double x, LengthUnits units) {
        final double tickpermeter = 1765.745;
        final double tolerance = 0.01*tickpermeter;

        x = units.toBase(x);

        this.motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.motor0.setTargetPositionTolerance((int)(tolerance));
        this.motor1.setTargetPositionTolerance((int)(tolerance));
        this.motor2.setTargetPositionTolerance((int)(tolerance));
        this.motor3.setTargetPositionTolerance((int)(tolerance));

        this.motor0.setTargetPosition((int)(tickpermeter*x));
        this.motor1.setTargetPosition((int)(-tickpermeter*x));
        this.motor2.setTargetPosition((int)(-tickpermeter*x));
        this.motor3.setTargetPosition((int)(tickpermeter*x));

        this.motor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        this.motor0.setPower(power);
        this.motor1.setPower(power);
        this.motor2.setPower(power);
        this.motor3.setPower(power);

        while(
                !this.isFinished &&
                        (this.motor0.isBusy() ||
                                this.motor1.isBusy() ||
                                this.motor2.isBusy() ||
                                this.motor3.isBusy())
        );
    }
    public void auto_rotate(double power, double w, AngleUnits units) {
        final double tickperrad = 393.5168;
        final double tolerance = 0.1*tickperrad;

        w = units.toBase(w);

        this.motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.motor0.setTargetPositionTolerance((int)(tolerance));
        this.motor1.setTargetPositionTolerance((int)(tolerance));
        this.motor2.setTargetPositionTolerance((int)(tolerance));
        this.motor3.setTargetPositionTolerance((int)(tolerance));

        this.motor0.setTargetPosition((int)(tickperrad*w));
        this.motor1.setTargetPosition((int)(tickperrad*w));
        this.motor2.setTargetPosition((int)(-tickperrad*w));
        this.motor3.setTargetPosition((int)(-tickperrad*w));

        this.motor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        this.motor0.setPower(power);
        this.motor1.setPower(power);
        this.motor2.setPower(power);
        this.motor3.setPower(power);

        while(
                !this.isFinished &&
                        (this.motor0.isBusy() ||
                                this.motor1.isBusy() ||
                                this.motor2.isBusy() ||
                                this.motor3.isBusy())
        );
    }

    public boolean isFinished = false;

    private DcMotorEx motor0;
    private DcMotorEx motor1;
    private DcMotorEx motor2;
    private DcMotorEx motor3;
}
