package org.firstinspires.ftc.teamcode.ControlSystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.OpModes.TeleOp.TeleOpMode;

import Framework.Drivers.Driver;
import Framework.Drivers.DriverType;
import Framework.Error;
import Framework.GeneralError;
import Framework.MecanumAlg.DriveTrainDriver;
import Framework.MecanumAlg.MecanumAlg;
import Framework.Tasks.Clock;
import Framework.Tasks.Task;
import Framework.Tasks.TaskLoop;
import Framework.Tasks.TaskManager;
import Framework.Units.AngleUnits;

public class DriveTrainTeleOp extends TaskLoop {
    public DriveTrainTeleOp(Clock clock) {
        super(new TaskManager.DriverList(
                new DriverType[] {DriverDirectory.DRIVE_TRAIN_TELEOP},
                new DriverType[] {DriverDirectory.GAMEPAD}),
                clock);
    }

    @Override
    protected Error init() {
        this.driveTrain = (DriveTrain) super.boundDrivers[0];
        this.gamepad = (GamePadDriver) super.unboundDrivers[0];

        return GeneralError.NO_ERROR;
    }

    @Override
    public Error loop() {
        Error error = this.driveTrain.gamepad_setDriveTrain(
                this,
                this.gamepad.getLeftJoyStick_x(GamePadDriver.PadID.GAMEPAD1),
                this.gamepad.getLeftJoyStick_y(GamePadDriver.PadID.GAMEPAD1),
                this.gamepad.getRightJoyStick_x(GamePadDriver.PadID.GAMEPAD1)
        );

        if(error != GeneralError.NO_ERROR)
            return error;

        if(this.gamepad.getLeftJoyStick_x(GamePadDriver.PadID.GAMEPAD1) > 0.1)
            this.driveTrain.runAuxWheel(this, this.gamepad.getLeftTrigger(GamePadDriver.PadID.GAMEPAD1) > 0.5 ? 1.0 : 0.0);

        return GeneralError.NO_ERROR;
    }

    private DriveTrain driveTrain;
    private GamePadDriver gamepad;

    public static class DriveTrain extends Driver {
        public DriveTrain(HardwareMap map) {
            super(DriverDirectory.DRIVE_TRAIN_TELEOP);

            this.motor0 = map.dcMotor.get("motor0");
            this.motor1 = map.dcMotor.get("motor1");
            this.motor2 = map.dcMotor.get("motor2");
            this.motor3 = map.dcMotor.get("motor3");

            this.auxWheel = map.dcMotor.get("aux");

            this.motor0.setDirection(DcMotorSimple.Direction.REVERSE);
            this.motor1.setDirection(DcMotorSimple.Direction.REVERSE);
            this.motor2.setDirection(DcMotorSimple.Direction.FORWARD);
            this.motor3.setDirection(DcMotorSimple.Direction.FORWARD);

            this.auxWheel.setDirection(DcMotorSimple.Direction.REVERSE);

            this.motor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            this.auxWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            this.alg = new MecanumAlg();
        }

        public Error gamepad_setDriveTrain(Task task, double x, double y, double w) {
            Error error = super.validateCall(task);
            if(error != GeneralError.NO_ERROR)
                return error;

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

            this.motor0.setPower(settings.w0);
            this.motor1.setPower(settings.w1);
            this.motor2.setPower(settings.w2);
            this.motor3.setPower(settings.w3);

            return GeneralError.NO_ERROR;
        }

        public Error runAuxWheel(Task task, double power) {
            Error error = super.validateCall(task);
            if(error != GeneralError.NO_ERROR)
                return error;

            this.auxWheel.setPower(power);

            return GeneralError.NO_ERROR;
        }

        private DcMotor motor0;
        private DcMotor motor1;
        private DcMotor motor2;
        private DcMotor motor3;

        private DcMotor auxWheel;

        private MecanumAlg alg;
    }
}
