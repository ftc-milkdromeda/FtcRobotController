package org.firstinspires.ftc.teamcode.ControlSystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.OpModes.TeleOp.TeleOpMode;

import Framework.Drivers.Driver;
import Framework.Drivers.DriverType;
import Framework.Error;
import Framework.Tasks.Clock;
import Framework.Tasks.Task;
import Framework.Tasks.TaskManager;

public class Carousel extends GamePadDriver.MomentarySwitch{
    public Carousel(Clock clock, TeleOpMode.Side side) {
        super(new TaskManager.DriverList(new DriverType[]{DriverDirectory.CAROUSEL}, new DriverType[] {DriverDirectory.GAMEPAD}));

        this.side = side;
    }

    @Override
    protected Error init() {
        this.gamepad = (GamePadDriver) super.unboundDrivers[0];
        this.carousel = (CarouselDriver) super.boundDrivers[0];
        return super.init();
    }

    @Override
    protected void runTask() {
        this.carousel.setOn(this, this.side);
    }

    public void setSide(TeleOpMode.Side side) {
        this.side = side;
    }

    @Override
    protected void stopTask() {
        this.carousel.setOff(this);
    }

    @Override
    protected boolean getButton() {
        return this.gamepad.getY(GamePadDriver.PadID.GAMEPAD2);
    }

    public static class CarouselDriver extends Driver {
        public CarouselDriver(HardwareMap map) {
            super(DriverDirectory.CAROUSEL);

            this.motor = map.dcMotor.get("carousel");
            this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        public void setOn(Task task, TeleOpMode.Side side) {
            Error error = super.validateCall(task);
            this.motor.setPower(0.2 * side.getMultiplier());
        }

        public void setOff(Task task) {
            this.motor.setPower(0.0);
        }

        private DcMotor motor;
    }

    private CarouselDriver carousel;
    private GamePadDriver gamepad;

    private TeleOpMode.Side side;
}
