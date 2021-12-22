package org.firstinspires.ftc.teamcode.ControlSystem;

import com.qualcomm.robotcore.hardware.Gamepad;

import Framework.Drivers.Driver;
import Framework.Drivers.DriverType;
import Framework.Error;
import Framework.GeneralError;
import Framework.Tasks.Clock;
import Framework.Tasks.Task;
import Framework.Tasks.TaskLoop;
import Framework.Tasks.TaskManager;

public class GamePadDriver extends Driver {
    public GamePadDriver(Gamepad gamepad1, Gamepad gamepad2) {
        super(DriverDirectory.GAMEPAD);

        this.pad[PadID.GAMEPAD1.id] = gamepad1;
        this.pad[PadID.GAMEPAD2.id] = gamepad2;
    }

    public double getRightJoyStick_x(PadID id) {
        double value = this.pad[id.id].right_stick_x;
        return Math.pow(value, 4) * Math.signum(value);
    }

    public double getRightJoyStick_y(PadID id) {
        return -this.pad[id.id].right_stick_y * this.pad[id.id].right_stick_y * Math.signum(this.pad[id.id].right_stick_y);
    }

    public double getLeftJoyStick_x(PadID id) {
        double value = this.pad[id.id].left_stick_x;
        return Math.pow(value, 2) * Math.signum(value);
    }

    public double getLeftJoyStick_y(PadID id) {
        double value = this.pad[id.id].left_stick_y;
        return -Math.pow(value, 2) * Math.signum(value);
    }

    public boolean getY(PadID id) {
        return this.pad[id.id].y;
    }

    public boolean getX(PadID id) {
        return this.pad[id.id].x;
    }

    public boolean getB(PadID id) {
        return this.pad[id.id].b;
    }

    public double getRightTrigger(PadID id) {
        return this.pad[id.id].right_trigger;
    }

    public double getLeftTrigger(PadID id) {
        return this.pad[id.id].left_trigger;
    }

    public boolean getRightBumper(PadID id) {
        return this.pad[id.id].right_bumper;
    }

    public boolean getLeftBumper(PadID id) {
        return this.pad[id.id].left_bumper;
    }

    public boolean getRightDPad(PadID id) {
        return this.pad[id.id].dpad_right;
    }

    public boolean getLeftDPad(PadID id) {
        return this.pad[id.id].dpad_left;
    }

    public boolean getUpDPad(PadID id) {
        return this.pad[id.id].dpad_up;
    }

    public boolean getDownDPad(PadID id) {
        return this.pad[id.id].dpad_down;
    }

    Gamepad[] pad = new Gamepad[2];

    public abstract static class MomentarySwitch extends TaskLoop {
        public MomentarySwitch(TaskManager.DriverList driverList, Clock clock) {
            super(driverList, clock);

            this.clock = clock;
        }
        public MomentarySwitch(TaskManager.DriverList driverList) {
            super(driverList);

            this.clock = null;
        }
        public MomentarySwitch(Clock clock) {
            super(clock);

            this.clock = clock;
        }
        public MomentarySwitch() {
            super();

            this.clock = null;
        }

        @Override
        public final Error loop() {
            if(this.getButton()) {
                if(this.clock != null)
                    this.clock.pauseTask(this.getTaskClockID());

                this.state = true;
                this.runTask();

                while (!super.isInterrupted() && this.getButton()) ;

                this.state = false;
                this.stopTask();

                if(this.clock != null)
                    this.clock.resumeTask(this.getTaskClockID());
            }

            return GeneralError.NO_ERROR;
        }

        protected abstract void runTask();
        protected abstract void stopTask();

        public boolean state() {
            return this.state;
        }

        protected abstract boolean getButton();

        private boolean state = false;

        protected final Clock clock;
    }
    public abstract static class ToggleSwitch extends TaskLoop {
        public ToggleSwitch(TaskManager.DriverList driverList, Clock clock) {
            super(driverList, clock);

            this.clock = clock;
        }
        public ToggleSwitch(TaskManager.DriverList driverList) {
            super(driverList);

            this.clock = null;
        }
        public ToggleSwitch(Clock clock) {
            super(clock);

            this.clock = clock;
        }
        public ToggleSwitch() {
            super();

            this.clock = null;
        }

        @Override
        public final Error loop() {
            if (this.getButton()) {
                if(this.clock != null)
                    this.clock.pauseTask(this.getTaskClockID());

                if (this.state)
                    this.stopTask();
                else
                    this.runTask();

                this.state = !this.state;

                while (!super.isInterrupted() && this.getButton()) ;

                if(this.clock != null)
                    this.clock.resumeTask(this.getTaskClockID());
            }

            return GeneralError.NO_ERROR;
        }

        protected abstract void runTask();
        protected abstract void stopTask();

        protected abstract boolean getButton();

        public boolean state() {
            return this.state;
        }

        private boolean state = false;

        private Clock clock = null;
    }

    public enum PadID {
        GAMEPAD1(0), GAMEPAD2(1);

        PadID(int id) {
            this.id =  id;
        }

        public int getId() {
            return this.id;
        }

        private int id;
    }
}
