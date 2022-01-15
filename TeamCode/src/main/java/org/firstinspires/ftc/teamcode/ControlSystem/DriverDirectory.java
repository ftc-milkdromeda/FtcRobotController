package org.firstinspires.ftc.teamcode.ControlSystem;

import Framework.Drivers.Driver;
import Framework.Drivers.DriverType;
import Framework.Tasks.TaskManager;

public enum DriverDirectory implements DriverType {
    DRIVE_TRAIN_AUTO,
    DRIVE_TRAIN_TELEOP,
    ARM,
    INTAKE,
    CAPSTONE_HELPER,
    GAMEPAD,
    CAROUSEL,
    CAMERA,
    LENGTH
    ;

    DriverDirectory() {
        this.id = DriverCounter.getNextDriver();
    }

    @Override
    public int getID() {
        return this.id;
    }

    private int id;

    private static class DriverCounter{
        static int getNextDriver() {
            return DriverCounter.currentDriver++;
        }
        static int currentDriver = 0;
    }
}
