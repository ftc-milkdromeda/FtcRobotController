package org.firstinspires.ftc.teamcode.ControlSystem;

import androidx.annotation.NonNull;

import Framework.Error;

public enum RobotError implements Error {
    ARM_NOT_IN_POSITION;

    @NonNull
    @Override
    public String toString() {
        return this.getSource() + ": " + super.toString();
    }

    @Override
    public String getSource() {
        return "RobotError";
    }
}
