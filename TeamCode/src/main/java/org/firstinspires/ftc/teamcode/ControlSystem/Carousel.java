package org.firstinspires.ftc.teamcode.ControlSystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Carousel {
    public Carousel(HardwareMap map) {
        this.motor = map.dcMotor.get("carousel");
    }

    public void setOn() {
        this.motor.setPower(1.0);
    }

    public void setOff() {
        this.motor.setPower(0.0);
    }

    private DcMotor motor;
}
