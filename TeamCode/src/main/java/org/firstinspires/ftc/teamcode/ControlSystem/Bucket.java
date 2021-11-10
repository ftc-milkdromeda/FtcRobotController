package org.firstinspires.ftc.teamcode.ControlSystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Bucket {
    public Bucket(HardwareMap map) {
        this.ramp = map.servo.get("ramp");
        this.elevatorBucket = map.servo.get("elevator");

        this.intake = map.dcMotor.get("intake");
        this.lift = map.dcMotor.get("lift");

        this.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void toggleIntake() {
        this.state = State.INTAKE_MODE;

        this.toggleRampDown();
    }

    public void toggleTransfer() {
        this.state = State.TRANSFER_MODE;

        this.toggleRampUp();
    }

    public void raiseLift() {
        if(this.state != State.TRANSFER_MODE)
            return;

        this.toggleElevatorUp();
    }

    public void lowerLift() {
        if(this.state != State.TRANSFER_MODE)
            return;

        this.toggleElevatorDown();
    }

    public void dumpElement() {
        if(this.state != State.TRANSFER_MODE)
            return;

        this.toggleBucketDump();
    }

    public void resetBucket() {
        if(this.state != State.TRANSFER_MODE)
            return;

        this.toggleBucketReset();
    }

    public void intakeElement() {
        if(this.state != State.INTAKE_MODE)
            return;

        this.intake.setPower(1.0);
    }

    private void toggleRampDown() {
        this.ramp.setPosition(1.0);
    }

    private void toggleRampUp() {
        this.ramp.setPosition(0.0);
    }

    private void toggleElevatorUp() {
        if(this.lift.isBusy())
            return;

        this.lift.setTargetPosition(100);
        this.lift.setPower(.5);
    }

    private void toggleElevatorDown() {
        if(this.lift.isBusy())
            return;

        this.lift.setTargetPosition(0);
        this.lift.setPower(.5);
    }

    private void toggleBucketDump() {
        this.elevatorBucket.setPosition(1.0);
    }

    private void toggleBucketReset() {
        this.elevatorBucket.setPosition(0.0);
    }

    private enum State {
        INTAKE_MODE,
        TRANSFER_MODE;
    }

    private Servo ramp;
    private Servo elevatorBucket;

    private DcMotor intake;
    private DcMotor lift;

    private State state;
}
