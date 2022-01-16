package org.firstinspires.ftc.teamcode.ControlSystem;

import android.widget.ArrayAdapter;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.sun.tools.javac.jvm.Gen;

import org.checkerframework.checker.units.qual.C;

import Framework.Drivers.Driver;
import Framework.Drivers.DriverType;
import Framework.Error;
import Framework.GeneralError;
import Framework.Tasks.Clock;
import Framework.Tasks.Task;
import Framework.Tasks.TaskLoop;
import Framework.Tasks.TaskManager;
import Framework.Units.AngleUnits;
import Framework.Units.AngularVelocityUnits;

public class ArmSystem extends TaskLoop {
    public ArmSystem(Clock clock) {
        super(
                new TaskManager.DriverList(
                        new DriverType[] {DriverDirectory.ARM, DriverDirectory.INTAKE, DriverDirectory.CAPSTONE_HELPER},
                        new DriverType[]{DriverDirectory.GAMEPAD}
                )
                );

        this.currentState = State.INTAKE;

        this.intakeSwitch = new GamePadDriver.MomentarySwitch() {
            @Override
            protected void runTask() {
                if(ArmSystem.this.currentState != ArmSystem.State.INTAKE) {
                    ArmSystem.this.arm.placeCapStone(ArmSystem.this);
                }
                else {
                    if(ArmSystem.this.intake.intakeReverse(ArmSystem.this) != GeneralError.NO_ERROR)
                        return;
                    if(ArmSystem.this.arm.setIntake(ArmSystem.this) != GeneralError.NO_ERROR)
                        return;

                    try {
                        Thread.sleep(500);
                    } catch (InterruptedException e) {
                        return;
                    }

                    if(ArmSystem.this.intake.intakeOn(ArmSystem.this) != GeneralError.NO_ERROR)
                        return;
                }
            }

            @Override
            protected void stopTask() {
                if(ArmSystem.this.currentState == ArmSystem.State.INTAKE) {
                    ArmSystem.this.intake.intakeSlow(ArmSystem.this);
                    ArmSystem.this.arm.setHome(ArmSystem.this);

                    try {
                        Thread.sleep(500);
                    } catch (InterruptedException e) {
                        return;
                    }

                    ArmSystem.this.intake.intakeOff(ArmSystem.this);

                }
                else {
                    ArmSystem.this.arm.resetDump(ArmSystem.this);
                }
            }

            @Override
            protected boolean getButton() {
                return ArmSystem.this.gamepad.getRightBumper(GamePadDriver.PadID.GAMEPAD1);
            }
        };

        this.setHover = new GamePadDriver.ToggleSwitch() {
            @Override
            protected void runTask() {
                if(ArmSystem.this.currentState == ArmSystem.State.INTAKE) {
                    ArmSystem.this.arm.setHigh(ArmSystem.this, true);
                }
                else {
                    ArmSystem.this.arm.hoverArm(ArmSystem.this);
                    ArmSystem.this.currentState = ArmSystem.State.CAPSTONE_HOVER;
                }
            }

            @Override
            protected void stopTask() {
                if(ArmSystem.this.currentState == ArmSystem.State.INTAKE) {
                    ArmSystem.this.arm.setHome(ArmSystem.this);
                }
                else {
                    ArmSystem.this.arm.setCapHeight(ArmSystem.this);
                    ArmSystem.this.currentState = ArmSystem.State.CAPSTONE;
                }
            }

            @Override
            protected boolean getButton() {
                return ArmSystem.this.gamepad.getY(GamePadDriver.PadID.GAMEPAD1);
            }
        };
        this.activate = new GamePadDriver.MomentarySwitch() {
            @Override
            protected void runTask() {
                if(ArmSystem.this.currentState == ArmSystem.State.INTAKE) {
                    ArmSystem.this.arm.dump(ArmSystem.this);
                }
                else if(ArmSystem.this.currentState == ArmSystem.State.CAPSTONE_HOVER) {
                    ArmSystem.this.arm.setCapIntake(ArmSystem.this);
                }
                else {
                    ArmSystem.this.arm.setCapIntakeHelper(ArmSystem.this);
                }
            }

            @Override
            protected void stopTask() {
                if(ArmSystem.this.currentState == ArmSystem.State.INTAKE) {
                    ArmSystem.this.arm.resetDump(ArmSystem.this);
                }
                else if(ArmSystem.this.currentState == ArmSystem.State.CAPSTONE_HOVER) {
                    ArmSystem.this.arm.hoverArm(ArmSystem.this);
                }
                else {
                    ArmSystem.this.arm.resetCapArm(ArmSystem.this);
                }
            }

            @Override
            protected boolean getButton() {
                return ArmSystem.this.gamepad.getB(GamePadDriver.PadID.GAMEPAD1);
            }
        };

        this.helperIntake = new GamePadDriver.MomentarySwitch() {
            @Override
            protected void runTask() {
                if(ArmSystem.this.helperState == HelperState.ON) {
                    ArmSystem.this.helper.setDown(ArmSystem.this);
                }
            }

            @Override
            protected void stopTask() {
                if(ArmSystem.this.helperState == HelperState.ON) {
                    ArmSystem.this.helper.resetNormal(ArmSystem.this);
                }
            }

            @Override
            protected boolean getButton() {
                return ArmSystem.this.gamepad.getLeftBumper(GamePadDriver.PadID.GAMEPAD1);
            }
        };
        this.helperOn = new GamePadDriver.ToggleSwitch() {
            @Override
            protected void runTask() {
                if(ArmSystem.this.helperState == HelperState.OFF)
                    ArmSystem.this.helper.setNormal(ArmSystem.this);
                ArmSystem.this.helperState = HelperState.ON;
            }

            @Override
            protected void stopTask() {
                if(ArmSystem.this.helperState == HelperState.ON)
                    ArmSystem.this.helper.setUp(ArmSystem.this);
                ArmSystem.this.helperState = HelperState.OFF;
            }

            @Override
            protected boolean getButton() {
                return ArmSystem.this.gamepad.getX(GamePadDriver.PadID.GAMEPAD1);
            }
        };
    }

    @Override
    protected Error init() {
        this.gamepad = (GamePadDriver)super.unboundDrivers[0];

        this.arm = (ArmDriver)super.boundDrivers[0];
        this.intake = (IntakeDriver)super.boundDrivers[1];
        this.helper = (CapstoneHelper)super.boundDrivers[2];

        TaskManager.startTask(this.intakeSwitch);
        TaskManager.startTask(this.activate);
        TaskManager.startTask(this.setHover);
        TaskManager.startTask(this.helperOn);
        TaskManager.startTask(this.helperIntake);

        return GeneralError.NO_ERROR;
    }

    @Override
    protected Error destructor() {
        TaskManager.stopTask(this.intakeSwitch.getTaskID());
        TaskManager.stopTask(this.activate.getTaskID());
        TaskManager.stopTask(this.setHover.getTaskID());
        TaskManager.stopTask(this.helperOn.getTaskID());
        TaskManager.stopTask(this.helperIntake.getTaskID());

        return GeneralError.NO_ERROR;
    }

    @Override
    public synchronized Error loop() {
        if(this.gamepad.getRightDPad(GamePadDriver.PadID.GAMEPAD1)) {
            this.currentState = State.INTAKE;
            this.arm.setHome(this);
        }
        if(this.gamepad.getLeftDPad(GamePadDriver.PadID.GAMEPAD1)) {
            this.currentState = State.CAPSTONE;
            this.arm.setCapHeight(this);
        }

        return GeneralError.NO_ERROR;
    }

    public static class ArmDriver extends Driver {
        public ArmDriver(HardwareMap map) {
            super(DriverDirectory.ARM);

            this.arm = (DcMotorEx) map.dcMotor.get("arm");

            this.bucket = map.servo.get("bucket");

            this.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            this.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            this.currentPos = Position.HOME;
        }

        public synchronized Error dump(Task task) {
            final double dumpAmount = 0.5;
            if(this.currentPos == Position.HIGH || this.currentPos == Position.MID) {
                this.bucket.setPosition(this.currentPos.bucketSetting - dumpAmount);
                return GeneralError.NO_ERROR;
            }

            return RobotError.ARM_NOT_IN_POSITION;
        }
        public synchronized Error resetDump(Task task) {
            if(this.currentPos == Position.CAP_HEIGHT ||this.currentPos == Position.HIGH || this.currentPos == Position.MID) {
                this.bucket.setPosition(this.currentPos.bucketSetting);
                return GeneralError.NO_ERROR;
            }

            return RobotError.ARM_NOT_IN_POSITION;
        }

        public synchronized Error dumpLow(Task task) {
            if(this.currentPos == Position.LOW) {
                this.arm.setTargetPosition((int)Position.LOW_DUMP.motorSetting);
                this.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                this.arm.setPower(ArmDriver.motorPower);

                this.currentPos = Position.LOW_DUMP;
                return GeneralError.NO_ERROR;
            }

            return RobotError.ARM_NOT_IN_POSITION;
        }
        public synchronized Error dumpLowReset(Task task) {
            if(this.currentPos == Position.LOW_DUMP) {
                this.arm.setTargetPosition((int)this.currentPos.motorSetting);
                this.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                this.arm.setPower(ArmDriver.motorPower);

                this.currentPos = Position.LOW;
                return GeneralError.NO_ERROR;
            }

            return RobotError.ARM_NOT_IN_POSITION;

        }

        public synchronized Error setHome(Task task) {
            Error error = super.validateCall(task);
            if(error != GeneralError.NO_ERROR)
                return error;

            this.bucket.setPosition(Position.HOME.bucketSetting);

            this.arm.setTargetPositionTolerance(ArmDriver.tolerance);
            this.arm.setTargetPosition((int)Position.HOME.motorSetting);
            this.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.arm.setPower(ArmDriver.motorPower);

            while(!Thread.interrupted() && this.arm.isBusy());

            this.currentPos = Position.HOME;

            return GeneralError.NO_ERROR;
        }

        public synchronized Error setHigh(Task task, boolean syncBucket) {
            Error error = super.validateCall(task);
            if(error != GeneralError.NO_ERROR)
                return error;

            if(syncBucket) {
                ServoSpeed servoRegulator = new ServoSpeed(
                        this.bucket,
                        this.currentPos.bucketSetting,
                        Position.HIGH.bucketSetting,
                        ArmDriver.motorPower * ArmDriver.maxMotorSpeed,
                        AngularVelocityUnits.RAD_SEC
                );

                this.arm.setTargetPosition((int)Position.HIGH.motorSetting);
                this.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                TaskManager.startTask(servoRegulator);

                this.arm.setPower(ArmDriver.motorPower);

                while(!Thread.interrupted() && this.arm.isBusy());
            }
            else {
                this.arm.setTargetPosition((int)Position.HIGH.motorSetting);
                this.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                this.arm.setPower(ArmDriver.motorPower);

                while(!Thread.interrupted() && this.arm.isBusy());

                this.bucket.setPosition(Position.HIGH.bucketSetting);
            }

            this.currentPos = Position.HIGH;

            return GeneralError.NO_ERROR;
        }
        public synchronized Error setMid(Task task, boolean syncBucket) {
            Error error = super.validateCall(task);
            if(error != GeneralError.NO_ERROR)
                return error;

            if(syncBucket) {
                ServoSpeed servoRegulator = new ServoSpeed(
                        this.bucket,
                        this.currentPos.bucketSetting,
                        Position.MID.bucketSetting,
                        ArmDriver.motorPower * ArmDriver.maxMotorSpeed,
                        AngularVelocityUnits.RAD_SEC
                );

                this.arm.setTargetPosition((int)Position.MID.motorSetting);
                this.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                TaskManager.startTask(servoRegulator);

                this.arm.setPower(ArmDriver.motorPower);

                while(!Thread.interrupted() && this.arm.isBusy());
            }
            else {
                this.arm.setTargetPosition((int)Position.MID.motorSetting);
                this.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                this.arm.setPower(ArmDriver.motorPower);

                while(!Thread.interrupted() && this.arm.isBusy());

                this.bucket.setPosition(Position.MID.bucketSetting);
            }

            this.currentPos = Position.MID;

            return GeneralError.NO_ERROR;
        }
        public synchronized Error setLow(Task task, boolean syncBucket) {
            Error error = super.validateCall(task);
            if(error != GeneralError.NO_ERROR)
                return error;

            if(syncBucket) {
                ServoSpeed servoRegulator = new ServoSpeed(
                        this.bucket,
                        this.currentPos.bucketSetting,
                        Position.LOW.bucketSetting,
                        ArmDriver.motorPower * ArmDriver.maxMotorSpeed,
                        AngularVelocityUnits.RAD_SEC
                );

                this.arm.setTargetPosition((int)Position.LOW.motorSetting);
                this.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                TaskManager.startTask(servoRegulator);

                this.arm.setPower(ArmDriver.motorPower);

                while(!Thread.interrupted() && this.arm.isBusy());
            }
            else {
                this.arm.setTargetPosition((int)Position.LOW.motorSetting);
                this.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                this.arm.setPower(ArmDriver.motorPower);

                while(!Thread.interrupted() && this.arm.isBusy());

                this.bucket.setPosition(Position.LOW.bucketSetting);
            }

            this.currentPos = Position.LOW;

            return GeneralError.NO_ERROR;
        }

        public synchronized Error setIntake(Task task) {
            Error error = super.validateCall(task);
            if(error != GeneralError.NO_ERROR)
                return error;

            if(this.currentPos != Position.HOME)
                return RobotError.ARM_NOT_IN_POSITION;

            this.bucket.setPosition(Position.INTAKE.bucketSetting);

            this.currentPos = Position.INTAKE;

            return GeneralError.NO_ERROR;
        }

        public synchronized Error setCapHeight(Task task) {
            Error error = super.validateCall(task);
            if(error != GeneralError.NO_ERROR)
                return error;

            this.arm.setTargetPositionTolerance(ArmDriver.tolerance);
            this.arm.setTargetPosition((int)Position.CAP_HEIGHT.motorSetting);

            this.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.arm.setPower(ArmDriver.motorPower);

            while(!Thread.interrupted() && this.arm.isBusy());

            this.bucket.setPosition(Position.CAP_HEIGHT.bucketSetting);

            this.currentPos = Position.CAP_HEIGHT;

            return GeneralError.NO_ERROR;
        }
        public synchronized Error placeCapStone(Task task) {
            final double dumpAmount = 0.5;

            Error error = super.validateCall(task);
            if(error != GeneralError.NO_ERROR)
                return error;

            if(this.currentPos != Position.CAP_HEIGHT)
                return RobotError.ARM_NOT_IN_POSITION;

            this.bucket.setPosition(this.currentPos.bucketSetting - dumpAmount);

            return GeneralError.NO_ERROR;
        }

        public synchronized Error hoverArm(Task task) {
            Error error = super.validateCall(task);
            if(error != GeneralError.NO_ERROR)
                return error;

            this.arm.setTargetPositionTolerance(ArmDriver.tolerance);
            this.arm.setTargetPosition((int)Position.CAP_HOVER.motorSetting);

            this.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.arm.setPower(ArmDriver.motorPower);

            while(!Thread.interrupted() && this.arm.isBusy());

            this.bucket.setPosition(Position.CAP_HOVER.bucketSetting);

            this.currentPos = Position.CAP_HOVER;

            return GeneralError.NO_ERROR;
        }

        public synchronized Error setCapIntake(Task task) {
            Error error = super.validateCall(task);
            if(error != GeneralError.NO_ERROR)
                return error;

            if(this.currentPos != Position.CAP_HOVER)
                return RobotError.ARM_NOT_IN_POSITION;

            this.arm.setTargetPositionTolerance(ArmDriver.tolerance);
            this.arm.setTargetPosition((int) Position.CAP_INTAKE.motorSetting);

            this.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            this.arm.setPower(ArmDriver.motorPower);

            while(!Thread.interrupted() && this.arm.isBusy());

            this.currentPos = Position.CAP_INTAKE;

            return GeneralError.NO_ERROR;
        }
        public synchronized Error setCapIntakeHelper(Task task) {
            Error error = super.validateCall(task);
            if(error != GeneralError.NO_ERROR)
                return error;

            if(this.currentPos != Position.CAP_HEIGHT)
                return RobotError.ARM_NOT_IN_POSITION;


            this.arm.setTargetPositionTolerance(ArmDriver.tolerance);
            this.arm.setTargetPosition((int)Position.CAP_INTAKE_HELPER.motorSetting);

            this.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.arm.setPower(ArmDriver.motorPower * 0.5);

            while(!Thread.interrupted() && this.arm.isBusy());

            this.bucket.setPosition(Position.CAP_INTAKE_HELPER.bucketSetting);

            this.currentPos = Position.CAP_INTAKE_HELPER;

            return GeneralError.NO_ERROR;
        }
        public synchronized Error resetCapArm(Task task) {
            final double resetBucket = 0.7;

            if(this.currentPos != Position.CAP_INTAKE_HELPER)
                return RobotError.ARM_NOT_IN_POSITION;

            this.arm.setTargetPosition((int) this.currentPos.motorSetting - 25);
            this.arm.setTargetPositionTolerance(ArmDriver.tolerance);

            this.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.arm.setPower(0.1);

            while(this.arm.isBusy() && !Thread.interrupted())

            this.bucket.setPosition(resetBucket);

            this.setCapHeight(task);

            return GeneralError.NO_ERROR;
        }

        private static double ticksToAngle(AngleUnits units, int ticks) {
            return units.toUnit(ticks / ArmDriver.ticksPerAngle);
        }
        private static double angleToServoMag(double angle, AngleUnits units) {
            angle = units.toBase(angle);
            return angle / ArmDriver.maxServoAngle;
        }

        private final static int ticksPerAngle = 100;
        private final static double maxServoAngle = AngleUnits.DEG.toBase(310);

        private DcMotorEx arm;

        private Servo bucket;

        private Position currentPos;

        private final static double motorPower = 0.2;
        private final static double maxMotorSpeed = AngularVelocityUnits.REV_MIN.toBase(120);

        private static final int tolerance = 10;

        private enum Position {
            INTAKE(0, 0.424),
            HOME(0,0),
            HIGH(906,0.95),
            MID(1050,.95),
            LOW(1200,0.95),

            LOW_DUMP(0,0),

            CAP_HEIGHT(738,0.9),
            CAP_HOVER(1077, 1.0),
            CAP_INTAKE_HELPER(1079,0.65),
            CAP_INTAKE(1200,1.0);

            Position(double motorSetting, double bucketSetting) {
                this.motorSetting = motorSetting;
                this.bucketSetting = bucketSetting;
            }

            final double motorSetting;
            final double bucketSetting;
        }

        private class ServoSpeed extends TaskLoop{
            public ServoSpeed(Clock clock, Servo servo, double currentPos, double finalPos, double speed, AngularVelocityUnits units) {
                super(clock);

                TaskManager.startTask(clock);

                this.servo = servo;

                this.currentPos = currentPos;
                this.finalPos = finalPos;

                speed = units.toBase(speed * Math.signum(finalPos - currentPos));

                this.increment = speed * this.timeInterval / ArmDriver.maxServoAngle;

                this.numOfIterations = (int) ((finalPos - currentPos) / this.increment);

                this.clock = clock;
            }
            public ServoSpeed(Servo servo, double currentPos, double finalPos, double speed, AngularVelocityUnits units) {
                this(new Clock(10), servo, currentPos, finalPos, speed, units);
            }

            @Override
            protected Error destructor() {
                TaskManager.stopTask(this.clock.getTaskID());
                this.servo.setPosition(this.finalPos);

                return GeneralError.NO_ERROR;
            }

            @Override
            public Error loop() {
                if(iteration++ >= numOfIterations)
                    super.interrupt();
                this.currentPos += this.increment;
                this.servo.setPosition(currentPos);


                return GeneralError. NO_ERROR;
            }

            private final Servo servo;

            private final double timeInterval = 0.1;

            private final int numOfIterations;
            private final double increment;

            private int iteration = 0;

            private double currentPos;
            private final double finalPos;

            private final Clock clock;
        }
    }

    public static class IntakeDriver extends Driver {
        public IntakeDriver(HardwareMap map) {
            super(DriverDirectory.INTAKE);

            this.intakeMotor = map.crservo.get("IntakeServo");
        }

        public Error intakeOn(Task task) {
            Error error = super.validateCall(task);
            if(error != GeneralError.NO_ERROR)
                return error;

            System.out.println("intakeOn");

            this.intakeMotor.setPower(-1.0);

            return GeneralError.NO_ERROR;
        }

        public Error intakeOff(Task task) {
            Error error = super.validateCall(task);
            if(error != GeneralError.NO_ERROR)
                return error;

            System.out.println("intakeOff");

            this.intakeMotor.setPower(0.0);

            return GeneralError.NO_ERROR;
        }

        public Error intakeReverse(Task task) {
            Error error = super.validateCall(task);
            if(error != GeneralError.NO_ERROR)
                return error;

            this.intakeMotor.setPower(0.3);

            return GeneralError.NO_ERROR;

        }

        public Error intakeSlow(Task task) {
            Error error = super.validateCall(task);
            if(error != GeneralError.NO_ERROR)
                return error;

            this.intakeMotor.setPower(-0.3);

            return GeneralError.NO_ERROR;

        }

        CRServo intakeMotor;
    }

    public static class CapstoneHelper extends Driver {
        public CapstoneHelper(HardwareMap map) {
            super(DriverDirectory.CAPSTONE_HELPER);

            this.helper = map.servo.get("helper");
        }

        public Error setUp(Task task) {
            Error error = super.validateCall(task);
            if(error != GeneralError.NO_ERROR)
                return error;

            this.currentPos = 0.0;
            helper.setPosition(currentPos);


            return GeneralError.NO_ERROR;
        }

        public Error setNormal(Task task) {
            Error error = super.validateCall(task);
            if(error != GeneralError.NO_ERROR)
                return error;

            this.currentPos = 0.275;
            helper.setPosition(currentPos);


            return GeneralError.NO_ERROR;
        }
        public Error resetNormal(Task task) {
            Error error = super.validateCall(task);
            if(error != GeneralError.NO_ERROR)
                return error;

            double buffer = this.currentPos;
            this.currentPos = 0.1;

            CapstoneHelper.ServoSpeed regulator = new CapstoneHelper.ServoSpeed(
                    this.helper,
                    buffer,
                    this.currentPos,
                    5,
                    AngularVelocityUnits.REV_MIN
            );

            TaskManager.startTask(regulator);

            while(!Thread.interrupted() && !regulator.isDone);

            TaskManager.stopTask(regulator.getTaskID());

            buffer = this.currentPos;
            this.currentPos = 0.275;

            regulator = new CapstoneHelper.ServoSpeed(
                    this.helper,
                    buffer,
                    this.currentPos,
                    5,
                    AngularVelocityUnits.REV_MIN
            );

            TaskManager.startTask(regulator);

            while(!Thread.interrupted() && !regulator.isDone);

            TaskManager.stopTask(regulator.getTaskID());

            return GeneralError.NO_ERROR;
        }

        public Error setDown(Task task) {
            Error error = super.validateCall(task);
            if(error != GeneralError.NO_ERROR)
                return error;

            this.currentPos = 0.35;
            helper.setPosition(currentPos);

            return GeneralError.NO_ERROR;
        }

        Servo helper;

        double currentPos;

        private class ServoSpeed extends TaskLoop{
            public ServoSpeed(Clock clock, Servo servo, double currentPos, double finalPos, double speed, AngularVelocityUnits units) {
                super(clock);

                TaskManager.startTask(clock);

                this.servo = servo;

                this.currentPos = currentPos;
                this.finalPos = finalPos;

                speed = units.toBase(speed * Math.signum(finalPos - currentPos));

                this.increment = speed * this.timeInterval / ArmDriver.maxServoAngle;

                this.numOfIterations = (int) ((finalPos - currentPos) / this.increment);

                this.clock = clock;
            }
            public ServoSpeed(Servo servo, double currentPos, double finalPos, double speed, AngularVelocityUnits units) {
                this(new Clock(10), servo, currentPos, finalPos, speed, units);
            }

            @Override
            protected Error destructor() {
                TaskManager.stopTask(this.clock.getTaskID());
                this.servo.setPosition(this.finalPos);

                return GeneralError.NO_ERROR;
            }

            @Override
            public Error loop() {
                if(iteration++ >= numOfIterations) {
                    super.interrupt();
                    this.isDone = true;
                }
                this.currentPos += this.increment;
                this.servo.setPosition(currentPos);

                return GeneralError. NO_ERROR;
            }

            private final Servo servo;

            private final double timeInterval = 0.1;

            private final int numOfIterations;
            private final double increment;

            private int iteration = 0;

            private double currentPos;
            private final double finalPos;

            private final Clock clock;

            public boolean isDone = false;
        }
    }

    private enum State {
        INTAKE, CAPSTONE, CAPSTONE_HOVER;
    }
    private enum HelperState {
        ON, OFF;
    }

    private State currentState;

    private GamePadDriver gamepad;
    private CapstoneHelper helper;
    private ArmDriver arm;
    private IntakeDriver intake;

    private GamePadDriver.MomentarySwitch intakeSwitch;

    private GamePadDriver.ToggleSwitch setHover;
    private GamePadDriver.MomentarySwitch activate;

    private GamePadDriver.ToggleSwitch helperOn;
    private GamePadDriver.MomentarySwitch helperIntake;

    private HelperState helperState = HelperState.OFF;
}
