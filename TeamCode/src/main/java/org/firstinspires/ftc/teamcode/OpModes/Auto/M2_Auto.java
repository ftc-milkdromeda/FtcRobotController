package org.firstinspires.ftc.teamcode.OpModes.Auto;

import android.graphics.Paint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ControlSystem.ArmSystem;
import org.firstinspires.ftc.teamcode.ControlSystem.Camera;
import org.firstinspires.ftc.teamcode.ControlSystem.CapDetection;
import org.firstinspires.ftc.teamcode.ControlSystem.Carousel;
import org.firstinspires.ftc.teamcode.ControlSystem.DriveTrainAuto;
import org.firstinspires.ftc.teamcode.ControlSystem.DriverDirectory;
import org.firstinspires.ftc.teamcode.OpModes.TeleOp.TeleOpMode;

import Framework.Drivers.DriverManager;
import Framework.Drivers.DriverType;
import Framework.Error;
import Framework.Tasks.Task;
import Framework.Tasks.TaskManager;
import Framework.Units.AngleUnits;
import Framework.Units.LengthUnits;

@Autonomous(name="M2 Auto")
//@Disabled
public class M2_Auto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        TaskManager.stopTask();
        DriverManager manager = new DriverManager(DriverDirectory.LENGTH.getID());

        this.driveTrain = new DriveTrainAuto(hardwareMap);
        this.camera = new Camera();
        this.armSystem = new ArmSystem.ArmDriver(super.hardwareMap);
        this.carousel = new Carousel.CarouselDriver(super.hardwareMap);

        Servo servo = super.hardwareMap.servo.get("bucket");

        servo.setPosition(0);

        manager.addDriver(this.driveTrain);
        manager.addDriver(this.camera);
        manager.addDriver(this.armSystem);
        manager.addDriver(this.carousel);

        manager.initDrivers();

        TaskManager.bindDriverManager(manager);

        ExitDetector task = new ExitDetector();

        while (!super.isStarted()) {
            if (super.gamepad1.b)
                this.side = TeleOpMode.Side.RED;
            if (super.gamepad1.x)
                this.side = TeleOpMode.Side.BLUE;
            if (super.gamepad1.y)
                this.startPos = AutoStart.CAROUSEL;
            if (super.gamepad1.a)
                this.startPos = AutoStart.WAREHOUSE;
            /*
            if (super.gamepad1.dpad_up)
                this.capPosition = CapDetection.Position.POS3;
            if (super.gamepad1.dpad_left || super.gamepad1.dpad_right)
                this.capPosition = CapDetection.Position.POS2;
            if (super.gamepad1.dpad_down)
                this.capPosition = CapDetection.Position.POS1;
                */

            super.telemetry.addData("Side", this.side);
            super.telemetry.addData("Start", this.startPos);
            super.telemetry.update();
        }

        if (M2_Auto.this.side == TeleOpMode.Side.RED && M2_Auto.this.startPos == AutoStart.WAREHOUSE)
            this.capDetection = new CapDetection((CapDetection.Position pos) -> {this.capPosition = pos; this.capDetectionComplete = true;}, RedWarehouse);
        else if (M2_Auto.this.side == TeleOpMode.Side.RED && M2_Auto.this.startPos == AutoStart.CAROUSEL)
            this.capDetection = new CapDetection((CapDetection.Position pos) -> {this.capPosition = pos; this.capDetectionComplete = true;}, RedCarousel);
        else if (M2_Auto.this.side == TeleOpMode.Side.BLUE && M2_Auto.this.startPos == AutoStart.WAREHOUSE)
            this.capDetection = new CapDetection((CapDetection.Position pos) -> {this.capPosition = pos; this.capDetectionComplete = true;}, BlueWarehouse);
        else
            this.capDetection = new CapDetection((CapDetection.Position pos) -> {this.capPosition = pos; this.capDetectionComplete = true;}, BlueCarousel);

        super.waitForStart();

        TaskManager.startTask(capDetection);
        TaskManager.startTask(task);

        while (super.opModeIsActive()) {
            super.telemetry.addData("Num Of Tasks", TaskManager.getNumOfTask());
            super.telemetry.addData("Side", this.side);
            super.telemetry.addData("Start", this.startPos);
            super.telemetry.update();
            super.telemetry.update();
        };

        TaskManager.stopTask();
    }

    private class ExitDetector extends Task {
        public ExitDetector() {
            super(new TaskManager.DriverList(
                    new DriverType[]{DriverDirectory.DRIVE_TRAIN_AUTO, DriverDirectory.CAROUSEL, DriverDirectory.ARM}, null
            ));
        }

        @Override
        protected Error init() {
            this.driveTrain = (DriveTrainAuto) super.boundDrivers[0];
            this.carousel = (Carousel.CarouselDriver) super.boundDrivers[1];
            this.armSystem = (ArmSystem.ArmDriver) super.boundDrivers[2];
            return super.init();
        }

        @Override
        protected Error destructor() {
            return super.destructor();
        }

        @Override
        public void run() {
            if (M2_Auto.this.side == TeleOpMode.Side.RED && M2_Auto.this.startPos == AutoStart.WAREHOUSE) {
                //red warehouse
                this.driveTrain.auto_moveForward(0.5, -6, LengthUnits.IN);
                this.driveTrain.auto_strafe(0.5, 18.5, LengthUnits.IN);

                while(!M2_Auto.this.capDetectionComplete && !super.isInterrupted());
                if (capPosition == CapDetection.Position.POS3) {
                    //extend arm
                    this.armSystem.setHigh(this, true);
                    //move forward
                    this.driveTrain.auto_moveForward(0.25, -15, LengthUnits.IN);
                    //dump preload
                    this.armSystem.dump(this);
                    try {
                        Thread.sleep(1000);
                    } catch (InterruptedException e) {
                        super.interrupt();
                        return;
                    }
                    this.armSystem.resetDump(this);
                    //move back
                    this.driveTrain.auto_moveForward(0.5, 15, LengthUnits.IN);
                }
                else if (capPosition == CapDetection.Position.POS2) {
                    this.armSystem.setMid(this, true);
                    this.driveTrain.auto_moveForward(0.25, -6.5, LengthUnits.IN); //needs adjustment
                    this.armSystem.dump(this);
                    try {
                        Thread.sleep(1000);
                    } catch (InterruptedException e) {
                        super.interrupt();
                        return;
                    }
                    this.armSystem.resetDump(this);
                    this.driveTrain.auto_moveForward(0.5, 6.5, LengthUnits.IN);
                }
                else {//position 1
                    this.armSystem.setLow(this, true);
                    this.armSystem.hoverArm(this);
                    this.driveTrain.auto_moveForward(0.25, -5, LengthUnits.IN);
                    this.armSystem.setCapIntake(this);
                    try {
                        Thread.sleep(2000);
                    } catch (InterruptedException e) {
                        super.interrupt();
                        return;
                    }
                    this.armSystem.resetCapArm(this);
                    this.driveTrain.auto_moveForward(0.5, 5, LengthUnits.IN);
                }
                this.armSystem.setHome(this);
                this.driveTrain.auto_rotate(0.5, -90, AngleUnits.DEG);
                this.driveTrain.auto_strafe(0.5, 16, LengthUnits.IN); //14.5 away from the barrier, 1.5in extra travel
                this.driveTrain.auto_moveForward(0.5, 3.5 + 48, LengthUnits.IN); //3.5in to line up with the tile edge, and then move 2 tiles
            }
            else if (M2_Auto.this.side == TeleOpMode.Side.RED && M2_Auto.this.startPos == AutoStart.CAROUSEL) {
                this.carousel.setOn(this, M2_Auto.this.side);
                this.driveTrain.auto_moveForward(0.05, -5, LengthUnits.IN);

                try {
                    Thread.sleep(10000);
                }
                catch (InterruptedException e) {
                    super.interrupt();
                    return;
                }
                this.carousel.setOff(this);

                this.driveTrain.auto_strafe(0.25, -28, LengthUnits.IN);
                this.driveTrain.auto_moveForward(0.25, -5, LengthUnits.IN);
                /*
                //red carousel
                this.driveTrain.auto_moveForward(0.5, -4.5, LengthUnits.IN);
                this.driveTrain.auto_rotate(0.5, 180, AngleUnits.DEG);
                this.driveTrain.auto_moveForward(0.5, 4.5, LengthUnits.IN); //5in away from tile edge
                this.driveTrain.auto_strafe(0.5, -17, LengthUnits.IN); //extra 1.5in, need adjustment
                this.carousel.setOn(this, TeleOpMode.Side.RED);
                //sleeps for 2 seconds
                try {
                    Thread.sleep(2000);
                } catch (InterruptedException e) {
                    super.interrupt();
                    return;
                }
                this.carousel.setOff(this);
                this.driveTrain.auto_strafe(0.5, 46, LengthUnits.IN); //move to x pos of starting pos
                this.driveTrain.auto_moveForward(0.5, 4.5, LengthUnits.IN); //move to tile edge
                this.driveTrain.auto_rotate(0.5, 180, AngleUnits.DEG);

                while(!M2_Auto.this.capDetectionComplete && !super.isInterrupted());
                if (capPosition == CapDetection.Position.POS3) {
                    //extend arm
                    this.armSystem.setHigh(this, true);
                    //move forward
                    this.driveTrain.auto_moveForward(0.25, -15, LengthUnits.IN);
                    //dump preload
                    this.armSystem.dump(this);
                    try {
                        Thread.sleep(500);
                    } catch (InterruptedException e) {
                        super.interrupt();
                        return;
                    }
                    this.armSystem.resetDump(this);
                    //move back
                    this.driveTrain.auto_moveForward(0.5, 15, LengthUnits.IN);
                }
                else if (capPosition == CapDetection.Position.POS2) {
                    this.armSystem.setMid(this, true);
                    this.driveTrain.auto_moveForward(0.25, -6.5, LengthUnits.IN); //needs adjustment
                    this.armSystem.dump(this);
                    try {
                        Thread.sleep(500);
                    } catch (InterruptedException e) {
                        super.interrupt();
                        return;
                    }
                    this.armSystem.resetDump(this);
                    this.driveTrain.auto_moveForward(0.5, 6.5, LengthUnits.IN);
                }
                else {//position 1
                    this.armSystem.setLow(this, true);
                    this.armSystem.hoverArm(this);
                    this.driveTrain.auto_moveForward(0.25, -5, LengthUnits.IN);
                    this.armSystem.setCapIntake(this);
                    try {
                        Thread.sleep(2000);
                    } catch (InterruptedException e) {
                        super.interrupt();
                        return;
                    }
                    this.armSystem.resetCapArm(this);
                    this.driveTrain.auto_moveForward(0.5, 5, LengthUnits.IN);
                }

                this.armSystem.setHome(this);
                this.driveTrain.auto_strafe(0.75, 48, LengthUnits.IN);
                this.driveTrain.auto_moveForward(0.75, -18, LengthUnits.IN); //questionable calculations, might be completely wrong
                 */
                super.interrupt();
            }
            else if (M2_Auto.this.side == TeleOpMode.Side.BLUE && M2_Auto.this.startPos == AutoStart.WAREHOUSE) {
                //blue warehouse
                this.driveTrain.auto_moveForward(0.5, -6, LengthUnits.IN);
                this.driveTrain.auto_strafe(0.5, -18.5, LengthUnits.IN);

                while(!M2_Auto.this.capDetectionComplete && !super.isInterrupted());
                if (capPosition == CapDetection.Position.POS3) {
                    //extend arm
                    this.armSystem.setHigh(this, true);
                    //move forward
                    this.driveTrain.auto_moveForward(0.25, -15, LengthUnits.IN);
                    //dump preload
                    this.armSystem.dump(this);
                    try {
                        Thread.sleep(1000);
                    } catch (InterruptedException e) {
                        super.interrupt();
                        return;
                    }
                    this.armSystem.resetDump(this);
                    //move back
                    this.driveTrain.auto_moveForward(0.5, 15, LengthUnits.IN);
                }
                else if (capPosition == CapDetection.Position.POS2) {
                    this.armSystem.setMid(this, true);
                    this.driveTrain.auto_moveForward(0.25, -6.5, LengthUnits.IN); //needs adjustment
                    this.armSystem.dump(this);
                    try {
                        Thread.sleep(1000);
                    } catch (InterruptedException e) {
                        super.interrupt();
                        return;
                    }
                    this.armSystem.resetDump(this);
                    this.driveTrain.auto_moveForward(0.5, 6.5, LengthUnits.IN);
                }
                else {//position 1
                    this.armSystem.setLow(this, true);
                    this.driveTrain.auto_moveForward(0.25, -5, LengthUnits.IN);
                    this.armSystem.dumpLow(this);
                    try {
                        Thread.sleep(1000);
                    } catch (InterruptedException e) {
                        super.interrupt();
                        return;
                    }
                    this.armSystem.dumpLowReset(this);
                    this.driveTrain.auto_moveForward(0.5, 5, LengthUnits.IN);
                }
                this.armSystem.setHome(this);
                this.driveTrain.auto_rotate(0.5, 90, AngleUnits.DEG);
                this.driveTrain.auto_strafe(0.5, -16, LengthUnits.IN); //14.5 away from the barrier, 1.5in extra travel
                this.driveTrain.auto_moveForward(0.5, 3.5 + 48, LengthUnits.IN); //3.5in to line up with the tile edge, and then move 2 tiles
                super.interrupt();
            }
            else if (M2_Auto.this.side == TeleOpMode.Side.BLUE && M2_Auto.this.startPos == AutoStart.CAROUSEL) {
                this.carousel.setOn(this, M2_Auto.this.side);
                this.driveTrain.auto_moveForward(0.05, -5, LengthUnits.IN);

                try {
                    Thread.sleep(10000);
                }
                catch (InterruptedException e) {
                    super.interrupt();
                    return;
                }
                this.carousel.setOff(this);

                this.driveTrain.auto_strafe(0.25, 28, LengthUnits.IN);
                this.driveTrain.auto_moveForward(0.25, -5, LengthUnits.IN);
                //blue carousel
                /*
                this.driveTrain.auto_moveForward(0.5, -5.5, LengthUnits.IN);
                this.driveTrain.auto_rotate(0.5, 180, AngleUnits.DEG);
                this.driveTrain.auto_moveForward(0.5, 4.5, LengthUnits.IN); //5in away from tile edge
                this.driveTrain.auto_strafe(0.5, 17, LengthUnits.IN); //extra 1.5in, need adjustment
                this.carousel.setOn(this, TeleOpMode.Side.BLUE);
                //sleep for 2 seconds
                try {
                    Thread.sleep(2000);
                } catch (InterruptedException e) {
                    super.interrupt();
                    return;
                }
                this.carousel.setOff(this);
                this.driveTrain.auto_strafe(0.5, -46, LengthUnits.IN); //move to x pos of starting pos
                this.driveTrain.auto_moveForward(0.5, 4.5, LengthUnits.IN); //move to tile edge
                this.driveTrain.auto_rotate(0.5, 180, AngleUnits.DEG);

                while(!M2_Auto.this.capDetectionComplete && !super.isInterrupted());

                if (capPosition == CapDetection.Position.POS3) {
                    //extend arm
                    this.armSystem.setHigh(this, true);
                    //move forward
                    this.driveTrain.auto_moveForward(0.25, -15, LengthUnits.IN);
                    //dump preload
                    this.armSystem.dump(this);
                    try {
                        Thread.sleep(500);
                    } catch (InterruptedException e) {
                        super.interrupt();
                        return;
                    }
                    this.armSystem.resetDump(this);
                    //move back
                    this.driveTrain.auto_moveForward(0.5, 15, LengthUnits.IN);
                }
                else if (capPosition == CapDetection.Position.POS2) {
                    this.armSystem.setMid(this, true);
                    this.driveTrain.auto_moveForward(0.25, -6.5, LengthUnits.IN); //needs adjustment
                    this.armSystem.dump(this);
                    try {
                        Thread.sleep(500);
                    } catch (InterruptedException e) {
                        super.interrupt();
                        return;
                    }
                    this.armSystem.resetDump(this);
                    this.driveTrain.auto_moveForward(0.5, 6.5, LengthUnits.IN);
                }
                else {//position 1
                    this.armSystem.setLow(this, true);
                    this.armSystem.hoverArm(this);
                    this.driveTrain.auto_moveForward(0.25, -5, LengthUnits.IN);
                    this.armSystem.setCapIntake(this);
                    try {
                        Thread.sleep(2000);
                    } catch (InterruptedException e) {
                        super.interrupt();
                        return;
                    }
                    this.armSystem.resetCapArm(this);
                    this.driveTrain.auto_moveForward(0.5, 5, LengthUnits.IN);
                }

                this.armSystem.setHome(this);
                this.driveTrain.auto_strafe(0.5, -48, LengthUnits.IN);
                this.driveTrain.auto_moveForward(0.5, -18, LengthUnits.IN); //questionable calculations, might be completely wrong

                 */
                super.interrupt();
            }
        }

        DriveTrainAuto driveTrain;
        Carousel.CarouselDriver carousel;
        ArmSystem.ArmDriver armSystem;
        Camera camera;
    }

    public enum AutoStart {WAREHOUSE, CAROUSEL}

    DriveTrainAuto driveTrain;
    Carousel.CarouselDriver carousel;
    ArmSystem.ArmDriver armSystem;
    Camera camera;
    CapDetection capDetection;

    TeleOpMode.Side side = TeleOpMode.Side.RED;
    AutoStart startPos = AutoStart.WAREHOUSE;

    CapDetection.Position capPosition = CapDetection.Position.POS3;
    boolean capDetectionComplete = true;

    CapDetection.CapConfiguration RedWarehouse = new CapDetection.CapConfiguration(
            new CapDetection.Coordinate(529,728,720,890, 30337),
            new CapDetection.Coordinate(529,728,720,890, 30906),
            new CapDetection.Coordinate(0, 0, 1, 1, 10000),
            CapDetection.Position.POS3
    );

    CapDetection.CapConfiguration RedCarousel = new CapDetection.CapConfiguration(
            new CapDetection.Coordinate(32,727,276,882, 29029),
            new CapDetection.Coordinate(510,721,720,882, 33776),
            new CapDetection.Coordinate(0, 0, 1, 1, 1000),
            CapDetection.Position.POS3
    );

    CapDetection.CapConfiguration BlueWarehouse = new CapDetection.CapConfiguration(
            new CapDetection.Coordinate(0, 0, 1, 1, 1000),
            new CapDetection.Coordinate(0,723,237,890, 38483),
            new CapDetection.Coordinate(659,732,720,884, 8716),
            CapDetection.Position.POS1
    );

    CapDetection.CapConfiguration BlueCarousel = new CapDetection.CapConfiguration(
            new CapDetection.Coordinate(0, 0, 1, 1, 10000),
            new CapDetection.Coordinate(06,730,720,887,17720),
            new CapDetection.Coordinate(06,730,720,887,17720),
            CapDetection.Position.POS1
    );
}