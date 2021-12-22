package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ControlSystem.ArmSystem;
import org.firstinspires.ftc.teamcode.ControlSystem.Carousel;
import org.firstinspires.ftc.teamcode.ControlSystem.DriveTrainTeleOp;
import org.firstinspires.ftc.teamcode.ControlSystem.DriverDirectory;
import org.firstinspires.ftc.teamcode.ControlSystem.GamePadDriver;

import Framework.Drivers.DriverManager;
import Framework.Error;
import Framework.Tasks.Clock;
import Framework.Tasks.TaskManager;

@TeleOp(name="Teleop")
public class TeleOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        TaskManager.stopTask();

        this.side = Side.RED;
        TeleOpMode.output = this.telemetry;

        DriveTrainTeleOp.DriveTrain driveTrain = new DriveTrainTeleOp.DriveTrain(super.hardwareMap);

        GamePadDriver gamePad = new GamePadDriver(super.gamepad1, super.gamepad2);

        Carousel.CarouselDriver carouselDriver = new Carousel.CarouselDriver(super.hardwareMap);

        ArmSystem.ArmDriver arm = new ArmSystem.ArmDriver(super.hardwareMap);
        ArmSystem.IntakeDriver intake = new ArmSystem.IntakeDriver(super.hardwareMap);
        ArmSystem.CapstoneHelper capstoneHelper = new ArmSystem.CapstoneHelper(super.hardwareMap);

        DriverManager manager = new DriverManager(DriverDirectory.LENGTH.getID());

        manager.addDriver(gamePad);
        manager.addDriver(driveTrain);
        manager.addDriver(carouselDriver);

        manager.addDriver(arm);
        manager.addDriver(intake);
        manager.addDriver(capstoneHelper);

        manager.initDrivers();

        TaskManager.bindDriverManager(manager);

        Clock clock = new Clock(60);

        DriveTrainTeleOp driveTask = new DriveTrainTeleOp(clock);
        Carousel carousel = new Carousel(clock, this.side);
        ArmSystem armSystem = new ArmSystem(clock);

        Error clockError = TaskManager.startTask(clock);

        super.waitForStart();

        Error driveError = TaskManager.startTask(driveTask);
        Error carouselError = TaskManager.startTask(carousel);

        TaskManager.startTask(armSystem);

        while(super.opModeIsActive()){
            if(gamePad.getX(GamePadDriver.PadID.GAMEPAD2)) {
                this.side = Side.BLUE;
                carousel.setSide(Side.BLUE);
            }
            if(gamePad.getB(GamePadDriver.PadID.GAMEPAD2)) {
                this.side = Side.RED;
                carousel.setSide(Side.RED);
            }

            super.telemetry.addData("Task #", TaskManager.getNumOfTask());
            super.telemetry.addData("Side", this.side.toString());
            super.telemetry.update();
        }

        TaskManager.stopTask();
        manager.terminateDriver();
    }
    public enum Side {
        BLUE(-1),
        RED(1);

        Side(int value) {
            this.value = value;
        }

        public int getMultiplier() {
            return this.value;
        }

        int value;
    }

    protected Side side;

    public static Telemetry output;

}
