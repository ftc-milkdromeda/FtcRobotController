package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ControlSystem.DriveTrainAuto;
import org.firstinspires.ftc.teamcode.ControlSystem.DriveTrainTeleOp;
import org.firstinspires.ftc.teamcode.ControlSystem.DriverDirectory;
import org.firstinspires.ftc.teamcode.OpModes.TeleOp.TeleOpMode;

import Framework.Drivers.DriverManager;
import Framework.Drivers.DriverType;
import Framework.Error;
import Framework.GeneralError;
import Framework.Tasks.Clock;
import Framework.Tasks.Task;
import Framework.Tasks.TaskLoop;
import Framework.Tasks.TaskManager;
import Framework.Units.LengthUnits;

@Autonomous(name="Blue")
public class BlueAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DriverManager manager = new DriverManager(DriverDirectory.LENGTH.getID());

        this.driveTrain = new DriveTrainAuto(hardwareMap);
        Servo servo = super.hardwareMap.servo.get("bucket");

        servo.setPosition(0);

        manager.addDriver(this.driveTrain);
        manager.initDrivers();

        TaskManager.bindDriverManager(manager);

        ExitDetector task = new ExitDetector();

        super.waitForStart();

        TaskManager.startTask(task);

        while(super.opModeIsActive());

        TaskManager.stopTask();
    }

    private class ExitDetector extends Task {
        public ExitDetector() {
            super(new TaskManager.DriverList(
                    new DriverType[] {DriverDirectory.DRIVE_TRAIN_AUTO}, null
            ));
        }

        @Override
        protected Error init() {
            this.driveTrain = (DriveTrainAuto) super.boundDrivers[0];
            return super.init();
        }

        @Override
        protected Error destructor() {
            return super.destructor();
        }

        @Override
        public void run() {
            this.driveTrain.auto_moveForward(0.5, 3, LengthUnits.FEET);
        }

        DriveTrainAuto driveTrain;
    }

    TeleOpMode.Side side = TeleOpMode.Side.BLUE;

    DriveTrainAuto driveTrain;
}
