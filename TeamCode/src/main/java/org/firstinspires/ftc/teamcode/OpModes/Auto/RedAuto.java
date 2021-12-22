package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ControlSystem.DriveTrainAuto;
import org.firstinspires.ftc.teamcode.ControlSystem.DriverDirectory;
import org.firstinspires.ftc.teamcode.OpModes.TeleOp.TeleOpMode;

import Framework.Drivers.DriverManager;
import Framework.Drivers.DriverType;
import Framework.Error;
import Framework.Tasks.Task;
import Framework.Tasks.TaskManager;
import Framework.Units.LengthUnits;

@Autonomous(name="Red")
@Disabled
public class RedAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DriverManager manager = new DriverManager(DriverDirectory.LENGTH.getID());

        this.driveTrain = new DriveTrainAuto(hardwareMap);

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

    TeleOpMode.Side side = TeleOpMode.Side.RED;

    DriveTrainAuto driveTrain;
}
