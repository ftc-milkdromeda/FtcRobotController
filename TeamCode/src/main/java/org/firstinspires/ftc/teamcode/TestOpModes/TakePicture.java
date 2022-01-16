package org.firstinspires.ftc.teamcode.TestOpModes;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ControlSystem.Camera;
import org.firstinspires.ftc.teamcode.ControlSystem.DriverDirectory;

import Framework.Drivers.DriverManager;
import Framework.Drivers.DriverType;
import Framework.Images.Image;
import Framework.Tasks.Task;
import Framework.Tasks.TaskManager;

@TeleOp(name = "Task Picture")
public class TakePicture extends OpMode {
    @Override
    public void init() {
        TaskManager.stopTask();

        this.manager = new DriverManager(DriverDirectory.LENGTH.getID());

        this.camera = new Camera();

        this.manager.addDriver(this.camera);
        this.manager.initDrivers();

        TaskManager.bindDriverManager(manager);

        this.dummyTask = new Task(new TaskManager.DriverList(new DriverType[] {DriverDirectory.CAMERA}, null)) {
            @Override
            public void run() {}
        };

        super.telemetry.addData("Start Error", TaskManager.startTask(dummyTask).toString());
        super.telemetry.update();
    }

    @Override
    public void loop() {
        if(super.gamepad1.a) {
            Image image = this.camera.takeImage(dummyTask);
            image.saveImage(path + "TakePicture-image" + number++);
        }

        super.telemetry.addData("Num of Task", "%d", TaskManager.getNumOfTask());
        super.telemetry.update();
    }

    Camera camera;
    Task dummyTask;

    DriverManager manager = new DriverManager(DriverDirectory.LENGTH.getID());

    public static int number = 0;
    private String path = "/storage/self/primary/Pictures/";
}
