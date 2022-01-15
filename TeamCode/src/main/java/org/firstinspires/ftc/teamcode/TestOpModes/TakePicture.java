package org.firstinspires.ftc.teamcode.TestOpModes;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.ControlSystem.Camera;
import org.firstinspires.ftc.teamcode.ControlSystem.DriverDirectory;

import Framework.Drivers.DriverManager;
import Framework.Drivers.DriverType;
import Framework.Images.Image;
import Framework.Tasks.Task;
import Framework.Tasks.TaskManager;

public class TakePicture extends OpMode {
    @Override
    public void init() {
        this.manager = new DriverManager(DriverDirectory.LENGTH.getID());

        this.camera = new Camera();

        this.manager.addDriver(this.camera);
        this.manager.initDrivers();

        this.dummyTask = new Task(new TaskManager.DriverList(new DriverType[] {DriverDirectory.CAMERA}, null)) {
            @Override
            public void run() {}
        };

        TaskManager.startTask(dummyTask);
    }

    @Override
    public void loop() {
        if(super.gamepad1.a) {
            Image image = this.camera.takeImage(dummyTask);
            image.saveImage("TakePicture-image" + number++);
        }
    }

    Camera camera;
    Task dummyTask;

    DriverManager manager = new DriverManager(DriverDirectory.LENGTH.getID());

    public static int number = 0;
}
