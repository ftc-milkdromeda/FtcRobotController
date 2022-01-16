package org.firstinspires.ftc.teamcode.TestOpModes;


import android.graphics.Paint;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ControlSystem.Camera;
import org.firstinspires.ftc.teamcode.ControlSystem.CapDetection;
import org.firstinspires.ftc.teamcode.ControlSystem.DriverDirectory;
import org.firstinspires.ftc.teamcode.OpModes.Auto.M2_Auto;
import org.firstinspires.ftc.teamcode.OpModes.TeleOp.TeleOpMode;

import Framework.Drivers.DriverManager;
import Framework.Drivers.DriverType;
import Framework.Images.Image;
import Framework.Tasks.Task;
import Framework.Tasks.TaskManager;

@TeleOp(name = "CapTest")
public class CapTest extends OpMode {
    @Override
    public void init() {
        TaskManager.stopTask();

        this.manager = new DriverManager(DriverDirectory.LENGTH.getID());

        this.camera = new Camera();

        this.manager.addDriver(this.camera);
        this.manager.initDrivers();

        TaskManager.bindDriverManager(manager);
    }

    @Override
    public void loop() {
        if(super.gamepad1.right_bumper) {
            if (this.side == TeleOpMode.Side.RED && this.startPos == M2_Auto.AutoStart.WAREHOUSE)
                this.cap = new CapDetection((CapDetection.Position pos) -> {this.pos = pos; this.isDone = true;}, RedWarehouse);
            else if (this.side == TeleOpMode.Side.RED && this.startPos == M2_Auto.AutoStart.CAROUSEL)
                this.cap = new CapDetection((CapDetection.Position pos) -> {this.pos = pos; this.isDone = true;}, RedCarousel);
            else if (this.side == TeleOpMode.Side.BLUE && this.startPos == M2_Auto.AutoStart.WAREHOUSE)
                this.cap = new CapDetection((CapDetection.Position pos) -> {this.pos = pos; this.isDone = true;}, BlueWarehouse);
            else
                this.cap = new CapDetection((CapDetection.Position pos) -> {this.pos = pos; this.isDone = true;}, BlueCarousel);

            TaskManager.startTask(this.cap);
        }
        if (super.gamepad1.b)
            this.side = TeleOpMode.Side.RED;
        if (super.gamepad1.x)
            this.side = TeleOpMode.Side.BLUE;
        if (super.gamepad1.y)
            this.startPos = M2_Auto.AutoStart.CAROUSEL;
        if (super.gamepad1.a)
            this.startPos = M2_Auto.AutoStart.WAREHOUSE;

        super.telemetry.addData("Pos", this.pos);
        super.telemetry.addData("Is Done", this.isDone);
        super.telemetry.addData("Side", this.side);
        super.telemetry.addData("Start", this.startPos);
        super.telemetry.update();
    }

    Camera camera;
    CapDetection cap;

    boolean isDone = false;
    CapDetection.Position pos = CapDetection.Position.NO_POS;

    DriverManager manager = new DriverManager(DriverDirectory.LENGTH.getID());

    TeleOpMode.Side side = TeleOpMode.Side.RED;
    M2_Auto.AutoStart startPos = M2_Auto.AutoStart.WAREHOUSE;

    public static int number = 0;
    private String path = "/storage/self/primary/Pictures/";

    CapDetection.CapConfiguration RedWarehouse = new CapDetection.CapConfiguration(
            new CapDetection.Coordinate(529,728,720,890, 30906),
            new CapDetection.Coordinate(32,727,276,882, 29029),
            new CapDetection.Coordinate(0, 0, 1, 1, 10000),
            CapDetection.Position.POS3
    );

    CapDetection.CapConfiguration RedCarousel = new CapDetection.CapConfiguration(
            new CapDetection.Coordinate(510,721,720,882, 33776),
            new CapDetection.Coordinate(0,723,237,890, 38483),
            new CapDetection.Coordinate(0, 0, 1, 1, 1000),
            CapDetection.Position.POS3
    );

    CapDetection.CapConfiguration BlueWarehouse = new CapDetection.CapConfiguration(
            new CapDetection.Coordinate(0, 0, 1, 1, 1000),
            new CapDetection.Coordinate(659,732,720,884, 8716),
            new CapDetection.Coordinate(169,734,408,893, 32529),
            CapDetection.Position.POS1
    );

    CapDetection.CapConfiguration BlueCarousel = new CapDetection.CapConfiguration(
            new CapDetection.Coordinate(0, 0, 1, 1, 10000),
            new CapDetection.Coordinate(606,730,720,887,17720),
            new CapDetection.Coordinate(160,730,399,888, 37576),
            CapDetection.Position.POS1
    );
}
