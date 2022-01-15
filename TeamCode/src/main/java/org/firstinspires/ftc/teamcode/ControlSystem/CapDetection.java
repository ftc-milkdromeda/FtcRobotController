package org.firstinspires.ftc.teamcode.ControlSystem;

import com.sun.tools.javac.jvm.Gen;

import Framework.Drivers.DriverType;
import Framework.Error;
import Framework.GeneralError;
import Framework.Images.Image;
import Framework.Images.Pixel;
import Framework.Tasks.Task;
import Framework.Tasks.TaskManager;

public class CapDetection extends Task {
    public CapDetection(CallBack callBack, CapConfiguration config) {
        super(new TaskManager.DriverList(
                new DriverType[] {DriverDirectory.CAMERA},
                null
        ));

        this.callBack = callBack;
        this.config = config;
    }

    @Override
    protected Error init() {
        this.camera = (Camera) super.boundDrivers[0];

        return GeneralError.NO_ERROR;
    }

    @Override
    public void run() {
        Image[] imageSet = new Image[3];
        imageSet[0] = this.camera.takeImage(this);

        Error error = imageSet[0].toMono();
        if(error != GeneralError.NO_ERROR) {
            this.callBack.call(Position.NO_POS);
            super.setExitState(error);
            return;
        }

        error = imageSet[0].saveImage("CapDetection-0");
        if(error != GeneralError.NO_ERROR) {
            this.callBack.call(Position.NO_POS);
            super.setExitState(error);
            return;
        }

        for(int i = imageSet.length - 1; i >= 0; i--) {
            imageSet[i] = new Image(imageSet[0]);

            error = imageSet[i].cropImage(
                    this.config.getPos(i).x0,
                    this.config.getPos(i).y0,
                    this.config.getPos(i).x1,
                    this.config.getPos(i).y1
            );
            if(error != GeneralError.NO_ERROR) {
                this.callBack.call(Position.NO_POS);
                super.setExitState(error);
                return;
            }

            Pixel brightest = imageSet[i].findBrightest();
            Pixel dimmest = imageSet[i].findDimmest();

            error = imageSet[i].curveAdjAll(
                    (double x) -> this.colorCutoffFactor * (brightest.getLuminance() - dimmest.getLuminance()) + dimmest.getLuminance() > x ? 0 : 1
            );
            if(error != GeneralError.NO_ERROR) {
                this.callBack.call(Position.NO_POS);
                super.setExitState(error);
                return;
            }

            error = imageSet[i].saveImage("CapDetection-" + i);
            if(error != GeneralError.NO_ERROR) {
                this.callBack.call(Position.NO_POS);
                super.setExitState(error);
                return;
            }
        }

        int[] imagePixelCount = {
                this.countBlackPixel(imageSet[0]),
                this.countBlackPixel(imageSet[1]),
                this.countBlackPixel(imageSet[2])
        };

        Position pos = Position.POS1;

        for(int i = 1; i < imagePixelCount.length; i++)
            pos = imagePixelCount[pos.getValue()] < imagePixelCount[i] ? Position.getPos(i) : pos;

        this.callBack.call(imagePixelCount[pos.getValue()] > this.countCutoff ? pos : Position.NO_POS);
        this.setExitState(GeneralError.NO_ERROR);
    }

    private int countBlackPixel(Image image) {
        int count = 0;

        for(int i = 0; i < image.getWidth(); i++) {
            for(int j = 0; j < image.getHeight(); j++)
                count = image.getPixel(i, j).getLuminance() == 0 ? count + 1: count;
        }

        return count;
    }

    private CallBack callBack;
    private CapConfiguration config;

    private Camera camera;

    private final double colorCutoffFactor  = 0.5;
    private final int countCutoff = 100;

    public static class CapConfiguration {
        public CapConfiguration(Coordinate pos1, Coordinate pos2, Coordinate pos3) {
            this.pos1 = pos1;
            this.pos2 = pos2;
            this.pos3 = pos3;
        }

        public Coordinate getPos(int index) {
            switch (index) {
                case 1: return this.pos1;
                case 2: return this.pos1;
                case 3: return this.pos1;
                default: return null;
            }
        }

        Coordinate pos1;
        Coordinate pos2;
        Coordinate pos3;
    }
    public static class Coordinate {
        public Coordinate(int x, int y, int xOff, int yOff) {
            this.x0  = x;
            this.y0  = y;

            this.x1  = x + xOff;
            this.y1  = y + yOff;
        }

        public final int x0;
        public final int y0;

        public final int x1;
        public final int y1;

    }

    public interface CallBack {
        void call(Position pos);
    }

    public enum Position {
        NO_POS(-1),
        POS1(0),
        POS2(1),
        POS3(2);

        Position(int value) {
            this.value = value;
        }

        public int getValue() {
            return this.value;
        }

        public static Position getPos(int index) {
            switch (index) {
                case 0: return POS1;
                case 1: return POS2;
                case 2: return POS3;
                default: return NO_POS;
            }
        }

        private final int value;
    }
}
