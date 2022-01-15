package org.firstinspires.ftc.teamcode.ControlSystem;

import Framework.Drivers.Driver;
import Framework.Drivers.DriverType;
import Framework.Error;
import Framework.GeneralError;
import Framework.Images.RGBPixel;
import Framework.Tasks.Task;

import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.R;

import java.nio.ByteBuffer;

public class Camera extends Driver {
    public Camera() {
        super(DriverDirectory.CAMERA);

        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.vuforiaLicenseKey = Camera.key;
        params.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;

        ClassFactory factory = ClassFactory.getInstance();
        this.locale = factory.createVuforia(params);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB888, true);
        locale.setFrameQueueCapacity(1);
    }

    public Framework.Images.Image takeImage(Task task) {
        if(super.validateCall(task) != GeneralError.NO_ERROR)
            return null;

        VuforiaLocalizer.CloseableFrame frame;
        try {
            frame = this.locale.getFrameQueue().take();
        }
        catch (InterruptedException interrupt) {
            return null;
        }

        Image raw = null;
        for(int a = 0; a < frame.getNumImages(); a++) {
            if (frame.getImage(a).getFormat() == PIXEL_FORMAT.RGB888) {
                raw = frame.getImage(a);
                break;
            }
        }

        if(raw == null)
            return null;

        ByteBuffer byteArray = raw.getPixels();
        byteArray.rewind();
        byte array[] = new byte[byteArray.remaining()];
        byteArray.get(array);

        RGBPixel[][] pixelArray = new RGBPixel[raw.getHeight()][raw.getWidth()];
        for(int i = 0; i < pixelArray.length; i++) {
            for(int j = 0; j < pixelArray[i].length; j++)
                pixelArray[i][j] = new RGBPixel(
                        array[(i * pixelArray[i].length + j) * 3 + 0] / 255f,
                        array[(i * pixelArray[i].length + j) * 3 + 1] / 255f,
                        array[(i * pixelArray[i].length + j) * 3 + 2] / 255f
                        );
        }

        return  new Framework.Images.Image(pixelArray);
    }

    private VuforiaLocalizer locale;
    private static final String key = "Ac7O/sT/////AAABmfegDDc3nUE0pRyH393dmoYsL9SH/fuutgc8rqUBYUUvEzt3vPN4UjwVcrmWAy30T8nEa8zEvDsuA03b6QLWoxRURps/uuPOUI9xeKZP17fSjCU8EjpOTEPbMEyKTY0uiR10gAlmD8lSkDBIEbKDLAWVYC9VRzPDllyxA38m8sXXZjDPfIJ4IPe1Ae+hW2x4pxCGY3qrdoM/t/41ZBYXXs/QnGpymvC6Rwqqmqu3K+xisVGxrqeJ9Fj+Ew0etzUJFpUFYXqZDPocJSLvdK1wM5fh6fqOFqdeNAM21X4ccqMGRMraIbEbwWWiabzywi3L5IsyeWU4DvuXk7dV6BKngVkj+wXcBJ2fn+2qdCNzuS7M";
}
