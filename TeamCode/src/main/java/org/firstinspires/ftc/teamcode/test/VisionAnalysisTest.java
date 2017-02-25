package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.detection.objects.Rectangle;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.VisionOpMode;
import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.opencv.core.Point;
import org.opencv.core.Size;

/**
 * Created by 292486 on 2/22/2017.
 */

@Autonomous(name="Vision Analysis", group="vision")
public class VisionAnalysisTest extends VisionOpMode {

    @Override
    public void init(){
        super.init();

        telemetry.addData("Super inited", "");
        telemetry.update();

        this.setCamera(Cameras.SECONDARY);  //Front-facing camera
        this.setFrameSize(new Size(900, 900));  //Frame to analyze (bigger = slower but more accurate)

        telemetry.addData("this'd", "");
        telemetry.update();

        enableExtension(Extensions.BEACON);
        enableExtension(Extensions.ROTATION);   //Corrects screen rotation
        //enableExtension(Extensions.CAMERA_CONTROL);

        telemetry.addData("Extensions", "");
        telemetry.update();

        beacon.setAnalysisMethod(Beacon.AnalysisMethod.FAST);   //Complex, default, realtime
        //beacon.setAnalysisBounds(new Rectangle(new Point(width/2, height/2), width-200, 200));  //Cropping. Use if we can approximate location

        telemetry.addData("analysis method", "");
        telemetry.update();

        beacon.setColorToleranceBlue(0);  // -1 ~ 1 (0 default)
        beacon.setColorToleranceRed(0);   // -1 ~ 1 (0 default)

        telemetry.addData("tolerances", "");
        telemetry.update();

        rotation.setIsUsingSecondaryCamera(true);
        rotation.disableAutoRotate();
        rotation.setActivityOrientationFixed(ScreenOrientation.PORTRAIT);

        telemetry.addData("rotation", "");
        telemetry.update();

        cameraControl.setColorTemperature(CameraControlExtension.ColorTemperature.AUTO);    //Can set white balance values for better
        cameraControl.setAutoExposureCompensation();

        telemetry.addData("Temperatured", "");
        telemetry.update();
    }

    @Override
    public void loop(){
        super.loop();

        telemetry.addData("Center: ", beacon.getAnalysis().getCenter());
        telemetry.addData("Color: ", beacon.getAnalysis().getColorString());
        telemetry.addData("Confidence", beacon.getAnalysis().getConfidenceString());
        telemetry.addData("Buttons", beacon.getAnalysis().getButtonString());
        telemetry.addData("Frame Size", "Width: " + width + " Height: " + height);
        telemetry.update();
    }

}
