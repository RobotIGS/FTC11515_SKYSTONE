package org.firstinspires.ftc.teamcode.Autonomous;

import android.hardware.Camera;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareMaps.HardwareChassis;
import org.firstinspires.ftc.teamcode.Library.Movement.ControlledDrive;
import org.firstinspires.ftc.teamcode.Library.VuforiaNavigator;

@TeleOp(name="TestFlashlight")
public class TestFlashlight extends LinearOpMode {
    private VuforiaNavigator vuforiaNavigator;
    private HardwareChassis robot;
    private ControlledDrive controlledDrive;

    @Override public void runOpMode() {
        this.robot = new HardwareChassis(hardwareMap);
        this.vuforiaNavigator = new VuforiaNavigator(hardwareMap, robot);
        this.controlledDrive= new ControlledDrive(hardwareMap);

        Camera cam = Camera.open();
        Camera.Parameters p = cam.getParameters();
        p.setFlashMode(Camera.Parameters.FLASH_MODE_TORCH);
        cam.setParameters(p);
        cam.startPreview();

        //drive method, speed forward, speed sideways
        //controlledDrive.driveConditionally(0.5, 0, () -> true);
        //TODO: method grabSkystone() --> void, skystone greifen
    }
}
