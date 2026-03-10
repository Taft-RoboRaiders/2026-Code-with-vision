package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CameraSubsystem extends SubsystemBase
{

  private final UsbCamera camera;

  public CameraSubsystem()
  {
    if (RobotBase.isSimulation())
    {
      // In simulation, start a dummy camera feed
      camera = CameraServer.startAutomaticCapture();
      // Don't set properties that don't exist in sim
    } else
    {
      // On real robot, initialize camera properly
      camera = CameraServer.startAutomaticCapture(0);

      // Wrap property settings in try/catch to avoid crashing if something fails
      try
      {
        camera.setResolution(320, 240);
        camera.setFPS(20);
        camera.setBrightness(50);
        camera.setExposureAuto();
      } catch (Exception e)
      {
        System.out.println("Camera property setup failed: " + e.getMessage());
      }
    }
  }

  @Override
  public void periodic()
  {
    // Nothing needed for now
  }
}
