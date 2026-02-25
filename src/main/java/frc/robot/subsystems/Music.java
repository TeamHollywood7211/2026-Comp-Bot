
package frc.robot.subsystems;

import com.ctre.phoenix6.Orchestra;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Music extends SubsystemBase {
    private final Orchestra orchestra;

    public Music(Swerve swerve) {
        orchestra = new Orchestra();

        // The Tuner X SwerveDrivetrain provides access to modules by index (0-3)
        // We add both the Drive and Steer motors from each module for more 'voices'
        for (int i = 0; i < 4; i++) {
            orchestra.addInstrument(swerve.getModule(i).getDriveMotor());
            orchestra.addInstrument(swerve.getModule(i).getSteerMotor());
        }
    }

    public void playSong(String fileName) {
        String path = Filesystem.getDeployDirectory().getAbsolutePath() + "/" + fileName;
        
        if (orchestra.isPlaying()) {
            orchestra.stop();
        }
        
        // loadMusic returns a StatusCode; you can check this for errors
        orchestra.loadMusic(path);
        orchestra.play();
    }

    public void stop() {
        orchestra.stop();
    }
}