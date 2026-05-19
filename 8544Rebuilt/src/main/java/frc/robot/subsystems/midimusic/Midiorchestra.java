package frc.robot.subsystems.midimusic;
import  frc.robot.subsystems.shooter.*;
import  frc.robot.subsystems.drive.*;
import com.ctre.phoenix6.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Midiorchestra extends SubsystemBase {
    

Orchestra m_orchestra = new Orchestra();
public Midiorchestra() {
// Add devices to the orchestra
}

public Midiorchestra(Shooter shooter, Drive drive) {
shooter.addInstruments(m_orchestra);
m_orchestra.loadMusic("output-smb-flag.chrp");

}

public void playmusic() {
//status =
m_orchestra.play();


}
public void stopmusic() {
//status =
m_orchestra.stop();


}
}