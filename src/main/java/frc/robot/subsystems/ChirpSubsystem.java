package frc.robot.subsystems;

import java.util.HashMap;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.chirp;
import frc.robot.SystemManager;;

/** Orchestra™ */
public class ChirpSubsystem extends SubsystemBase {

    public enum SongType{
        onEnable,
        whileEnabled,
        onDisable,
    }

    private final class song{
        protected final String name;
        protected final SongType type;
        protected final Orchestra[] orchestras;
        public song(String name, SongType type, Orchestra[] orchestras){
            this.name=name;
            this.type=type;
            this.orchestras=orchestras;
        }

        public boolean play(){
            boolean success = true;
    
            for (int i = 0; i < orchestras.length; i++) {
                if (!orchestras[i].play().isOK()) {
                    System.out.println("Song '" + name + "', track " + i + " failed to play.");
                    success = false;
                }
            }
            return success;
        }

        public boolean stop(){
            boolean success = true;

            for (int i = 0; i < orchestras.length; i++) {
                if (!orchestras[i].stop().isOK()) {
                    System.out.println("Song '" + name + "', track " + i + " failed to stop.");
                    success = false;
                }
            }

            return success;
        }

        public boolean isPlaying(){
            return orchestras[0].isPlaying();
        }
    }

    private final HashMap<SongType, song> songs = new HashMap<>();


   
    protected TalonFX[] motors;


    protected song current=null;

    protected StringPublisher songString = NetworkTableInstance.getDefault().getStringTopic("audio/song").publish();
    protected DoubleArraySubscriber desiredChassisStatesSubscriber = NetworkTableInstance.getDefault().
        getDoubleArrayTopic("/SmartDashboard/swerve/desiredStates").subscribe(new double[]{0,0,0,0,0,0,0,0});
    
    protected DoubleArraySubscriber realChassisStatesSubscriber = NetworkTableInstance.getDefault().
        getDoubleArrayTopic("/SmartDashboard/swerve/measuredStates").subscribe(new double[]{0,0,0,0,0,0,0,0});
    
    protected boolean somethingWasPlaying=false;


    public ChirpSubsystem() {
        motors= SystemManager.swerve.getMotors();

        if (chirp.shouldUseEnableSong){
            loadSong(chirp.enableSongName, SongType.onEnable, chirp.enableTrackCount);
        }

        if (chirp.shouldUseDisableSong){
            loadSong(chirp.disableSongName, SongType.onDisable, chirp.disableTrackCount);
        }

        if (chirp.shouldUseWhileEnabledSong){
            loadSong(chirp.whileEnabledSongName, SongType.whileEnabled, chirp.whileEnabledTrackCount);
        }

        songString.set("Currently playing nothing");


    }

    public void enable(){
        if (chirp.shouldUseEnableSong){
            play(SongType.onEnable);
        }
        stop();
    }

    public void disable(){
        stop();
        if (chirp.shouldUseEnableSong){
            play(SongType.onDisable);
        }

    }

    @Override
    public void periodic(){

        SmartDashboard.putBoolean("swerveIsRunning", swerveIsMoving());
        
        if (isPlaying() && swerveIsMoving()){
            stop();
        }
        else if (!isPlaying() && chirp.shouldUseWhileEnabledSong && DriverStation.isEnabled()){
            play(SongType.whileEnabled);
        }
        if (isPlaying()!=somethingWasPlaying){
            if (isPlaying()) songString.set("Currently playing " + this.current.name + " as the " + this.current.type.toString());
            else songString.set("Currently playing nothing");
            somethingWasPlaying=isPlaying();
        }

    }


    public boolean isPlaying(){
        return current!=null&&current.isPlaying();
    }

    public boolean swerveIsMoving(){
        double[] desiredStates = desiredChassisStatesSubscriber.get();
        double[] realStates = realChassisStatesSubscriber.get();
        for (int i=0; i<8;i+=2){
            System.out.println(realStates[i]-desiredStates[i]);
            if (Math.abs(desiredStates[i+1])>0.02 || Math.abs(realStates[i]-desiredStates[i])>1) return true;
        }

        return false;


    }



    /**
     * Loads a song to be played later with {@link ChirpSubsystem#playSong(String)}.
     * <p>
     * Files for the song should be located at {@code deploy/chirp/<name>_<track>.chrp}
     *
     * @param name The name of the song to load.
     * @param trackCount The number of tracks the song has.
     * @param motors The motors that will be used to play the song. There should be at least as many motors as there are tracks.
     *               Motors will be distributed as evenly as possible between the tracks, filling all tracks first, then going
     *               back to add multiple motors to the same track (you can use nulls to skip over tracks when assigning motors).
     */
    public void loadSong(String name, SongType type, int trackCount) {

        if (motors.length < trackCount) {
            System.out.printf(
                "Not enough motors for song '%s' (Got: %d, Recommended: %d).%n",
                name,
                motors.length,
                trackCount
            );
        }

        Orchestra[] orchestras = new Orchestra[trackCount];

        // create a separate instance of Orchestra for each track because we can't
        // figure out how to put multiple tracks into one .chrp file :(
        for (int track = 0; track < trackCount; track++) {
            Orchestra orchestra = new Orchestra();
            boolean loadSucceeded=true;
            String path = chirp.path + name + "_" + track + ".chrp";
            if (!orchestra.loadMusic(path).isOK()) {
                loadSucceeded=false;
                System.err.printf(
                    "Failed to load song '%s', track %d. Make sure the file '%s' exists.%n",
                    path,
                    track,
                    path
                );
            }

            if (trackCount==1 && !loadSucceeded){
                path = chirp.path + name + ".chrp";

                if (!orchestra.loadMusic(path).isOK()) {
                    System.err.printf(
                        "Failed to load song '%s' with no track value. Make sure the file '%s' exists.%n",
                        path,
                        path
                    );
                }
            }


            for (int i = track; i < motors.length; i += trackCount) {
                TalonFX motor = motors[i];
                if (motor != null) orchestra.addInstrument(motor, 0);
            }


            orchestras[track] = orchestra;
        }


        songs.put(type, new song(name, type, orchestras));

        
    }

    /**
     * Play a song that was already loaded with {@link ChirpSubsystem#loadSong(String, int, CANTalonFX...)}.
     * @param type The type of the song to play. For example 
     * @return Whether the song was successfully played.
     */
    public boolean play(SongType type) {
        stop();

        song song = songs.get(type);

        if (song == null) {
          
            return false;
        }

        
        current=song;
        

        return song.play();
    }



    public boolean stop() {
        //if (!(current!=null&&current.isPlaying())) return false;
        if (current==null) return false;
        System.out.println("stop atempted");

        return current.stop();
    }
}