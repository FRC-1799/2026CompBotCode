package frc.robot.subsystems.vision.calibration;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.vision.LimelightHelpers;
import frc.robot.subsystems.vision.calibration.spec.RobotSpec;

import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.util.*;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.subsystems.vision.calibration.MathCal.*;

public class CameraCal {


    private final RobotSpec _robotSpec;
    private final List<Camera> _cameras;
    private final List<FiduciaryTag> _tags;

    public Pose3d RobotPose = new Pose3d();

    private StructArrayPublisher<Pose3d> calTagPublisher;
    private StructArrayPublisher<Pose3d> calSeenTagPublisher;
    private StructPublisher<Pose3d> calTagRobotPublisher;
    private StructArrayPublisher<Pose3d> calCameraTestPublisher;
    private StructArrayPublisher<Pose3d> calCameraPredictedPosition;
    private boolean publishSetupDone = false;

    private ObjectMapper mapper;

    private boolean _isSimulating = false;

    public CameraCal(RobotSpec robotSpec, List<Camera> cameras, List<FiduciaryTag> tags) {
        if ((tags.size() & 1) == 0)
            throw new IllegalArgumentException("number of ids must be odd so there is center tag");

        _robotSpec = robotSpec;
        _cameras = cameras;
        _tags = tags;

        _isSimulating = _cameras.stream().anyMatch(c -> c.testPositionOnRobot != Pose3d.kZero);
    }

    public CameraCal(RobotSpec robotSpec, List<Camera> cameras, List<Integer> ids, double distance, double spacing, double height) {
        this(robotSpec, cameras, generateTags(ids, distance, spacing, height));
    }

    public void SetDefaultCameraPositions() {
        _cameras.stream().forEach(c -> LimelightHelpers.setCameraPose_RobotSpace(
                c.id,
                0f,
                0f,
                0f,
                0f,
                0f,
                0f
        ));
    }

    public void PublishData() {
        if (!publishSetupDone) {
            PublishDataSetup();
            publishSetupDone = true;
        }

        // pub robot
        calTagRobotPublisher.set(RobotPose);

        // pub tags
        var tags = _tags.stream().map(tag -> {
                    var t = RelativeToPose(tag.position, RobotPose);

                    // flip it 180 degrees around its Z
                    // must be a better way to that
                    t = new Pose3d(
                            t.getTranslation(),
                            new Rotation3d(
                                    t.getRotation()
                                            .getQuaternion()
                                            .times(
                                                    Quaternion.
                                                            fromRotationVector(
                                                                    VecBuilder
                                                                            .fill(0, 0, Math.PI)
                                                            )
                                            )
                            )
                    );

                    return t;
                }
        ).toArray(Pose3d[]::new);
        calTagPublisher.set(tags);

        // pub camera test positions
        if (_isSimulating) {
            var cams = _cameras.stream().map(cam -> {
                        return RelativeToPose(cam.testPositionOnRobot, RobotPose);
                    }
            ).toArray(Pose3d[]::new);
            calCameraTestPublisher.set(cams);
        }

        // pub seen fids
        var fids = GetLimelightCalData();
        var seenTags = fids.stream().map(cam -> {
                    return cam.fidList.stream().filter(f -> f.fiducialID == 35).map(fid -> {
                        var pose = Camera2RobotSpace(fid.getTargetPose_CameraSpace());
                        //reverse tag
                        pose = LocalPlus(Degrees.of(180), Degrees.of(180), Degrees.of(0), pose);
                        return pose;
                    }).collect(Collectors.toList());
                }
        ).flatMap(List::stream).toArray(Pose3d[]::new);
        calSeenTagPublisher.set(seenTags);

        // predicted camera positions
        var cameraPositions = fids.stream().map(cam -> {
                    return cam.fidList.stream().filter(f -> f.fiducialID == 35).map(fid -> {
                        var pose = Camera2RobotSpace(fid.getTargetPose_CameraSpace());
                        var cameraToTarget = new Transform3d(pose.getTranslation(), pose.getRotation());
                        var tagPose = _tags.stream().filter(t -> t.id == fid.fiducialID).findFirst();

                        if(!tagPose.isEmpty()) {
                            var camPose = tagPose.get().position
                                    .transformBy(cameraToTarget.inverse());
                            return Optional.of(camPose);
                        }

                        return Optional.empty();
                    })
                            .filter(Optional::isPresent)
                            .map(Optional::get)
                            .collect(Collectors.toList());
                }
        ).flatMap(List::stream).toArray(Pose3d[]::new);
        calCameraPredictedPosition.set(cameraPositions);


    }

    private void PublishDataSetup() {
        calTagPublisher = NetworkTableInstance.getDefault()
                .getStructArrayTopic("SmartDashboard/Calibrate/Tags", Pose3d.struct).publish();

        calTagRobotPublisher = NetworkTableInstance.getDefault()
                .getStructTopic("SmartDashboard/Calibrate/Robot", Pose3d.struct).publish();

        if (_isSimulating) {
            calCameraTestPublisher = NetworkTableInstance.getDefault()
                    .getStructArrayTopic("SmartDashboard/Calibrate/CamerasTest", Pose3d.struct).publish();
        }

        calSeenTagPublisher = NetworkTableInstance.getDefault()
                .getStructArrayTopic("SmartDashboard/Calibrate/TagsSeen", Pose3d.struct).publish();

        calCameraPredictedPosition = NetworkTableInstance.getDefault()
                .getStructArrayTopic("SmartDashboard/Calibrate/CameraPositionPredict", Pose3d.struct).publish();
    }

    protected static List<FiduciaryTag> generateTags(List<Integer> ids, double distance, double spacing, double height) {
        if ((ids.size() & 1) == 0)
            throw new IllegalArgumentException("number of ids must be odd so there is center tag");

        final List<Pose3d> _relativeTags;
        // only works if odd
        var outsideCount = (ids.size() - 1) / 2;

        AtomicInteger tagIndex = new AtomicInteger();
        return IntStream.range(outsideCount * -1, outsideCount + 1)
                .mapToObj(i -> {
                    return new FiduciaryTag(
                            ids.get(tagIndex.getAndIncrement()),
                            new Pose3d(
                                    distance,
                                    i * spacing,
                                    height,
                                    new Rotation3d()
                            )
                    );
                }).toList();
    }

    public List<CameraCalData> GetLimelightCalData() {
        return _isSimulating ? getSimLimelightCallData() : getRealLimelightCalData();
    }

    private List<CameraCalData> getRealLimelightCalData() {
        return _cameras.stream().map(c -> {
            var fidList = GetLatestFiducials(c.id);
            return new CameraCalData(c.id, fidList);
        }).toList();
    }

    private List<CameraCalData> getSimLimelightCallData() {
        return _cameras.stream().map(cam -> {
            // find tags we can see
            var tags = FindSeenTags(cam);

            var fidList = LimelightHelpers.getRawFiducials(cam.id);
            return new CameraCalData(cam.id, new List<LimelightHelpers.LimelightTarget_Fiducial>() {
                @Override
                public int size() {
                    return 0;
                }

                @Override
                public boolean isEmpty() {
                    return false;
                }

                @Override
                public boolean contains(Object o) {
                    return false;
                }

                @Override
                public Iterator<LimelightHelpers.LimelightTarget_Fiducial> iterator() {
                    return null;
                }

                @Override
                public Object[] toArray() {
                    return new Object[0];
                }

                @Override
                public <T> T[] toArray(T[] a) {
                    return null;
                }

                @Override
                public boolean add(LimelightHelpers.LimelightTarget_Fiducial limelightTargetFiducial) {
                    return false;
                }

                @Override
                public boolean remove(Object o) {
                    return false;
                }

                @Override
                public boolean containsAll(Collection<?> c) {
                    return false;
                }

                @Override
                public boolean addAll(Collection<? extends LimelightHelpers.LimelightTarget_Fiducial> c) {
                    return false;
                }

                @Override
                public boolean addAll(int index, Collection<? extends LimelightHelpers.LimelightTarget_Fiducial> c) {
                    return false;
                }

                @Override
                public boolean removeAll(Collection<?> c) {
                    return false;
                }

                @Override
                public boolean retainAll(Collection<?> c) {
                    return false;
                }

                @Override
                public void clear() {

                }

                @Override
                public frc.robot.subsystems.vision.LimelightHelpers.LimelightTarget_Fiducial get(int index) {
                    return null;
                }

                @Override
                public frc.robot.subsystems.vision.LimelightHelpers.LimelightTarget_Fiducial set(int index, LimelightHelpers.LimelightTarget_Fiducial element) {
                    return null;
                }

                @Override
                public void add(int index, LimelightHelpers.LimelightTarget_Fiducial element) {

                }

                @Override
                public frc.robot.subsystems.vision.LimelightHelpers.LimelightTarget_Fiducial remove(int index) {
                    return null;
                }

                @Override
                public int indexOf(Object o) {
                    return 0;
                }

                @Override
                public int lastIndexOf(Object o) {
                    return 0;
                }

                @Override
                public ListIterator<LimelightHelpers.LimelightTarget_Fiducial> listIterator() {
                    return null;
                }

                @Override
                public ListIterator<LimelightHelpers.LimelightTarget_Fiducial> listIterator(int index) {
                    return null;
                }

                @Override
                public List<LimelightHelpers.LimelightTarget_Fiducial> subList(int fromIndex, int toIndex) {
                    return List.of();
                }
            });
        }).toList();
    }

    public List<FiduciaryTag> FindSeenTags(Camera camera) {
        var tags = _tags.stream().map(t -> {
            var tagPosition = t.position;
            var tag = new FiduciaryTag(t.id, tagPosition);
            tag.cameraAngleTo = LookAtRotation(RobotPose.getTranslation(), tagPosition.getTranslation(), VecBuilder.fill(0, 0, 1));

            var x = tag.getXAngle();
            var y = tag.getYAngle();
            var z = tag.getZAngle();

            return tag;
        }).toList();

        return tags;
    }

    public List<LimelightHelpers.LimelightTarget_Fiducial> GetLatestFiducials(String id) {
        var results = getLatestResults(id);

        return Arrays.stream(results.targets_Fiducials).toList();
    }

    public String httpRequest(URI uri) {
        try {
            HttpClient client = HttpClient.newHttpClient();
            HttpRequest request = HttpRequest.newBuilder()
                    .uri(uri)
                    .build();

            HttpResponse<String> response =
                    client.send(request, HttpResponse.BodyHandlers.ofString());

            return response.body();
        } catch (Exception e) {
            DataLogManager.log(e.toString());
        }

        return "";
    }

    public URI createLimelightRequestUri(String name, String path) {
        var uri = URI.create(SmartDashboard.getString(name + "_Interface", ""));
        if (!Objects.equals(uri.toString(), "")) {
            return URI.create("http://" + uri.getHost() + ":5807/" + path);
        }
        return URI.create("");
    }

    public frc.robot.subsystems.vision.LimelightHelpers.LimelightResults getLatestResults(String limelightName) {

        long start = System.nanoTime();
        LimelightHelpers.LimelightResults results = new LimelightHelpers.LimelightResults();
        if (mapper == null) {
            mapper = new ObjectMapper().configure(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES, false);
        }

        try {
            var llUri = createLimelightRequestUri(limelightName, "results");
            if (!Objects.equals(llUri.toString(), "")) {
                var jsonString = httpRequest(llUri);

                if (jsonString == null || jsonString.isEmpty() || jsonString.isBlank()) {
                    results.error = "lljson error: empty json";
                } else {
                    results = mapper.readValue(jsonString, LimelightHelpers.LimelightResults.class);
                    if (results.imuResults != null) {
                        results.imuResults.parseDataArray();
                    }
                }
            }
        } catch (JsonProcessingException e) {
            results.error = "lljson error: " + e.getMessage();
        }

        long end = System.nanoTime();
        double millis = (end - start) * .000001;
        results.latency_jsonParse = millis;

        return results;
    }

//    public LimelightCalData FindCameraPosition(String limelightId) {
//        var fids = LimelightHelpers.getRawFiducials(limelightId);
//
//        if(f.length == 6) {
//            var camTranslate = new Translation3d(f[2], f[0] * -1, f[1]);
//            var camRot = new Rotation3d(Degrees.of(f[5] * -1), Degrees.of(f[3]), Degrees.of(f[4] * -1));
//
//            var botTranslate = new Translation3d(1,1,1);
//
//            var fidPost = botTranslate.minus(camTranslate);
//
//            var pose = new Pose3d(fidPost, camRot);
//            SmartDashboard.putNumberArray("Limelight-tpp-raw", f);
//        }
//
//    }
}

