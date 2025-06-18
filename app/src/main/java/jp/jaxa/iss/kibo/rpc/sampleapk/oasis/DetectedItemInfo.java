package jp.jaxa.iss.kibo.rpc.sampleapk.oasis;

import java.util.List;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

public class DetectedItemInfo {
    public int areaIdx;
    public int orientationIdx;
    public Point point;
    public Quaternion orientation;
    public int arId;
    public double[] tvec;
    public double angle;
    public List<String> items;
    public String imageFilename;

    public DetectedItemInfo(int areaIdx, int orientationIdx, Point point, Quaternion orientation, int arId, double[] tvec, double angle, List<String> items, String imageFilename) {
        this.areaIdx = areaIdx;
        this.orientationIdx = orientationIdx;
        this.point = point;
        this.orientation = orientation;
        this.arId = arId;
        this.tvec = tvec;
        this.angle = angle;
        this.items = items;
        this.imageFilename = imageFilename;
    }
}
