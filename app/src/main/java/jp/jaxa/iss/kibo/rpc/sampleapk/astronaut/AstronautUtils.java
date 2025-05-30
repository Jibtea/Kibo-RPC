package jp.jaxa.iss.kibo.rpc.sampleapk.astronaut;

import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcApi;

/**
 * Utility class for astronaut-related actions.
 */
public class AstronautUtils {
    public static void moveToAstronautAndReport(KiboRpcApi api) {
        Point point = new Point(11.143d, -6.7607d, 4.9654d);
        Quaternion quaternion = new Quaternion(0f, 0f, 0.707f, 0.707f);
        api.moveTo(point, quaternion, false);
        api.reportRoundingCompletion();
    }
    public static void recognizeAndReportTargetItem(KiboRpcApi api) {
        // TODO: Implement target item recognition logic here
        api.notifyRecognitionItem();
    }
    public static void moveToTargetItemAndSnapshot(KiboRpcApi api) {
        // TODO: Implement logic to move to the target item
        api.takeTargetItemSnapshot();
    }
}
