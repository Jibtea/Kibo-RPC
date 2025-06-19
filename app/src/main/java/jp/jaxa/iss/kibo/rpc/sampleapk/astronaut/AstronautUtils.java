package jp.jaxa.iss.kibo.rpc.sampleapk.astronaut;

import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcApi;

/**
 * Utility class for astronaut-related actions.
 */
public class AstronautUtils {

    private static Point targetItemPoint;
    private static Quaternion targetItemQuaternion;
    public static void moveToAstronautAndReport(KiboRpcApi api) {
        Point point = new Point(11.143d, -6.7607d, 4.9654d);
        Quaternion quaternion = new Quaternion(0f, 0f, 0.707f, 0.707f);
        api.moveTo(point, quaternion, false);
        api.reportRoundingCompletion();
    }
    public static void recognizeAndReportTargetItem(KiboRpcApi api,List<DetectedItemInfo> DetectedAreas) {
        // TODO: Implement target item recognition logic here
        boolean isTargetItemFound = false;
        //Capture the image of the target Area

        //call func to recognize target item
        DetectedItemInfo targetAreaItemInfo = api.recognizeTargetItem();
        if (targetAreaItemInfo != null&& !isTargetItemFound) {
            //เทียบว่าในข้อมูลที่มีอันไหนตรงบ้าง
            for(DetectedItemInfo Area : DetectedAreas){
                for(AreaItem : Area.items){
                    if(AreaItem.equals(targetAreaItemInfo.items.get(0))) {
                        targetItemPoint = Area.point;
                        targetItemQuaternion = Area.orientation;
                        targetAreaItemInfo.arId = Area.arId; // Set the AR ID from the recognized area
                        targetAreaItemInfo.imageFilename = Area.imageFilename; // Set the image filename

                        isTargetItemFound = true;
                        break; // Exit loop once we find a match
                    }
                }
            }

            moveToTargetItemAndSnapshot(api);
            api.reportTargetItemRecognition(targetAreaItemInfo);
        } else {
            // Handle case where no item is recognized
            api.reportNoTargetItemRecognized();
        }
        
        api.notifyRecognitionItem();
    }
    public static void moveToTargetItemAndSnapshot(KiboRpcApi api) {
        // TODO: Implement logic to move to the target item
        api.moveTo(targetItemPoint, targetItemQuaternion, true);
        api.takeTargetItemSnapshot();
    }
}
