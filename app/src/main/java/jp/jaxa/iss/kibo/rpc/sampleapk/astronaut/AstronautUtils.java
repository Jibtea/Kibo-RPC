package jp.jaxa.iss.kibo.rpc.sampleapk.astronaut;
import jp.jaxa.iss.kibo.rpc.sampleapk.vision.MarkerRotationAligner;
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

        // ถ่ายภาพจากนักบินอวกาด
        Mat image = api.getMatNavCam();
        api.saveMatImage(image, "astronaut_recognition_image.png");

        //=============เอาโค้ดrecognitionไม่ก้ออะไรก็ได้มาใส่ตรงนี้เพื่อดูว่านักบินอวกาศต้องการอะไร=================
        
        
        // ==================================================

        if (!result.isEmpty()) {
            api.reportMessage("Target item detected!");
            api.reportMissionCompletion(); // หรือ method อื่นที่ต้องเรียก
        } else {
            api.reportMessage("No target item found.");
        }
        // Notify the API that recognition is complete
        api.notifyRecognitionItem();

        // อาจต้องมีreturnค่าที่ตรวจได้ออกว่าชีต้องการอะไร ไม่ก็เอาค่าลงmodelอื่นๆเพื่อดึงค่าไปใช้
    }


    public static void moveToTargetItemAndSnapshot(KiboRpcApi api) {
        // TODO: Implement logic to move to the target item
        //เอาค่าตะกี้มาเทียบว่าวัตถุทีชีจะหาอยู่พิกัดไหนที่เราเคยไป
        //อันนี้แค่มอคต้องมีแก้อีกเยอะ
        List<Point> CheckedAreas = api.getCheckedAreas(); // Get previously checked areas
        for (int i = 0; i < CheckedAreas.size(); i++) {
            Point area = CheckedAreas.get(i);
            // เช็คว่าareaที่เราเก็บมีitemตรงไหม
            if (area.equals(targetItemLocation)) {
                api.moveTo(area, new Quaternion(0f, 0f, 0.707f, 0.707f), false);
                //เพิ่มเงื่อนไขขยับหุ่นให้ตรงมุมและระยะห่างที่เหมาะสมจะได้ได้คะแนนดีๆ 
                // functionที่ทำให้หุ่นขยับไปตรงมุมยังไม่ทำระยะห่าง
                MarkerRotationAligner aligner = new MarkerRotationAligner(api, Aruco.DICT_4X4_50);
                aligner.rotateUntilMarkerTopRight();
                api.reportRoundingCompletion();
                break;
            }
        }

        api.takeTargetItemSnapshot();
    }
}
