// เพิ่มคลาสเล็ก ๆ สำหรับผลลัพธ์
public class MarkerDetectionResult {
    public final List<Mat> corners;
    public final Mat ids;

    public MarkerDetectionResult(List<Mat> corners, Mat ids) {
        this.corners = corners;
        this.ids = ids;
    }
}
