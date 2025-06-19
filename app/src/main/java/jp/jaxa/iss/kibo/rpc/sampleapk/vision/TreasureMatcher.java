package jp.jaxa.iss.kibo.rpc.sampleapk.vision;

public class TreasureMatcher {

    public static String matchTreasure(int treasureNumber) {
        switch (treasureNumber) {
            case 1: return "coin";
            case 2: return "compass";
            case 3: return "coral";
            case 4: return "crystal";
            case 5: return "diamond";
            case 6: return "emerald";
            case 7: return "fossil";
            case 8: return "key";
            case 9: return "letter";
            case 10: return "shell";
            case 11: return "treasure_box";
            default: return "unknown";
        }
    }

    /**
     * How to use:
     * String treasureName = TreasureMatcher.matchTreasure(3); // คืน "coral"
     */
}
