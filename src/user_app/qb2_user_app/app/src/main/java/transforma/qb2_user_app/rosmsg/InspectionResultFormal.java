package transforma.qb2_user_app.rosmsg;


/**
 * TODO: PRIMAL CLASS
 * This the data class for storing detailed inspection information
 */

public class InspectionResultFormal {


    private int finishingFloor;
    private int finishingWall;
    private int finishingCeiling;
    private int alignmentFloor;
    private int alignmentWall;
    private int alignmentCeiling;
    private int crackAndDmgFloor;
    private int crackAndDmgWall;
    private int crackAndDmgCeiling;


    public InspectionResultFormal(int finishingFloor, int finishingWall, int finishingCeiling, int alignmentFloor, int alignmentWall, int alignmentCeiling, int crackAndDmgFloor, int crackAndDmgWall, int crackAndDmgCeiling) {
        this.finishingFloor = finishingFloor;
        this.finishingWall = finishingWall;
        this.finishingCeiling = finishingCeiling;
        this.alignmentFloor = alignmentFloor;
        this.alignmentWall = alignmentWall;
        this.alignmentCeiling = alignmentCeiling;
        this.crackAndDmgFloor = crackAndDmgFloor;
        this.crackAndDmgWall = crackAndDmgWall;
        this.crackAndDmgCeiling = crackAndDmgCeiling;
    }
}
