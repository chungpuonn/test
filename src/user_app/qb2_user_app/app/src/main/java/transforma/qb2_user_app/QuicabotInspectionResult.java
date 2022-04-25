package transforma.qb2_user_app;

import androidx.appcompat.app.AppCompatActivity;
import android.os.Bundle;

import com.bin.david.form.core.SmartTable;
import com.bin.david.form.core.TableConfig;
import com.bin.david.form.data.column.Column;
import com.bin.david.form.data.style.FontStyle;
import com.bin.david.form.data.table.TableData;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import ROS.RosMsgRaw;
import transforma.qb2_user_app.rosmsg.InspectionResult;
import transforma.qb2_user_app.rosmsg.InspectionResultFormal;


/**
 * This is the Activity Class for displaying inpection summary. It uses SmartTable library to display everything
 */

//TODO: A lot to be implemented
public class QuicabotInspectionResult extends QuicabotAppCompatActivity {

    SmartTable<InspectionResult> smartTable;
    TableConfig config;
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_quicabot_inspection_result);

        smartTable = findViewById(R.id.resultTable);
        config = smartTable.getConfig();
        final Column<Integer> finishingColumnFloor = new Column<>("Finishing", "finishingFloor");
        final Column<Integer> finishingColumnWall = new Column<>("Finishing", "finishingWall");
        final Column<Integer> finishingColumnCeiling = new Column<>("Finishing", "finishingCeiling");
        final Column<Integer> alignColumnFloor = new Column<>("Alignment", "alignmentFloor");
        final Column<Integer> alignColumnWall = new Column<>("Alignment", "alignmentWall");
        final Column<Integer> alignColumnCeiling = new Column<>("Alignment", "alignmentCeiling");
        final Column<Integer> crackColumnFloor = new Column<>("Crack&Dmg", "crackAndDmgFloor");
        final Column<Integer> crackColumnWall = new Column<>("Crack&Dmg", "crackAndDmgWall");
        final Column<Integer> crackColumnCeiling = new Column<>("Crack&Dmg", "crackAndDmgCeiling");


        final Column floor = new Column("Floor",finishingColumnFloor,alignColumnFloor,crackColumnFloor);
        final Column wall = new Column("Wall",finishingColumnWall,alignColumnWall, crackColumnWall);
        final Column ceiling = new Column("Ceiling",finishingColumnCeiling, alignColumnCeiling,crackColumnCeiling);
        List<InspectionResultFormal> list = new ArrayList<>();
        for (int i = 0; i < 5; i++) {
            Random random = new Random();
            InspectionResultFormal resultFormal = new InspectionResultFormal(random.nextInt(),random.nextInt(),random.nextInt(),random.nextInt(),random.nextInt(),random.nextInt(),random.nextInt(),random.nextInt(),random.nextInt());
            list.add(resultFormal);
        }



        TableData tableData = new TableData<>("Internal Finishes",list, floor, wall, ceiling);

        config.setShowTableTitle(false);
        //config.setColumnTitleBackground(new BaseBackgroundFormat(R.color.colorSecondaryAccent));
        config.setTableTitleStyle(new FontStyle(15, R.color.colorSecondary)) ;
        //config.setMinTableWidth(ViewGroup.LayoutParams.MATCH_PARENT);
        smartTable.setTableData(tableData);
        }

    /**
     * Method is called when RosBridge receives a message
     *
     * @param message
     * @param msgRaw
     */
    @Override
    public void onMessage(String message, RosMsgRaw msgRaw) {

    }
}
