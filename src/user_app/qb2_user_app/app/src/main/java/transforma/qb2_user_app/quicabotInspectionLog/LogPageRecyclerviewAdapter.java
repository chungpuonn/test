package transforma.qb2_user_app.quicabotInspectionLog;

import androidx.annotation.NonNull;
import androidx.cardview.widget.CardView;
import androidx.recyclerview.widget.RecyclerView;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.CheckBox;
import android.widget.TextView;

import java.util.ArrayList;
import transforma.qb2_user_app.R;

//TODO: Optimise file manager

/**
 * This is the recyclerview adapter class for the logging page
 *
 * It binds the customised cardView to the recyclerView by passing in the logging data to the constructor
 * No amendments to be made to this class unless the format of the Inspection Log Class is changed
 * Logging is automatically done by the app
 */

public class LogPageRecyclerviewAdapter extends RecyclerView.Adapter<LogPageRecyclerviewAdapter.MyViewHolder> {

    private QuicabotInspectionLogPage context;
    private ArrayList<InspectionLog> inspectionLogList;
    private ArrayList<InspectionLog> selectedLogList = new ArrayList<>();

    LogPageRecyclerviewAdapter(QuicabotInspectionLogPage context, ArrayList<InspectionLog> logList){
        this.context = context;
        inspectionLogList = logList;
    }

    @NonNull
    @Override
    public MyViewHolder onCreateViewHolder(@NonNull ViewGroup viewGroup, int i) {
        View view;
        LayoutInflater myInflater = LayoutInflater.from(context);
        view = myInflater.inflate(R.layout.activity_quicabot_inspection_log_page_recyclerview_card, viewGroup,false);
        return new MyViewHolder(view);

    }

    @Override
    public void onBindViewHolder(@NonNull MyViewHolder myViewHolder, int position) {
        myViewHolder.logTitle.setText(inspectionLogList.get(position).getStartDateTime());
        myViewHolder.logCard.setOnClickListener((view) ->{
            context.showDetails(inspectionLogList.get(position));
        });

        //TODO: This selection if left for future development for data selection and filtering
//        myViewHolder.selctionBox.setOnClickListener((view) ->{
//            myViewHolder.selctionBox.setSelected(!myViewHolder.selctionBox.isSelected());
//            if (myViewHolder.selctionBox.isSelected() && !selectedLogList.contains(inspectionLogList.get(position))){
//                selectedLogList.add(inspectionLogList.get(position));
//            } else  {
//                try {selectedLogList.remove(inspectionLogList.get(position));}
//                catch (Exception e) {e.printStackTrace();}
//            }
//        });
    }


    @Override
    public int getItemCount() {
        return inspectionLogList.size();
    }

    static class MyViewHolder extends RecyclerView.ViewHolder{

        TextView logTitle;
        CardView logCard;
        CheckBox selctionBox;

        MyViewHolder(View itemView){
            super(itemView);
            logTitle = itemView.findViewById(R.id.logTitle);
            selctionBox =itemView.findViewById(R.id.logSelcetionCheckBox);
            logCard = itemView.findViewById(R.id.inspectionLogCard);

        }

    }
}
