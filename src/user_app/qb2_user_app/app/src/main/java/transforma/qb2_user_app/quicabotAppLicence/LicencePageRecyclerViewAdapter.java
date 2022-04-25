package transforma.qb2_user_app.quicabotAppLicence;

import android.content.Context;
import android.content.Intent;
import android.net.Uri;
import android.os.Build;
import androidx.annotation.NonNull;
import androidx.recyclerview.widget.RecyclerView;
import android.text.Layout;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.TextView;
import java.util.List;
import transforma.qb2_user_app.R;

/**
 * This is the recyclerview adapter class for the licence page
 *
 * It binds the customised cardView to the recyclerView by passing in the licenceDate to the constructor
 * No amendments to be made to this class if just simply adding more licence card to the page
 */

public class LicencePageRecyclerViewAdapter extends RecyclerView.Adapter<LicencePageRecyclerViewAdapter.MyViewHolder> {

    private Context myContext;
    private List<AppLicenceUsed> myLicenceData;

    LicencePageRecyclerViewAdapter(Context myContext, List<AppLicenceUsed> myLicenceData) {
        this.myContext = myContext;
        this.myLicenceData = myLicenceData;
    }

    @NonNull
    @Override
    public MyViewHolder onCreateViewHolder(@NonNull ViewGroup viewGroup, int i) {

        LayoutInflater myInflater = LayoutInflater.from(myContext);
        View view = myInflater.inflate(R.layout.activity_quicabot_licence_page_recyclerview_card, viewGroup,false);
        return new MyViewHolder(view);
    }

    @Override
    public void onBindViewHolder(@NonNull MyViewHolder myViewHolder, int position) {

        myViewHolder.licenceTitle.setText(myLicenceData.get(position).getTitle());
        myViewHolder.licenceContent.setText(myLicenceData.get(position).getContent());
        myViewHolder.licenceType.setText(myLicenceData.get(position).getType());
        myViewHolder.licenceHomepage.setOnClickListener((view)->{ //An intent visiting the home page
            Intent intent = new Intent(Intent.ACTION_VIEW);
            intent.setData(Uri.parse(myLicenceData.get(position).getHomepage()));
            myContext.startActivity(intent);
        });
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) { // Allow text justification at certain Android Version
            myViewHolder.licenceContent.setJustificationMode(Layout.JUSTIFICATION_MODE_INTER_WORD);
        }


    }

    @Override
    public int getItemCount() {
        return myLicenceData.size();
    }

    static class MyViewHolder extends RecyclerView.ViewHolder{

        TextView licenceTitle, licenceContent, licenceType;
        Button licenceHomepage;
        MyViewHolder(View itemView){
            super(itemView);
            licenceTitle = itemView.findViewById(R.id.licenceTitle);
            licenceContent =itemView.findViewById(R.id.licenceContent);
            licenceType =itemView.findViewById(R.id.licenceType);
            licenceHomepage =itemView.findViewById(R.id.licenceHomepage);

        }

    }
}
