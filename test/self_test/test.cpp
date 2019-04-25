#include <fstream>
#include <sstream>
#include <iostream>
#include <stdint.h>
#include <string>
#include <vector>

using namespace std;


struct TrackedFeature
{
  int id = -1;
  double x = -1.0;
  double y = -1.0;
//   double ts = -1;
};

//read three lines from the output_result, i.e. one spatiotemporal window - or one frame.
istream &read_frame(istream &fin, vector<TrackedFeature> &feature_list, double &timestamp)
{
    TrackedFeature temp_Feature;
    string line;
    // cout<<"Start reading frame (three lines)"<<endl;

    for(int line_counter=0;line_counter<3;line_counter++)
    {
        getline(fin, line);
        istringstream iss(line);
        if(line_counter%3 == 0)//id row
        {
            iss>>timestamp;
            while(iss>>temp_Feature.id)
            {
                // iss>>temp_Feature.id;
                feature_list.push_back(temp_Feature);
            }
        }
        else if(line_counter%3 == 1)//x row
        {
            double temp = -1;
            double temp_x = -1;
            int position = 0;
            iss>>temp;
            if(temp!=timestamp)
            {
                cout<<"ts="<<temp<<", timestamp is wrong!"<<endl;
                // return 0;
            }
            while(iss>>temp_x)
            {
                // iss>>temp_x;
                feature_list.at(position).x = temp_x;
                position++;
            }
        }
        else if(line_counter%3 == 2)//y row
        {
            double temp = -1;
            double temp_y = -1;
            int position = 0;
            iss>>temp;
            if(temp!=timestamp)
            {
                cout<<"ts="<<temp<<", timestamp is wrong!"<<endl;
                // return 0;
            }
            while(iss>>temp_y)
            {
                // iss>>temp_y;
                feature_list.at(position).y = temp_y;
                position++;
            }
        }
    }
    
    return fin;
}

/*
 *
 *  main()
 * 
 */
int main(int argc, char const *argv[])
{
    ifstream fin("/home/albert/workSpace/data/output_result_day_long2.txt");
    vector<TrackedFeature> feature_list;
    double timestamp = -1.0;
    while(read_frame(fin, feature_list, timestamp))
    {
        cout<<"\nResult "<<":\n";
        // for(auto ft:feature_list)
        // {
        //     if(ft.id==0)
        //     {
        //         //invalid feature
        //     }
        //     cout<<timestamp<<"  "<<ft.id<<"  "<<ft.x<<"  "<<ft.y<<endl;
        // }
        for(auto iter=feature_list.begin(); iter!=feature_list.end(); iter++)
        {
            if((*iter).id == 0)
            {
                iter = feature_list.erase(iter);
                iter--;//erase删除后会返回下一个iter
                continue;
            }
            cout<<timestamp<<"  "<<(*iter).id<<"  "<<(*iter).x<<"  "<<(*iter).y<<endl;
        }
        timestamp = -1.0;
        vector<TrackedFeature>().swap(feature_list);
    }
    
    return 0;
}


