#include <fstream>
#include <sstream>
#include <iostream>
#include <stdint.h>
#include <string>
#include <vector>

using namespace std;

struct Event
{
    uint16_t x;
    uint16_t y;
    bool polarity;
    uint64_t timestamp;
};

struct TrackedFeature
{
  int id = -1;
  double x = -1.0;
  double y = -1.0;
//   double ts = -1;
};

istream &read_event(istream &is, Event &new_event)
{
    double tmp_timestamp;
    is >> tmp_timestamp >> new_event.x >> new_event.y >> new_event.polarity;
    new_event.timestamp = uint64_t (tmp_timestamp*1e9);
    // new_event.x = new_event.x % 128;
    // new_event.y = new_event.y % 128;
    // cout<<new_event.timestamp<<","<<new_event.x<<","<<new_event.y<<","<<new_event.polarity<<endl;
    return is;
}

/*
 *
 *  main()
 * 
 */
int main(int argc, char const *argv[])
{
    ifstream fin("/home/albert/workSpace/data/new_result.txt");
    vector<TrackedFeature> feature_list;
    TrackedFeature temp_Feature;
    double timestamp = -1.0;
    string line;
    int line_counter = 0;
    cout<<"Start reading file"<<endl;
    while(getline(fin, line))
    {
        line_counter++;
        istringstream iss(line);
        if(line_counter%3 == 1)//id row
        {
            iss>>timestamp;
            while(iss)
            {
                iss>>temp_Feature.id;
                feature_list.push_back(temp_Feature);
            }
        }
        else if(line_counter%3 == 2)//x row
        {
            double temp = -1;
            double temp_x = -1;
            int position = 1;
            iss>>temp;
            if(temp!=timestamp)
            {
                cout<<"ts="<<temp<<", timestamp is wrong!"<<endl;
                return 0;
            }
            while(iss)
            {
                iss>>temp_x;
                feature_list.at(position).x = temp_x;
                position++;
            }
        }
        else //y row
        {
            double temp = -1;
            double temp_y = -1;
            int position = 1;
            iss>>temp;
            if(temp!=timestamp)
            {
                cout<<"ts="<<temp<<", timestamp is wrong!"<<endl;
                return 0;
            }
            while(iss)
            {
                iss>>temp_y;
                feature_list.at(position).y = temp_y;
                position++;
            }
        }
        cout<<"\nResult "<<line_counter<<":\n";
        for(auto ft:feature_list)
        {
            cout<<timestamp<<"  "<<ft.id<<"  "<<ft.x<<"  "<<ft.y<<endl;
        }

    }
    return 0;
}


