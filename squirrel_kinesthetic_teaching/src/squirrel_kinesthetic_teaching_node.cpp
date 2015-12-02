#include <string>
#include <cstdlib>
#include "squirrel_kinesthetic_teaching/TeachingNode.hpp"

using namespace std;
using namespace ros;

int main(int argc,char** argv){

    string name="squirrel_kinesthetic_teaching_node";  //can be read from argv

    init(argc, argv, name);
    cout << "Creating " << name << " node " << endl;
    TeachingNode teacher(name);

    while(ok()) {
	spinOnce();
    }

    return EXIT_SUCCESS;
}
