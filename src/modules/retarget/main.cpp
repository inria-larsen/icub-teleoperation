/**
* Copyright: (C) 2017 Inria
* Author: Luigi Penco
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*/


#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/LogStream.h>

#include "retargetModule.h"

#define DEFAULT_YARP_CONTEXT "retarget"

using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;
using namespace std;



int main (int argc, char * argv[])
{
    //Creating and preparing the Resource Finder
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("retarget.ini");         //default config file name.
    rf.setDefaultContext(DEFAULT_YARP_CONTEXT); //when no parameters are given to the module this is the default context
    rf.configure(argc,argv);

    if (rf.check("help"))
    {
        yInfo()<< "Possible parameters";
        yInfo()<< "\t--context          :Where to find an user defined .ini e.g. /" << DEFAULT_YARP_CONTEXT << "conf" ;
        yInfo()<< "\t--from             :Name of the file .ini user for configuration." ;
        yInfo()<< "\t--robot            :Robot name, overload the setting contained in the wbi_conf_file configuration file."                                                ;
        yInfo()<< "\t--rate             :Period (in ms) used by the module. Default set to 10ms."                                                      ;
        yInfo()<< "\t--name             :Prefix of the ports opened by the module. Set to the module name by default, i.e. jointMapping."    ;
        return 0;
    }

    Network yarp;

    if (!yarp.checkNetwork())
    {
        fprintf(stderr,"Sorry YARP network is not available\n");
        return -1;
    }

    //Creating the module
    retargetModule module;
    return module.runModule(rf);
}
