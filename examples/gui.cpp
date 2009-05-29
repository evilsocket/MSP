#include "../msp.hpp"

using namespace MSP;
  
int main(int argc, char** argv) {
    try{
		Matrix<double> finger("images/finger.png");
		Gui<double>    gui(finger);
		
		while(1){
			sleep(1);	
		}
    }
    catch( MSPException me ){
        me.what();
    }
    
    return (EXIT_SUCCESS);
}

