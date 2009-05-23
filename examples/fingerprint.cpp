#include "../msp.hpp"

using namespace MSP;
  
int main(int argc, char** argv) {
    try{
        Matrix<double> finger("images/finger.png"), 
                       original(finger),
                       orientation, 
                       frequency, 
                       entropy,
                      *feature;
        vector< Matrix<double> * > features;
        vector< Matrix<double> * >::iterator i;
               
        finger.soften();
		finger.save( "soften.png" );
		
        finger.orientation( orientation );
		orientation.save( "orientation.png" );
        finger.frequency( frequency, orientation );
		frequency.save( "frequency.png" );
        finger.entropy( entropy, frequency );
		entropy.save( "entropy.png" );
        finger.gabor( orientation, frequency );
		finger.save( "gabor.png" );
        finger.binarize();
		finger.save( "binarize.png" );
        finger.thin();
		finger.save( "thin.png" );
        finger.features( features, orientation, entropy, 3, 4 );

        double RED[] = { 255, 0, 0 };
        
        for( i = features.begin(); i != features.end(); i++ ){
            feature = *i;
            original.rect( (*feature)(0), (*feature)(1), 3, 3, RED );
            delete *i;
        }
        
        original.save("featured.png");
    }
    catch( MSPException me ){
        me.what();
    }
    
    return (EXIT_SUCCESS);
}

