#include "../msp.hpp"

using namespace MSP;
  
void callback( uint epoch, Matrix<double>& input, Matrix<double>& target, double error ){
    printf( "epoch[%.4d] : %f \n", epoch, error );
}

int main(int argc, char** argv) {
    try{
        Matrix<uint> layers(3);
        
        layers.set( 2, 4, 1 );
        
        AI ai( layers, 0.6, 0.1 );
        
        Matrix<double> i1(2), i2(2), i3(2), i4(2), 
                       t1(1), t2(1), t3(1), t4(1);
        
        i1.set( 0.0, 0.0 ); t1(0) = 0.0;
        i2.set( 0.0, 1.0 ); t2(0) = 0.0;
        i3.set( 1.0, 0.0 ); t3(0) = 0.0;
        i4.set( 1.0, 1.0 ); t4(0) = 1.0;
       
        vector< Matrix<double> * > in, out;
       
        in.push_back(&i1);
        in.push_back(&i2);
        in.push_back(&i3);
        in.push_back(&i4);
        
        out.push_back(&t1);
        out.push_back(&t2);
        out.push_back(&t3);
        out.push_back(&t4);
        
        ai.setcallback(callback);
        
        ai.train( in, out, 100000000, 0.0000001 );
        
        printf( "1 AND 1 : %f\n", ai.urecognize( i4 ) );
        
        ai.save("and.ai");
    }
    catch( MSPException me ){
        me.what();
    }
    
    return (EXIT_SUCCESS);
}

