/***************************************************************************
 *   @file msp.h                                                           *
 *   @version 0.1beta                                                      *
 *   @brief Multi-dimensional Space Processing Library .                   *
 *   @author Simone Margaritelli (aka evilsocket) <evilsocket@gmail.com>   *
 *                       		                                   *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#ifndef __msp_hpp__
#   define __msp_hpp__

#ifndef _GNU_SOURCE
#   define _GNU_SOURCE
#endif

#include <math.h>
#include <memory.h>
#include <sys/types.h>
#include <dirent.h>
#include <errno.h>
#include <stdlib.h>

#include <X11/Xlib.h>
#include <X11/keysym.h>

#include <exception>
#include <string>
#include <vector>

using std::vector;
using std::string;


typedef unsigned int uint;

/**
 * @namespace MSP
 * @brief Multi-dimensional Space Processing Library core namespace . 
 */
namespace MSP {
        
/*! Helper constants . */
#define BLOCK_L  32
#define BLOCK_L2 16
#define BLOCK_W  16
#define BLOCK_W2 8
#define WG2      8
#define EPSILON  0.0001
#define LPSIZE   3
#define LPFACTOR 0.020408163

#ifndef M_PI
#   define M_PI 3.1415926535897932384626433832795
#endif
/*! M_PI / 2.0 */
#define H_M_PI  1.5707963267948965579989817342720925807952880859375
/*! M_PI * 2.0 */
#define D_M_PI  6.28318530717958623199592693708837032318115234375
    
#define DEFAULT_SOFTEN_SIZE 3
    
#define DEFAULT_MIN_FREQ 0.04
#define DEFAULT_MAX_FREQ 0.33333333333333333333333333333333333333333333333
    
#define DEFAULT_ORIENT_BSIZE 7
#define DEFAULT_ORIENT_FSIZE 8
    
#define DEFAULT_GABOR_RADIUS 4.0    
    
#define DEFAULT_L_VAL 0
#define DEFAULT_H_VAL 255
    
/**
 * @def Dimension check flag .
 * @brief Undefine this to exclude dimensions checks of the system . Undefining it will speed up computations, but you will risk seg faults .
 */
#define MSPDIMCHECK

/**
 * @def Imaging capability flag .
 * @brief Undefine this to exclude image loading/saving capabilities of the system .
 */  
#define USEMAGICK

#ifdef USEMAGICK
#   include <magick/api.h>
#endif

/**
 * @enum RESIZEMETHOD
 * @brief Depth resizing method enumeration . 
 */
typedef enum {
    TO_CHOP,    /*! Just cut down (or up in case of depth increasing) values on z-axis . */
    TO_AVERAGE, /*! Compute average among values on z-axis . */
    TO_BIGGER,  /*! Compute bigger among values on z-axis . */
    TO_SMALLER  /*! Compute smaller among values on z-axis . */
}
RESIZEMETHOD;

/**
 * @enum MASKMETHOD
 * @brief Mask application type enumeration . 
 */
typedef enum {
    MASK_EXCLUDE_OUTER, /*! Exclude mask outer bytes . */
    MASK_EXCLUDE_INNER, /*! Exclude mask inner bytes . */
}
MASKMETHOD;

/**
 * @class MSPException
 * @brief General exception class for Matrix class .
 * @see class Matrix
 */
class MSPException : public std::exception {
private :
    /*! File that generated the exception . */
    string file;
    /*! Line of code where the exception occurred . */
    uint   line;
    /*! Exception message . */
    string message;
    
public  :
    
    /**
     * @brief Constructor of MSPException class .
     * @param file File that generated the exception .
     * @param line Line of code where the exception occurred .
     * @param format Format string for exception message .
     * @parm ... Other parameters to put in `format` .
     */
    MSPException( char *file, uint line, const char *format, ... ) : std::exception() {
        char message[0xFF] = {0x00};
        va_list ap;
        
        va_start( ap, format );
        vsprintf( message, format, ap );
        va_end(ap);
        
        this->file    = file;
        this->line    = line;
        this->message = message;
    }
    /**
     * @brief Destructor of MSPException class .
     */
    ~MSPException() throw(){}
    /**
     * @brief Print to `stderr` the whole exception body .
     */
    void what(){
        fprintf( stderr, "%s (line %d) : %s\n", file.c_str(), line, message.c_str() );
    }
};

/**
 * @class Matrix
 * @brief Multidimensional rapresentation of an object .
 */
template <typename T> class Matrix{
private :
    /**
     * @brief Return the average among the values .
     * @param values Vector of values to compute the average of .
     * @param size Size of the values vector .
     * @return The average among values .
     */
    inline T average( T *values, uint size );
    /**
     * @brief Return the bigger among the values .
     * @param values Vector of values to compute the bigger of .
     * @param size Size of the values vector .
     * @return The bigger among values .
     */
    inline T bigger( T *values, uint size );
    /**
     * @brief Return the smaller among the values .
     * @param values Vector of values to compute the smaller of .
     * @param size Size of the values vector .
     * @return The smaller among values .
     */
    inline T smaller( T *values, uint size );
    /**
     * @brief Private method to resize the matrix according to chopping algorithm .
     * @param width New width of the matrix .
     * @param height New height of the matrix . 
     * @param depth New depth of the matrix .
     * @see RESIZEMETHOD::TO_CHOP
     */
    inline void resize_chop( uint width, uint height, uint depth );
    /**
     * @brief Private method to resize the matrix according to average algorithm .
     * @param width New width of the matrix .
     * @param height New height of the matrix . 
     * @param depth New depth of the matrix .
     * @see RESIZEMETHOD::TO_AVERAGE
     */
    inline void resize_average( uint width, uint height, uint depth );
    /**
     * @brief Private method to resize the matrix according to bigger algorithm .
     * @param width New width of the matrix .
     * @param height New height of the matrix . 
     * @param depth New depth of the matrix .
     * @see RESIZEMETHOD::TO_BIGGER
     */
    inline void resize_bigger( uint width, uint height, uint depth );
    /**
     * @brief Private method to resize the matrix according to smaller algorithm .
     * @param width New width of the matrix .
     * @param height New height of the matrix . 
     * @param depth New depth of the matrix .
     * @see RESIZEMETHOD::TO_SMALLER
     */
    inline void resize_smaller( uint width, uint height, uint depth );
    /**
     * @brief Private method to apply the mask to the matrix according to exclude-outer algorithm .
     * @param mask The mask to apply .
     * @param trigger Value to consider positive on the mask . 
     * @see MASKMETHOD::MASK_EXCLUDE_OUTER
     */
    inline void mask_ex_outer( Matrix& mask, T trigger );
    /**
     * @brief Private method to apply the mask to the matrix according to exclude-inner algorithm .
     * @param mask The mask to apply .
     * @param trigger Value to consider positive on the mask . 
     * @see MASKMETHOD::MASK_EXCLUDE_INNER
     */
    inline void mask_ex_inner( Matrix& mask, T trigger );
    /**
     * @brief Private method to apply the mask to the matrix according to exclude-outer algorithm .
     * @param mask The mask to apply .
     * @see MASKMETHOD::MASK_EXCLUDE_OUTER
     */
    inline void mask_ex_outer( Matrix& mask );
    /**
     * @brief Private method to apply the mask to the matrix according to exclude-inner algorithm .
     * @param mask The mask to apply .
     * @see MASKMETHOD::MASK_EXCLUDE_INNER
     */
    inline void mask_ex_inner( Matrix& mask );
    /**
     * @brief Low pass filter to compute orientation mask of the matrix .
     * @param theta Matrix of atan between x and y coefficients .
     * @param orientation Direction matrix .
     * @param filtersize Filter mask size .
     * @param width Width of the matrix to compute orientation mask of .
     * @param height Height of the matrix to compute orientation mask of .
     * @see Matrix::orientation
     */
    inline void orientation_lowpass( Matrix<double>& theta, Matrix<double>& orientation, uint filtersize, uint width, uint height );
    /**
     * @brief Gabor operator .
     * 
     * The Gabor filter operator is defined as
     *
     * \f$ g(x,y;\lambda,\theta,\psi,\sigma,\gamma)=\exp\left(-\frac{x'^2+\gamma^2y'^2}{2\sigma^2}\right)\cos\left(2\pi\frac{x'}{\lambda}+\psi\right) \f$
     *      
     * where
     *
     * \f$ x' = x \cos\theta + y \sin\theta\, \f$
     * 
     * and
     *
     * \f$ y' = -x \sin\theta + y \cos\theta\, \f$
     *
     * @param x X coordinate .
     * @param y Y coordinate .
     * @param phi Direction factor of (x,y) element .
     * @param frequency Frequency factor of (x,y) element .
     * @param radius Radius for Gabor operator .
     * @return The Gabor transform value of (x,y) element .
     */
    inline double gabor( int x, int y, double phi, double frequency, double radius );
    
    /**
     * @brief Find empty neigbours in entropy mask .
     * @param x X coordinate of the region to search in .
     * @param y Y coordinate of the region to search in .
     * @param region Region size .
     * @return The number of empty neighbours in that region .
     */
    inline uint entropy_empty_neighbours( Matrix& entropy, uint x, uint y, uint region );
    
    /**
     * @brief Compute difference percentage between `total` and `value` .
     * @param total First parameter to use for difference computing .
     * @param value Second parameter to use for difference computing .
     * @return Difference percentage between `total` and `value` .
     */
    inline double delta( T total, T value );
    
#ifdef USEMAGICK
    /*! Flag to check imagemagick initialization . */
    bool magick_initialized;
#endif
    
public  :
    /*! Inner buffer allocation flag . */
    bool  allocated;
    /*! Width of the matrix . */
    uint  width;
    /*! Height of the matrix . */
    uint  height;
    /*! Depth of the matrix . */
    uint  depth;
    /*! Matrix inner buffer . */
    T *** data;
    /*! Empty (uninitialized) matrix constructor . */
    Matrix();
    /**
     * @brief Standard matrix constructor .
     * @param width Width of the matrix to create .
     * @param height Height of the matrix to create .
     * @param depth Depth of the matrix to create .
     */
    Matrix( uint width, uint height = 1, uint depth = 1 );
    /**
     * @brief Clone matrix constructor .
     * @param m Matrix to clone .
     */
    Matrix( Matrix& m );
    /**
     * @brief Clone matrix constructor .
     * @param m Matrix to clone .
     */
    Matrix( const Matrix& m );
    /**
     * @brief Vector to matrix constructor .
     * @param rawdata Pointer to vector raw data, could be 1D (a vector) or nD (a matrix) .
     * @param width Width of the raw data .
     * @param height Height of the raw data .
     * @param depth Depth of the raw data .
     */
    Matrix( T *rawdata, uint width, uint height, uint depth );
        
#ifdef USEMAGICK
    /**
     * @brief Image loading constructor .
     * @param filename File name of the image to load as a matrix .
     * @see Matrix::load
     */
    Matrix( const char *filename );
#endif
    /*! Matrix class destructor */
    ~Matrix();
    
#ifdef USEMAGICK
    /**
     * @brief Method to load an image as a matrix .
     * @param filename File name of the image to load as a matrix .
     * @return A reference of the matrix .
     */
    inline Matrix& load( const char *filename );
    /**
     * @brief Method to save the matrix into an image .
     * @param filename File name of the image to save matrix in .
     * @return A reference of the matrix . */
    inline Matrix& save( const char *filename );
#endif
    /*! Return the size in bytes of entire matrix . */
    inline uint size();
    
    /*! Fill the vector with values specified by argument list . */
    inline Matrix& set( T v, ... );
    
    /*! Fill the matrix with `value` . */
    inline Matrix& fill( T value );
    /**
     * @brief Fill sector (x,y,x + width,y + height) with `value` .
     * @param x X coordinate of top left corner of the sector . 
     * @param y Y coordinate of top left corner of the sector .
     * @param width Width of the sector to fill .
     * @param height Height of the sector to fill .
     * @param value Value to use to fill the sector .
     */
    inline Matrix& fill( uint x, uint y, uint width, uint height, T value );
    
    /**
     * @brief Draw a line between (x1,y1) and (x2,y2) .
     * @param x1 X coordinate of the start point . 
     * @param y1 Y coordinate of the start point .
     * @param x2 X coordinate of the end point .
     * @param y2 Y coordinate of the end point .
     * @param color Color of the line to draw .
     */
    inline Matrix& line( uint x1, uint y1, uint x2, uint y2, T * color );
    /**
     * @brief Draw an outlined rectangle between at top point (x,y) .
     * @param x X coordinate of top left corner of the rectangle . 
     * @param y Y coordinate of top left corner of the rectangle . 
     * @param width Width of the rectangle .
     * @param height Height of the rectangle .
     * @param color Color of the rectangle outline .
     */
    inline Matrix& rect( uint x, uint y, uint width, uint height, T * color );
    
    /**
     * @brief Clear a matrix sector .
     * @param x X coordinate of top left corner of the sector to clear . 
     * @param y Y coordinate of top left corner of the sector to clear .
     * @param width Width of the sector to clear .
     * @param height Height of the sector to clear .
     */
    inline Matrix& clear( uint x, uint y, uint width, uint height );
    /*! Clear the matrix . */
    inline Matrix& clear();
    /*! Assign `m` to this matrix . */
    inline Matrix& assign( Matrix& m );
    /*! Assign `m` to this matrix . */
    inline Matrix& assign( const Matrix& m );
    /*! Resize matrix width, height and depth according to `method` algorithm . */
    inline Matrix& resize( uint width, uint height, uint depth, RESIZEMETHOD method = TO_CHOP );
    /*! Set `v` on row number `row` . */
    inline Matrix& setrow( Matrix& v, uint row );
    /*! Set `v` on column number `col` . */
    inline Matrix& setcol( Matrix& v, uint col );
    /**
     * @brief Create a width*depth vector and flatten inner matrix data .
     * Es : 0 1 2 3
     *      2 3 5 7 ---> 0 1 2 3 2 3 5 7
     */
    inline Matrix& flatten( Matrix& flat );
    /**
     * @brief Create a width*depth vector and flatten inner matrix data .
     * Es : 0 1 2 3
     *      2 3 5 7 ---> 0 1 2 3 2 3 5 7
     */
    inline Matrix& flatten();
    
    /**
     * @brief Apply the mask on the matrix according to `method` algorithm .
     * @param mask The mask to apply .
     * @param trigger Value to consider positive while applying the mask .
     * @param method Algorithm to use while applying the mask .
     * @see enum MASKMETHOD
     */
    inline Matrix& mask( Matrix& mask, T trigger, MASKMETHOD method = MASK_EXCLUDE_OUTER );
    /**
     * @brief Apply the mask on the matrix according to `method` algorithm .
     * @param mask The mask to apply .
     * @param method Algorithm to use while applying the mask .
     * @see enum MASKMETHOD
     */
    inline Matrix& mask( Matrix& mask, MASKMETHOD method = MASK_EXCLUDE_OUTER );
   
    /**
     * @brief Edge detection with Prewitt algorithm .
     *
     * Mathematically, the operator uses two 3×3 kernels which are convolved with the original image to calculate approximations <br/>
     * of the derivatives - one for horizontal changes, and one for vertical. <br/>
     * If we define \f$ \mathbf{A} \f$ as the source image, and \f$ \mathbf{G_x} \f$ and \f$ \mathbf{G_y} \f$ are two images which at each point<br/>
     * contain the horizontal and vertical derivative approximations, the latter are computed as :<br/>
     *
     * \f$ 
     *  \mathbf{G_x} = \begin{array}{ccc} 
     *  -1 & 0 & +1 \\
     *  -1 & 0 & +1 \\
     *  -1 & 0 & +1 
     *  \end{array} * \mathbf{A}
     *  \quad \mbox{and} \quad 
     *  \mathbf{G_y} = \begin{array}{ccc}
     *  -1 & -1 & -1 \\
     *    0 & 0 & 0 \\
     *  +1 & +1 & +1 
     *  \end{array} * \mathbf{A} \f$
     *
     * @param edges Destination matrix for edges .
     * @param threshold Threshold value to use in Prewitt algorithm .
     */
    inline Matrix& edges( Matrix& edges, uint threshold );
    /**
     * @brief Edge detection with Prewitt algorithm .
     *
     * Mathematically, the operator uses two 3×3 kernels which are convolved with the original image to calculate approximations <br/>
     * of the derivatives - one for horizontal changes, and one for vertical. <br/>
     * If we define \f$ \mathbf{A} \f$ as the source image, and \f$ \mathbf{G_x} \f$ and \f$ \mathbf{G_y} \f$ are two images which at each point<br/>
     * contain the horizontal and vertical derivative approximations, the latter are computed as :<br/>
     *
     * \f$ 
     *  \mathbf{G_x} = \begin{array}{ccc} 
     *  -1 & 0 & +1 \\
     *  -1 & 0 & +1 \\
     *  -1 & 0 & +1 
     *  \end{array} * \mathbf{A}
     *  \quad \mbox{and} \quad 
     *  \mathbf{G_y} = \begin{array}{ccc}
     *  -1 & -1 & -1 \\
     *    0 & 0 & 0 \\
     *  +1 & +1 & +1 
     *  \end{array} * \mathbf{A} \f$
     *
     * @param threshold Threshold value to use in Prewitt algorithm .
     */
    inline Matrix& edges( uint threshold );
    /**
     * @brief Hit-and-miss thinning algorithm .
     * @param thinned Destination matrix for thinning results .
     */
    inline Matrix& thin( Matrix& thinned );
    /**
     * @brief Hit-and-miss thinning algorithm .
     */
    inline Matrix& thin();
    /**
     * @brief Compute orientation map of the matrix .
     *
     * Compute a map that contains the ridge orientation of the matrix .
     * The values representing the orientation vary between -PI/2 and PI/2 .
     * The low-pass filter size is intented to exclude errors from computation .
     *
     * @param orientation Destination matrix for the map .
     * @param blocksize Size of block to consider for map computation .
     * @param filtersize Low-pass filter size .
     * @see Matrix::orientation_lowpass
     */
    inline Matrix& orientation( Matrix<double>& orientation, uint blocksize = DEFAULT_ORIENT_BSIZE, int filtersize = DEFAULT_ORIENT_FSIZE );
    /**
     * @brief Compute orientation map of the matrix .
     *
     * Compute a map that contains the ridge orientation of the matrix .
     * The values representing the orientation vary between -PI/2 and PI/2 .
     * The low-pass filter size is intented to exclude errors from computation .
     *
     * @param blocksize Size of block to consider for map computation .
     * @param filtersize Low-pass filter size .
     * @see Matrix::orientation_lowpass
     */
    inline Matrix& orientation( uint blocksize = DEFAULT_ORIENT_BSIZE, int filtersize = DEFAULT_ORIENT_FSIZE );
    /**
     * @brief Compute frequency map of the matrix .
     * @param frequency Destination matrix for the map .
     * @param orientation Orientation map computed with Matrix::orientation method .
     */
    inline Matrix& frequency( Matrix<double>& frequency, Matrix<double>& orientation );
    /**
     * @brief Compute frequency map of the matrix .
     * @param orientation Orientation map computed with Matrix::orientation method .
     */
    inline Matrix& frequency( Matrix<double>& orientation );
    /**
     * @brief Gabor transform enhancement of the matrix .
     *
     * The Gabor filter operator is defined as
     *
     * \f$ g(x,y;\lambda,\theta,\psi,\sigma,\gamma)=\exp\left(-\frac{x'^2+\gamma^2y'^2}{2\sigma^2}\right)\cos\left(2\pi\frac{x'}{\lambda}+\psi\right) \f$
     *      
     * where
     *
     * \f$ x' = x \cos\theta + y \sin\theta\, \f$
     * 
     * and
     *
     * \f$ y' = -x \sin\theta + y \cos\theta\, \f$
     *
     * @param gabor Destination matrix for Gabor transform .
     * @param orientation Orientation map computed with Matrix::orientation method .
     * @param frequency Frequency map computed with Matrix::frequency method .
     * @param radius The radius to use for Gabor transform .
     */
    inline Matrix& gabor( Matrix& gabor, Matrix<double>& orientation, Matrix<double>& frequency, double radius = DEFAULT_GABOR_RADIUS );
    /**
     * @brief Gabor transform enhancement of the matrix .
     *
     * The Gabor filter operator is defined as
     *
     * \f$ g(x,y;\lambda,\theta,\psi,\sigma,\gamma)=\exp\left(-\frac{x'^2+\gamma^2y'^2}{2\sigma^2}\right)\cos\left(2\pi\frac{x'}{\lambda}+\psi\right) \f$
     *      
     * where
     *
     * \f$ x' = x \cos\theta + y \sin\theta\, \f$
     * 
     * and
     *
     * \f$ y' = -x \sin\theta + y \cos\theta\, \f$
     *
     * @param orientation Orientation map computed with Matrix::orientation method .
     * @param frequency Frequency map computed with Matrix::frequency method .
     * @param radius The radius to use for Gabor transform .
     */
    inline Matrix& gabor( Matrix<double>& orientation, Matrix<double>& frequency, double radius = DEFAULT_GABOR_RADIUS );
    /**
     * @brief Soften mean matrix blurring .
     * @param soften Destination matrix for soften mean computation .
     * @param size Sector size for the soften mean computing .
     */
    inline Matrix& soften( Matrix& soften, uint size = DEFAULT_SOFTEN_SIZE );
    /**
     * @brief Soften mean matrix blurring .
     * @param size Sector size for the soften mean computing .
     */
    inline Matrix& soften( uint size = DEFAULT_SOFTEN_SIZE );
    /**
     * @brief Compute the entropy mask of the matrix .
     * @param entropy Destination matrix for map computing .
     * @param hvalue Positive value to use while building the map .
     * @param frequency Frequency map computed with Matrix::frequency .
     * @param lthreshold Minimum threshold value .
     * @param hthreshold Maximum threshold value .
     */
    inline Matrix& entropy( Matrix& entropy, Matrix<double>& frequency, double hvalue = 255.0, double lthreshold = DEFAULT_MIN_FREQ, double hthreshold = DEFAULT_MAX_FREQ );
    /**
     * @brief Compute the entropy mask of the matrix .
     * @param hvalue Positive value to use while building the map .
     * @param frequency Frequency map computed with Matrix::frequency .
     * @param lthreshold Minimum threshold value .
     * @param hthreshold Maximum threshold value .
     */
    inline Matrix& entropy( Matrix<double>& frequency, double hvalue = 255.0, double lthreshold = DEFAULT_MIN_FREQ, double hthreshold = DEFAULT_MAX_FREQ );
    /**
     * @brief Compute a matrix dilatation .
     * @param dilated Destination matrix for dilatation .
     * @param threshold Threshold dilatation value .
     * @param mask Dilatation mask value (usually should be threshold / 2) .
     */
    inline Matrix& dilate( Matrix& dilated, T threshold, T mask );
    /**
     * @brief Compute a matrix dilatation .
     * @param threshold Threshold dilatation value .
     * @param mask Dilatation mask value (usually should be threshold / 2) .
     */
    inline Matrix& dilate( T threshold, T mask );
    /**
     * @brief Compute a matrix erosion .
     * @param eroded Destination matrix for erosion .
     * @param lthreshold Low threshold erosion value .
     * @param hthreshold High threshold erosion value .
     * @param mask Erosion mask value .
     */
    inline Matrix& erode( Matrix& eroded, T lthreshold, T hthreshold, T mask );
    /**
     * @brief Compute a matrix erosion .
     * @param lthreshold Low threshold erosion value .
     * @param hthreshold High threshold erosion value .
     * @param mask Erosion mask value .
     */
    inline Matrix& erode( T lthreshold, T hthreshold, T mask );
    /**
     * @brief Compute the value histogram of a region of the matrix .
     * @param histogram Destination matrix for histogram computation .
     * @param x X coordinate of the region .
     * @param y Y coordinate of the region .
     * @param width Width of the region .
     * @param height Height of the region .
     */
    inline Matrix& histogram( Matrix<uint>& histogram, uint x, uint y, uint width, uint height );
    /**
     * @brief Compute the value histogram of the matrix .
     * @param histogram Destination matrix for histogram computation .
     */
    inline Matrix& histogram( Matrix<uint>& histogram );
    /*! Compute the value histogram of the matrix . */
    inline Matrix& histogram();
    /**
     * @brief Compute the mean value of the matrix .
     * @param histogram Matrix histogram computed with Matrix::histogram .
     * @return The mean value of the matrix .
     */
    inline double  mean( Matrix<uint>& histogram );
    /**
     * @brief Compute the variance value of the matrix .
     * @param histogram Matrix histogram computed with Matrix::histogram .
     * @return The variance value of the matrix .
     */
    inline double  variance( Matrix<uint>& histogram );
    /**
     * @brief Compute thresholded binarization of the matrix .
     * @param binarized Destination matrix for binarization computing .
     * @param lthreshold Low threshold for binarization .
     * @param hthreshold High threshold for binarization .
     */
    inline Matrix& binarize( Matrix& binarized, T lthreshold = DEFAULT_L_VAL, T hthreshold = DEFAULT_H_VAL );
    /**
     * @brief Compute thresholded binarization of the matrix .
     * @param lthreshold Low threshold for binarization .
     * @param hthreshold High threshold for binarization .
     */
    inline Matrix& binarize( T lthreshold = DEFAULT_L_VAL, T hthreshold = DEFAULT_H_VAL );
    /**
     * @brief Compute normalization of the matrix .
     * @param normalized Destination matrix for normalization computing .
     * @param mean Mean value of the matrix computed with Matrix::mean .
     * @param variance Variance value of the matrix computed with Matrix::variance .
     */
    inline Matrix& normalize( Matrix& normalized, double mean, double variance );
    /**
     * @brief Compute normalization of the matrix .
     * @param mean Mean value of the matrix computed with Matrix::mean .
     * @param variance Variance value of the matrix computed with Matrix::variance .
     */
    inline Matrix& normalize( double mean, double variance );
    /**
     * @brief Compute the transpose of the matrix .
     * @param transposed Destination matrix for transpose computation .
     */
    inline Matrix& transpose( Matrix& transposed );
    /*! Compute the transpose of the matrix . */
    inline Matrix& transpose();
    /**
     * @brief Compute matrix integration .
     * @param integral Destination matrix for integral computation .
     */
    inline Matrix& integral( Matrix& integral );
    /*! Compute matrix integration . */
    inline Matrix& integral();
    
    /**
     * @brief Compute matrix features by neighbours counting .
     * @param features Destination vector of the features .<br/>
     *                 Each feature is a 4x1 matrix in the form ( x, y, neighbours, orientation ) .
     * @param orientation Orientation mask computed with Matrix::orientation .
     * @param entropy Entropy mask computed with Matrix::entropy .
     * @param minneighbours Minimum number of neighbours for a pixel to be considered as a feature .
     * @param maxneighbours Maximum number of neighbours for a pixel to be considered as a feature .
     */
    inline Matrix& features( vector< Matrix<T> * >& features, 
                             Matrix& orientation,
                             Matrix& entropy,
                             uint minneighbours,
                             uint maxneighbours );
    
    /**
     * @brief Compute difference percentage between a sector of `m` and the relative sector in the matrix .
     * @param m The matrix to use for difference computing .
     * @param x X coordinate of the sector .
     * @param y Y coordinate of the sector .
     * @param width Width of the sector .
     * @param height Height of the sector .
     * @return Difference percentage between a sector of `m` and the relative sector in the matrix .
     */
    inline double delta( Matrix& m, uint x, uint y, uint widht, uint height );
    
    
    
    /**
     * @brief [] operator .
     * @param x X coordinate .
     * @return Pointer to X'th column .
     */
    inline T **    operator [] ( uint x );
    /**
     * @brief () operator .
     * @param x X coordinate .
     * @param y Y coordinate .
     * @param z Z coordinate .
     * @return Reference to value at (x,y,z) coordinates .
     */
    inline T&      operator () ( uint x, uint y = 0, uint z = 0 );
    /*! Assignation operator . */
    inline Matrix& operator =  ( Matrix& m );
    /*! Assignation operator . */
    inline Matrix& operator =  ( Matrix * m );
    /*! Compute linear sum with the matrix `m` . */
    inline Matrix& operator += ( Matrix& m );
    /*! Compute linear subtraction with the matrix `m` . */
    inline Matrix& operator -= ( Matrix& m );
    /*! Compute multiplication with the matrix `m` . */
    inline Matrix& operator *= ( Matrix& m );
    /*! Compute division with the matrix `m` . */
    inline Matrix& operator /= ( Matrix& m );
    
    /*! Compute linear sum with the scalar `s` . */
    inline Matrix& operator += ( T s );
    /*! Compute linear subtraction with the scalar `s` . */
    inline Matrix& operator -= ( T s );
    /*! Compute multiplication with the scalar `s` . */
    inline Matrix& operator *= ( T s );
    /*! Compute division with the scalar `s` . */
    inline Matrix& operator /= ( T s );
};

template <typename T> inline Matrix<T>& operator + ( Matrix<T>& a, Matrix<T>& b );
template <typename T> inline Matrix<T>& operator - ( Matrix<T>& a, Matrix<T>& b );
template <typename T> inline Matrix<T>& operator * ( Matrix<T>& a, Matrix<T>& b );
template <typename T> inline Matrix<T>& operator / ( Matrix<T>& a, Matrix<T>& b );

/**
 * @class System
 * @brief System utilities grouping class .
 */
class System{
public :   
    /**
     * @brief Load a vector of matrices from `path` according to `filter` file name pattern .
     * @param set Destination set to load matrices in .
     * @param path Path to load matrices from .
     * @param filter File name pattern to use for files filtering .
     */
    template <typename T> 
    inline static void loadset( vector< Matrix<T> * >& set, const char *path, const char *filter ){
        DIR *dp;
        struct dirent *dent;
        char filename[0xFF] = {0};

        if( (dp = opendir( path )) == NULL ){
            throw MSPException( __FILE__, __LINE__, "Could not open path '%s' for reading .", path );
        }
        while( (dent = readdir(dp)) != NULL ){
            if( strstr( dent->d_name, filter ) != 0 ){
                sprintf( filename, "%s/%s", path, dent->d_name );    
                set.push_back( new Matrix<T>(filename) );
            }
        }
        closedir( dp );
    }
    /**
     * @brief Destroy a vector of matrices previously loaded with System::loadset .
     * @param set Matrix set to destroy .
     */
    template <typename T> 
    inline static void freeset( vector< Matrix<T> * >& set ){
        uint i;
        for( i = 0; i < set.size(); i++ ){
            delete set[i];
        }
    }
};

/**
 * @brief Function prototype for AI training epoch callback .
 * @param epoch Epoch of training process .
 * @param input Input pattern .
 * @param target Desired target pattern .
 * @param error Error value for this training epoch .
 */
typedef	void (* AIEpochCallback)( uint epoch, Matrix<double>& input, Matrix<double>& target, double error );

/**
 * @class AI
 * @brief Back-propagation neural network class .
 */
class AI {
    private :
        
               /*! Output value of each neuron . */
        double **  m_out,   
               /*! Delta error value for each neuron . */
               **  m_delta,      
               /*! Vector of weights for each neuron . */
               *** m_weight,     
               /*! Storage for weight-change made in previous epoch . */
               *** m_last_state, 
               /*! Learning rate . */
               m_learning_rate, 
               /*! Momentum parameter . */
               m_momentum;       

        /*! Vector that contains each layer size . */
        Matrix<uint> m_layers;
               
               /*! Number of layers in net including input layer . */
        int    m_numlayers,      
               /*! Input layer index (usually is 0) . */
               m_in_idx,         
               /*! Output layer index (usually m_numlayers - 1) . */
               m_out_idx;        

        /*! Training epoch callback function pointer . */
        AIEpochCallback m_epoch_callback ;
        
        /**
         * @brief Sigmoid high-pass function .
         * 
         * The sigmoid function is used as a filter for output values, and it is defined as :
         * \f$ P(t) = \frac{1}{1 + e^{-t}} \f$
         *
         * @param value The value to compute the sigmoid of .
         * @return Sigmoid result .
         */
        inline double sigmoid(double value){
            return (double)(1 / (1 + exp(-value)));
        }

    public  :

        /**
         * @brief AI default class constructor .
         * @param layers Vector that contains each layer size .
         * @param learningrate Learning rate value for the network .
         * @param momentum Momentum value for the network .
         */
        AI( Matrix<uint>& layers, double learningrate, double momentum ){
            uint i = 0, j = 0, k = 0;
            
            m_learning_rate  = learningrate ;
            m_momentum	     = momentum ;
            m_numlayers      = layers.width;
            m_layers         = layers;
            m_in_idx         = 0;
            m_out_idx        = m_numlayers - 1;
            m_epoch_callback = (AIEpochCallback)0;
           
            m_out = new double * [m_numlayers];
            for( i = 0; i < m_numlayers; i++ ){
                m_out[i] = new double[m_layers(i)];
            }
            
            m_delta = new double * [m_numlayers];
            for( i = 1; i < m_numlayers; i++ ){
                m_delta[i] = new double[m_layers(i)];
            }
            
            m_weight = new double ** [m_numlayers];
            for( i = 1; i < m_numlayers; i++ ){
                m_weight[i] = new double*[m_layers(i)];
            }
            for( i = 1; i < m_numlayers; i++ ){
                for( j = 0; j < m_layers(i); j++ ){
                    m_weight[i][j] = new double[m_layers(i-1) + 1];
                }
            }
            
            m_last_state = new double ** [m_numlayers];
            for( i = 1; i < m_numlayers; i++ ){
                m_last_state[i] = new double*[m_layers(i)];
            }
            for( i = 1; i < m_numlayers; i++ ){
                for( j = 0; j < m_layers(i); j++ ){
                    m_last_state[i][j] = new double[m_layers(i-1) + 1];
                }
            }
            
            srand((unsigned)(time(NULL)));
            for( i = 1; i < m_numlayers; i++ ){
                for( j = 0; j < m_layers(i); j++ ){
                    for( k = 0; k < m_layers(i-1) + 1; k++ ){
                        m_weight[i][j][k]= (double)(rand())/(RAND_MAX/2) - 1;
                    }
                }
            }
            
            for( i = 1; i < m_numlayers; i++ ){
                for( j = 0; j < m_layers(i); j++ ){
                    for( k = 0; k < m_layers(i-1) + 1; k++ ){
                        m_last_state[i][j][k] = (double)0.0;
                    }
                }
            }
        }
        /**
         * @brief AI class constructor by file loading .
         * @param filepath File with a previously saved network .
         */
        AI( char * filepath ){
            this->load(filepath);
        }
        /**
         * @brief Class destructor .
         */
        ~AI(){
            uint i = 0, j = 0;
            
            for( i = 0; i < m_numlayers; i++ ){
                delete[] m_out[i];
            }
            delete m_out;
            
            for( i = 1; i < m_numlayers; i++ ){
                delete[] m_delta[i];
            }
            delete m_delta;
            
            for( i = 1; i < m_numlayers; i++ ){
                for( j = 0; j < m_layers(i); j++ ){
                    delete[] m_weight[i][j];
                }
            }
            for( i = 1; i < m_numlayers; i++ ){
                delete m_weight[i];
            }
            delete m_weight;
            
            for( i = 1; i < m_numlayers; i++ ){
                for( j = 0; j < m_layers(i); j++ ){
                    delete[] m_last_state[i][j];
                }
            }
            for( i = 1; i < m_numlayers; i++ ){
                delete m_last_state[i];
            }
            delete m_last_state;
        }
        /**
         * @brief Set the epoch callback function pointer .
         * @param callback The epoch callback functon pointer to use .
         */
        inline void setcallback( AIEpochCallback callback ){
            this->m_epoch_callback = callback;
        }

        /**
         * @brief Set the pattern on the input layer .
         * 
         * The propagation of the pattern is defined as :
         * 
         * \f$ f (x) = sigmoid \left(\sum_i w_i g_i(x)\right) \f$ 
         * 
         * where :
         * 
         * \f$ g_i (x) = \sum_i output_i w_i \f$
         *
         * @param in The input pattern to propagate on the network .
         */
        inline void setinput( Matrix<double>& in ){
            double sum;
            uint i = 0, j = 0, k = 0;
            
            for( i = 0; i < m_layers(m_in_idx); i++ ){
                m_out[m_in_idx][i] = in(i);
            }
            
            /* for each layer */
            for( i = 1; i < m_numlayers; i++ ){
                /* for each neuron in the layer */
                for( j = 0; j < m_layers(i); j++ ){
                    sum = 0.0;
                    /* for input from each neuron in preceeding layer */
                    for( k = 0; k < m_layers(i-1); k++ ){
                        sum += m_out[i-1][k] * m_weight[i][j][k];
                    }
                    /* sum + bias value */
                    m_out[i][j] = sigmoid( sum + m_weight[i][j][ m_layers(i-1) ] );
                }
            }
        }
        /**
         * @brief Back-propagation training algorithm .
         * @param in The input pattern to propagate on the network .
         * @param target The desired target pattern to train the network with .
         */
        inline void train( Matrix<double>& in, Matrix<double>& target ){
            double sum;
            int i = 0,
                    j = 0,
                    k = 0;
            
            setinput(in);
            
            for( i = 0; i < m_layers(m_out_idx); i++ ){
                m_delta[m_out_idx][i] = m_out[m_out_idx][i] * (1 - m_out[m_out_idx][i]) * (target(i) - m_out[m_out_idx][i]);
            }
            
            for( i = m_numlayers - 2; i > 0; i-- ){
                for( j = 0; j < m_layers(i); j++ ){
                    sum = 0.0;
                    for( k = 0; k < m_layers(i + 1); k++){
                        sum += m_delta[i+1][k] * m_weight[i+1][k][j];
                    }
                    m_delta[i][j] = m_out[i][j] * (1 - m_out[i][j]) * sum;
                }
            }
            
            for( i = 1; i < m_numlayers; i++ ){
                for( j = 0; j < m_layers(i); j++ ){
                    for( k = 0; k < m_layers(i - 1); k++ ){
                        m_weight[i][j][k] += m_momentum * m_last_state[i][j][k];
                    }
                    m_weight[i][j][m_layers(i - 1)] += m_momentum * m_last_state[i][j][m_layers(i - 1)];
                }
            }
            
            for( i = 1; i < m_numlayers; i++ ){
                for( j = 0; j < m_layers(i); j++ ){
                    for( k = 0; k < m_layers(i - 1); k++ ){
                        m_last_state[i][j][k] = m_learning_rate * m_delta[i][j] * m_out[i-1][k];
                        m_weight[i][j][k]    += m_last_state[i][j][k];
                    }
                    m_last_state[i][j][m_layers(i - 1)] =  m_learning_rate * m_delta[i][j];
                    m_weight[i][j][m_layers(i - 1)]    += m_last_state[i][j][m_layers(i - 1)];
                }
            }
        }
        /**
         * @brief Compute error value comparing output layer with `target` pattern .
         *  
         * The mean squared error function is defined as :
         * 
         * \f$ error = \left(\sum_i (target_i - output_i)^2\right) / 2 \f$
         * 
         * @param target The pattern to compare with the output layer .
         * @return The mean squared error .
         */
        inline double error( Matrix<double>& target ){
            uint i = 0, lsize = m_layers(m_out_idx);
            double mse = 0.0;
            
            for( i = 0; i < lsize; i++ ){
                mse += pow( (target(i) - m_out[m_out_idx][i]), 2.0 );
            }
            
            return mse / 2.0;
        }
        /**
         * @brief Train the network against N input patterns and N target patterns for `epochs` training loops OR until
         * the `threshold` error value is reached .
         * @param inputs Input training set .
         * @param targets Target training set .
         * @param epochs Maximum number of epochs to use for network training .
         * @param threshold Desired maximum error value .
         */
        inline void train( vector< Matrix<double> * >& inputs, vector< Matrix<double> * >& targets, uint epochs, double threshold ){
            uint idx = 0, i   = 0;
            double error ;
            
            for( i = 0; i < epochs; i++ ){
                idx = i % inputs.size();
                                
                this->train( *inputs[idx], *targets[idx] );
                
                error = this->error( *targets[idx] );
                
                if( m_epoch_callback != (AIEpochCallback)0 ){
                    (*m_epoch_callback)( i, *inputs[idx], *targets[idx], error );
                }
                
                if( error <= threshold ){
                    break;
                }
            }
        }
        /**
         * @brief Return the output layer vector .
         * @return The network output layer .
         */
        inline Matrix<double> output(){
            Matrix<double> out( m_layers(m_out_idx) );
            uint i;
            
            for( i = 0; i < m_layers(m_out_idx); i++ ){
                out(i) = m_out[m_out_idx][i];
            }
            
            return out;
        }
        /**
         * @brief Attempt to recognize the input pattern .
         * @param pattern The input pattern to recognize .
         * @return The output layer values .
         */
        inline Matrix<double> recognize( Matrix<double>& pattern ){
            setinput(pattern);
            return output();
        }
        /**
         * @brief Attempt to recognize the input pattern . <br/> 
         * This function is used instead of AI::recognize when the output layer has only one neuron .
         * @param pattern The input pattern to recognize .
         * @return The output layer value .
         */
        inline double urecognize( Matrix<double>& pattern ){
            setinput(pattern);
            return m_out[m_out_idx][0];
        }
        /**
         * @brief Save the network into a file .
         * @param filepath The file to save the network in .
         */
        inline void save( char * filepath ){
            uint i = 0, j = 0, k = 0;
            
            FILE * fd = fopen( filepath, "w+b" );
            
            if( fd == NULL ){
                throw MSPException( __FILE__, __LINE__, "Could not create file `%s` to save the network .", filepath );
            }
            
            fwrite( (void *)&m_learning_rate, 1, sizeof(double), fd );
            fwrite( (void *)&m_momentum,      1, sizeof(double), fd );
            fwrite( (void *)&m_numlayers,     1, sizeof(int), fd );
            
            for( i = 0; i < m_numlayers; i++ ){
                fwrite( (void *)&m_layers(i), 1, sizeof(int), fd );
            }
            
            for( i = 1; i < m_numlayers; i++ ){
                for( j = 0; j < m_layers(i); j++ ){
                    fwrite( (void *)&m_out[i][j], 1, sizeof(double), fd );
                }
            }
            
            for( i = 1; i < m_numlayers; i++ ){
                for( j = 0; j < m_layers(i); j++ ){
                    fwrite( (void *)&m_delta[i][j], 1, sizeof(double), fd );
                }
            }
            
            for( i = 1; i < m_numlayers; i++ ){
                for( j = 0; j < m_layers(i); j++ ){
                    for( k = 0; k < m_layers(i - 1) + 1; k++ ){
                        fwrite( (void *)&m_weight[i][j][k], 1, sizeof(double), fd );
                    }
                }
            }
            
            fclose(fd);
        }
        /**
         * @brief Load the network from a file .
         * @param filepath The file to load the network from .
         */
        inline void load( char * filepath ){
            uint i = 0, j = 0, k = 0;
            FILE * fd;
            
            if( (fd = fopen( filepath, "r+b" )) ){
                fread( (void *)&m_learning_rate, 1, sizeof(double), fd );
                fread( (void *)&m_momentum,      1, sizeof(double), fd );
                fread( (void *)&m_numlayers,	 1, sizeof(int), fd );
                
                m_layers.resize( m_numlayers, 1, 1 );
                for( i = 0; i < m_numlayers; i++ ){
                    fread( (void *)&m_layers(i), 1, sizeof(int), fd );
                }
                
                m_out = new double * [m_numlayers];
                for( i = 0; i < m_numlayers; i++ ){
                    m_out[i] = new double[m_layers(i)];
                }
                for( i = 1; i < m_numlayers; i++ ){
                    for( j = 0; j < m_layers(i); j++ ){
                        fread( (void *)&m_out[i][j], 1, sizeof(double), fd );
                    }
                }
                
                m_delta = new double * [m_numlayers];
                for( i = 1; i < m_numlayers; i++ ){
                    m_delta[i] = new double[m_layers(i)];
                }
                for( i = 1; i < m_numlayers; i++ ){
                    for( j = 0; j < m_layers(i); j++ ){
                        fread( (void *)&m_delta[i][j], 1, sizeof(double), fd );
                    }
                }
                
                m_weight = new double ** [m_numlayers];
                for( i = 1; i < m_numlayers; i++ ){
                    m_weight[i] = new double*[m_layers(i)];
                }
                for( i = 1; i < m_numlayers; i++ ){
                    for( j = 0; j < m_layers(i); j++ ){
                        m_weight[i][j] = new double[m_layers(i-1)+1];
                    }
                }
                for( i = 1; i < m_numlayers; i++ ){
                    for( j = 0; j < m_layers(i); j++ ){
                        for( k = 0; k < m_layers(i-1)+1; k++ ){
                            fread( (void *)&m_weight[i][j][k], 1, sizeof(double), fd );
                        }
                    }
                }
                
                m_last_state = new double ** [m_numlayers];
                for( i = 1; i < m_numlayers; i++ ){
                    m_last_state[i] = new double*[m_layers(i)];
                }
                for( i = 1; i < m_numlayers; i++ ){
                    for( j = 0; j < m_layers(i); j++ ){
                        m_last_state[i][j] = new double[m_layers(i-1)+1];
                    }
                }
                                
                m_in_idx  = 0 ;
                m_out_idx = m_numlayers - 1 ;
                
                fclose(fd);
            }
            else{
                throw MSPException( __FILE__, __LINE__, "Could not open file `%s` to load the network .", filepath );
            }
        }
};
        
template <typename T> 
Matrix<T>::Matrix(){
    this->width     = 0;
    this->height    = 0;
    this->depth     = 0;
    this->data      = NULL;
    this->allocated = false;
    
#ifdef USEMAGICK
    magick_initialized = false;
#endif
}

template <typename T> 
Matrix<T>::Matrix( uint width, uint height /*= 1*/, uint depth /*= 1*/){
    uint x, y;
    
    this->width  = width;
    this->height = height;
    this->depth  = depth;
    
    this->data = (T ***)malloc( width * sizeof(T **) );
    for( x = 0; x < width; x++ ){
        this->data[x] = (T **)malloc( height * sizeof(T *) );
    }
    for( x = 0; x < width; x++ ){
        for( y = 0; y < height; y++ ){
            this->data[x][y] = (T *)malloc( depth * sizeof(T) );
        }
    }
    
    this->allocated = true;
    
#ifdef USEMAGICK
    magick_initialized = false;
#endif
}

template <typename T> 
Matrix<T>::Matrix( Matrix& m ){
    this->width  = 0;
    this->height = 0;
    this->depth  = 0;
    
    assign( m );
    
    this->allocated = true;
    
#ifdef USEMAGICK
    magick_initialized = false;
#endif
}

template <typename T> 
Matrix<T>::Matrix( T *rawdata, uint width, uint height, uint depth ){
    uint x, y, z, invz;
    
    this->width  = width;
    this->height = height;
    this->depth  = depth;
    
    this->data = (T ***)malloc( width * sizeof(T **) );
    for( x = 0; x < width; x++ ){
        this->data[x] = (T **)malloc( height * sizeof(T *) );
    }
    for( x = 0; x < width; x++ ){
        for( y = 0; y < height; y++ ){
            this->data[x][y] = (T *)malloc( depth * sizeof(T) );
        }
    }
    
    this->allocated = true;
    
#ifdef USEMAGICK
    magick_initialized = false;
#endif
    
    for( y = 0; y < height; y++ ){
        for( x = 0; x < width; x++ ){
            for( z = 0, invz = depth - 1; z < depth; z++, invz-- ){
                data[x][y][z] = rawdata[ (y * width + x) * depth + invz ];
            }
        }
    }
}

#ifdef USEMAGICK
template <typename T> 
Matrix<T>::Matrix( const char *filename ){
    this->width     = 0;
    this->height    = 0;
    this->depth     = 0;
    this->data      = NULL;
    this->allocated = false;
    
    magick_initialized = false;
    load(filename);
}
#endif

template <typename T> 
Matrix<T>::~Matrix(){
    uint x, y;
    
    for( x = 0; x < width; x++ ){
        for( y = 0; y < height; y++ ){
            free(data[x][y]);
        }
        free(data[x]);
    }
    free(data);
    
    this->allocated = false;
}

#ifdef USEMAGICK
template <typename T> 
inline Matrix<T>& Matrix<T>::load( const char *filename ){
    ExceptionInfo exception;
    Image        *image;
    ImageInfo    *info;
    unsigned char * row = NULL;
    uint x, y, z;
    
    if( magick_initialized == false ){
        magick_initialized = true;
        InitializeMagick(".");
    }
    GetExceptionInfo(&exception);
    
    info = CloneImageInfo(NULL);
    strcpy( info->filename, filename );
    
    image = ReadImage( info, &exception );
    if( image && exception.severity == UndefinedException ){
        this->resize( image->columns, image->rows,  (image->colorspace == 1 ? 1 : 3) );
        row = (unsigned char *)calloc( 1, depth * width );
        
        NormalizeImage(image);
        for( y = 0; y < height; y++ ){
            ExportImagePixels( image, 0, y, width, 1, (image->colorspace == 1 ? "I" : "RGB"), CharPixel, row, &exception );
            for( x = 0; x < width; x++ ){
                for( z = 0; z < depth; z++ ){
                    data[x][y][z] = (T)row[x * depth + z];
                }
            }
        }
        
        free(row);
        DestroyImage(image);
        DestroyImageInfo(info);
        DestroyExceptionInfo(&exception);
    }
    else{
        if( row != NULL ){
            free(row);
        }
        DestroyImageInfo(info);
        DestroyExceptionInfo(&exception);
        throw MSPException( __FILE__, __LINE__, "%s .", exception.description );
    }
    
    return *this;
}

template <typename T> 
inline Matrix<T>& Matrix<T>::save( const char *filename ){
    ExceptionInfo exception;
    Image        *image;
    ImageInfo    *info;
    unsigned char * row = (unsigned char *)calloc( 1, depth * width );
    uint x, y, z;
    
    if( magick_initialized == false ){
        magick_initialized = true;
        InitializeMagick(".");
    }
    GetExceptionInfo(&exception);
    
    info = CloneImageInfo(NULL);
    info->depth = depth * 8;
    
    image = ConstituteImage( width, height, (depth > 1 ? "RGB" : "I"), CharPixel, data, &exception );
    strcpy( image->filename, filename );
    image->depth = depth * 8;
    
    image->colorspace = (depth > 1 ? RGBColorspace : GRAYColorspace);
    
    if( image && exception.severity == UndefinedException ){
        for( y = 0; y < height; y++ ){
            for( x = 0; x < width; x++ ){
                for( z = 0; z < depth; z++ ){
                    row[x * depth + z] = (unsigned  char)data[x][y][z];
                }
            }
            ImportImagePixels( image, 0, y, height, 1, (depth > 1 ? "RGB" : "I"), CharPixel, row );
        }
        
        WriteImage( info, image );
        free(row);
        DestroyImage(image);
        DestroyImageInfo(info);
        DestroyExceptionInfo(&exception);
    }
    else{
        free(row);
        DestroyImageInfo(info);
        DestroyExceptionInfo(&exception);
        throw MSPException( __FILE__, __LINE__, "%s .", exception.description );
    }
    
    return *this;
}
#endif

template <typename T> 
inline uint Matrix<T>::size(){
    return width * height * depth * sizeof(T);
}

template <typename T> 
inline Matrix<T>& Matrix<T>::set( T v, ... ){   
    #ifdef MSPDIMCHECK
    if( height != 1 || depth != 1 ){
        throw MSPException( __FILE__, __LINE__, "Matrix size mismatch (%dx%dx%d) .", width, height, depth );
    }
    #endif 
    
    va_list   list;
    vector<T> values;
    uint      i;
   
    values.push_back(v);
    
    va_start( list, v );
    for( i = 1; i < width; i++ ){
        values.push_back( va_arg( list, T ) );
    }
    va_end( list );
   
    for( i = 0; i < width; i++ ){
        data[i][0][0] = values[i];
    }
    
    return *this;
}

template <typename T> 
inline Matrix<T>& Matrix<T>::fill( T value ){
    uint x, y, z;
    
    for( x = 0; x < width; x++ ){
        for( y = 0; y < height; y++ ){
            for( z = 0; z < depth; z++ ){
                data[x][y][z] = value;
            }
        }
    }
    
    return *this;
}

template <typename T> 
inline Matrix<T>& Matrix<T>::fill( uint x, uint y, uint width, uint height, T value ){
    uint xs, ys, z, xe = x + width, ye = y + height;
    
    for( xs = x; xs < xe; xs++ ){
        for( ys = y; ys < ye; ys++ ){
            for( z = 0; z < depth; z++ ){
                data[xs][ys][z] = value;
            }
        }
    }
    
    return *this;
}

template <typename T> 
inline Matrix<T>& Matrix<T>::line( uint x1, uint y1, uint x2, uint y2, T * color ){
    int i, dx, dy, sdx, sdy, dxabs, dyabs, x, y, z, px, py;
    #define sgn(x) (x < 0 ? -1 : (x > 0 ? 1 : 0))
    dx    = x2 - x1 ;      /* the horizontal distance of the line */
    dy    = y2 - y1 ;      /* the vertical distance of the line */
    dxabs = (dx >= 0 ? dx : dx * -1);
    dyabs = (dy >= 0 ? dy : dy * -1);
    sdx   = sgn(dx);
    sdy   = sgn(dy);
    x     = dyabs >> 1;
    y     = dxabs >> 1;
    px    = x1;
    py    = y1;
    
    memcpy( data[px][py], color, depth * sizeof(T) );
    /* the line is more horizontal than vertical */
    if( dxabs >= dyabs ){
        for( i = 0; i < dxabs; i++ ){
            y += dyabs;
            if( y >= dxabs ){
                y  -= dxabs;
                py += sdy;
            }
            px += sdx;
            memcpy( data[px][py], color, depth * sizeof(T) );
        }
    }
    /* the line is more vertical than horizontal */
    else{
        for( i = 0; i < dyabs; i++ ){
            x += dxabs;
            if( x >= dyabs ){
                x  -= dyabs;
                px += sdx;
            }
            py += sdy;
            memcpy( data[px][py], color, depth * sizeof(T) );
        }
    }
    
    return *this;
}

template <typename T> 
inline Matrix<T>& Matrix<T>::rect( uint x, uint y, uint width, uint height, T * color ){
    line( x, y, x + width, y, color );
    line( x + width, y, x + width, y + height, color );
    line( x + width, y + height, x, y + height, color );
    line( x, y + height, x, y, color );
    
    return *this;
}
        
template <typename T> 
inline Matrix<T>& Matrix<T>::clear(){
    return fill( (T)0 );
}

template <typename T> 
inline Matrix<T>& Matrix<T>::clear( uint x, uint y, uint width, uint height ){
    return fill( x, y, width, height, (T)0 );
}

template <typename T> 
inline Matrix<T>& Matrix<T>::assign( Matrix& m ){
    uint x, y, z;
    
    resize( m.width, m.height, m.depth );

    for( x = 0; x < width; x++ ){
        for( y = 0; y < height; y++ ){
            for( z = 0; z < depth; z++ ){
                data[x][y][z] = m[x][y][z];
            }
        }
    }
    
    return *this;
}

template <typename T> 
inline Matrix<T>& Matrix<T>::assign( const Matrix& m ){
    uint x, y, z;
    
    resize( m.width, m.height, m.depth );

    for( x = 0; x < width; x++ ){
        for( y = 0; y < height; y++ ){
            for( z = 0; z < depth; z++ ){
                data[x][y][z] = m[x][y][z];
            }
        }
    }
    
    return *this;
}

template <typename T> 
inline T ** Matrix<T>::operator [] ( uint x ){
    return data[x];
}

template <typename T> 
inline T& Matrix<T>::operator () ( uint x, uint y /*= 0*/, uint z /*= 0*/ ){
    return data[x][y][z];
}

template <typename T> 
inline Matrix<T>& Matrix<T>::operator = ( Matrix& m ){
    assign(m);
    return *this;
}

template <typename T> 
inline Matrix<T>& Matrix<T>::operator = ( Matrix * m ){
    assign(*m);
    return *this;
}

template <typename T> 
inline Matrix<T>& Matrix<T>::operator += ( Matrix& m ){
    #ifdef MSPDIMCHECK
    if( m.width != this->width || m.height != this->height || m.depth != this->depth ){
        throw MSPException( __FILE__, __LINE__, "Matrix size mismatch (%dx%dx%d - %dx%dx%d) .", width, height, depth, m.width, m.height, m.depth );
    }
    #endif 
    
    uint x, y, z;
    
    for( x = 0; x < width; x++ ){
        for( y = 0; y < height; y++ ){
            for( z = 0; z < depth; z++ ){
                data[x][y][z] += m[x][y][z];
            }
        }
    }
    
    return *this;
}

template <typename T> 
inline Matrix<T>& Matrix<T>::operator -= ( Matrix& m ){
    #ifdef MSPDIMCHECK
    if( m.width != this->width || m.height != this->height || m.depth != this->depth ){
        throw MSPException( __FILE__, __LINE__, "Matrix size mismatch (%dx%dx%d - %dx%dx%d) .", width, height, depth, m.width, m.height, m.depth );
    }
    #endif
    
    uint x, y, z;
    
    for( x = 0; x < width; x++ ){
        for( y = 0; y < height; y++ ){
            for( z = 0; z < depth; z++ ){
                data[x][y][z] -= m[x][y][z];
            }
        }
    }
    
    return *this;
}

template <typename T> 
inline Matrix<T>& Matrix<T>::operator *= ( Matrix& m ){
    #ifdef MSPDIMCHECK
    if( m.height != this->height || m.depth != this->depth ){
        throw MSPException( __FILE__, __LINE__, "Matrix size mismatch (%dx%dx%d - %dx%dx%d) .", width, height, depth, m.width, m.height, m.depth );
    }
    #endif
    
    uint x, y, y2, z;
    Matrix mul( this->width, m.height, this->depth );
    
    for( x = 0; x < this->width; x++ ){
        for( y = 0; y < m.height; y++ ){
            for( y2 = 0; y2 < this->height; y2++ ){
                for( z = 0; z < depth; z++ ){
                    mul[x][y][z] += data[x][y2][z] * m[y2][y][z];
                }
            }
        }
    }
    
    assign(mul);
    
    return *this;
}

template <typename T> 
inline Matrix<T>& Matrix<T>::operator /= ( Matrix& m ){
    #ifdef MSPDIMCHECK
    if( m.width != this->width || m.height != this->height || m.depth != this->depth ){
        throw MSPException( __FILE__, __LINE__, "Matrix size mismatch (%dx%dx%d - %dx%dx%d) .", width, height, depth, m.width, m.height, m.depth );
    }
    #endif
    
    uint x, y, z;
    
    for( x = 0; x < width; x++ ){
        for( y = 0; y < height; y++ ){
            for( z = 0; z < depth; z++ ){
                data[x][y][z] /= m[x][y][z];
            }
        }
    }
    
    return *this;
}

template <typename T> 
inline Matrix<T>& Matrix<T>::operator += ( T s ){  
    uint x, y, z;
    
    for( x = 0; x < width; x++ ){
        for( y = 0; y < height; y++ ){
            for( z = 0; z < depth; z++ ){
                data[x][y][z] += s;
            }
        }
    }
    
    return *this;
}

template <typename T> 
inline Matrix<T>& Matrix<T>::operator -= ( T s ){  
    uint x, y, z;
    
    for( x = 0; x < width; x++ ){
        for( y = 0; y < height; y++ ){
            for( z = 0; z < depth; z++ ){
                data[x][y][z] -= s;
            }
        }
    }
    
    return *this;
}

template <typename T> 
inline Matrix<T>& Matrix<T>::operator *= ( T s ){
    uint x, y, z;
    
    for( x = 0; x < width; x++ ){
        for( y = 0; y < height; y++ ){
            for( z = 0; z < depth; z++ ){
                data[x][y][z] *= s;
            }
        }
    }
    
    return *this;
}

template <typename T> 
inline Matrix<T>& Matrix<T>::operator /= ( T s ){
    uint x, y, z;
    
    for( x = 0; x < width; x++ ){
        for( y = 0; y < height; y++ ){
            for( z = 0; z < depth; z++ ){
                data[x][y][z] /= s;
            }
        }
    }
    
    return *this;
}

template <typename T> 
inline Matrix<T>& operator + ( Matrix<T>& a, Matrix<T>& b ){
    Matrix<T> add(a);
    
    add += b;
    
    return add;
}

template <typename T> 
inline Matrix<T>& operator - ( Matrix<T>& a, Matrix<T>& b ){
    Matrix<T> sub(a);
    
    sub -= b;
    
    return sub;
}

template <typename T> 
inline Matrix<T>& operator * ( Matrix<T>& a, Matrix<T>& b ){
    Matrix<T> mul(a);
    
    mul *= b;
    
    return mul;
}

template <typename T> 
inline Matrix<T>& operator / ( Matrix<T>& a, Matrix<T>& b ){
    Matrix<T> div(a);
    
    div /= b;
    
    return div;
}

template <typename T> 
inline T Matrix<T>::average( T *values, uint size ){
    uint i;
    T    sum = 0;
    
    for( i = 0; i < size; i++ ){
        sum += values[i];
    }
    return sum / size;
}

template <typename T> 
inline T Matrix<T>::bigger( T *values, uint size ){
    uint i;
    T    bigger = values[0];
    
    for( i = 1; i < size; i++ ){
        if( values[i] > bigger ){
            bigger = values[i];
        }
    }
    return bigger;
}

template <typename T> 
inline T Matrix<T>::smaller( T *values, uint size ){
    uint i;
    T    smaller = values[0];
    
    for( i = 1; i < size; i++ ){
        if( values[i] < smaller ){
            smaller = values[i];
        }
    }
    return smaller;
}

template <typename T> 
inline void Matrix<T>::resize_chop( uint width, uint height, uint depth ){
    int x, y;
    if( this->allocated == true ){
        this->data = (T ***)realloc( this->data, width * sizeof(T **) );
        for( x = 0; x < width; x++ ){
            this->data[x] = (T **)realloc( this->data[x], height * sizeof(T *) );
        }
        for( x = 0; x < width; x++ ){
            for( y = 0; y < height; y++ ){
                this->data[x][y] = (T *)realloc( this->data[x][y], depth * sizeof(T) );
            }
        }
    }
    else{
        this->data = (T ***)malloc( width * sizeof(T **) );
        for( x = 0; x < width; x++ ){
            this->data[x] = (T **)malloc( height * sizeof(T *) );
        }
        for( x = 0; x < width; x++ ){
            for( y = 0; y < height; y++ ){
                this->data[x][y] = (T *)malloc( depth * sizeof(T) );
            }
        }
    }
    
    this->width     = width;
    this->height    = height;
    this->depth     = depth;
    this->allocated = true;
}

template <typename T> 
inline Matrix<T>& Matrix<T>::setrow( Matrix& v, uint row ){
    #ifdef MSPDIMCHECK
    if( row < 0 || row > height ){
        throw MSPException( __FILE__, __LINE__, "Invalid row index %d .", row );
    }
    if( v.width != width || v.depth != depth ){
        throw MSPException( __FILE__, __LINE__, "Invalid row vector size (%dx%dx%d) .", v.width, v.height, v.depth );
    }
    #endif
    
    uint x, z;
    for( x = 0; x < width; x++ ){
        for( z = 0; z < depth; z++ ){
            data[x][row][z] = v(x, 0, z);
        }
    }
    
    return *this;
}

template <typename T> 
inline Matrix<T>& Matrix<T>::setcol( Matrix& v, uint col ){
    #ifdef MSPDIMCHECK
    if( col < 0 || col > width ){
        throw MSPException( __FILE__, __LINE__, "Invalid column index %d .", col );
    }
    if( v.width != height || v.depth != depth ){
        throw MSPException( __FILE__, __LINE__, "Invalid column vector size (%dx%dx%d) .", v.width, v.height, v.depth );
    }
    #endif
    
    uint y, z;
    for( y = 0; y < height; y++ ){
        for( z = 0; z < depth; z++ ){
            data[col][y][z] = v(y, 0, z);
        }
    }
    
    return *this;
}

template <typename T> 
inline Matrix<T>& Matrix<T>::flatten( Matrix& flat ){
    #ifdef MSPDIMCHECK
    if( height != 1 ){
        throw MSPException( __FILE__, __LINE__, "Flatten is possible only on a Nx1xM matrix ." );
    }
    #endif
    
    flat.resize( width * depth, 1, 1 );
    
    uint x, z, i = 0;
    
    for( x = 0; x < width; x++ ){
        for( z = 0; z < depth; z++ ){
            flat( i++ ) = data[x][0][z];
        }
    }
    
    return *this;
}

template <typename T> 
inline Matrix<T>& Matrix<T>::flatten(){
    Matrix<T> flat;
    this->flatten(flat);
    assign(flat);
    
    return *this;
}

template <typename T> 
inline void Matrix<T>::resize_average( uint width, uint height, uint depth ){
    uint x, y, z;
    T    average;
    
    for( x = 0; x < this->width; x++ ){
        for( y = 0; y < this->height; y++ ){
            average = this->average( this->data[x][y], this->depth );
            for( z = 0; z < this->depth; z++ ){
                this->data[x][y][z] = average;
            }
        }
    }
    
    resize_chop( width, height, depth );
}

template <typename T> 
inline void Matrix<T>::resize_bigger( uint width, uint height, uint depth ){
    uint x, y, z;
    T    bigger;
    
    for( x = 0; x < this->width; x++ ){
        for( y = 0; y < this->height; y++ ){
            bigger = this->bigger( this->data[x][y], this->depth );
            for( z = 0; z < this->depth; z++ ){
                this->data[x][y][z] = bigger;
            }
        }
    }
    
    resize_chop( width, height, depth );
}

template <typename T> 
inline void Matrix<T>::resize_smaller( uint width, uint height, uint depth ){
    uint x, y, z;
    T    smaller;
    
    for( x = 0; x < this->width; x++ ){
        for( y = 0; y < this->height; y++ ){
            smaller = this->smaller( this->data[x][y], this->depth );
            for( z = 0; z < this->depth; z++ ){
                this->data[x][y][z] = smaller;
            }
        }
    }
    
    resize_chop( width, height, depth );
}

template <typename T> 
inline Matrix<T>& Matrix<T>::resize( uint width, uint height, uint depth, RESIZEMETHOD method /*= TO_CHOP*/){
    if( width != this->width || height != this->height || depth != this->depth ){
        switch(method){
            case TO_CHOP    : resize_chop( width, height, depth );    break;
            case TO_AVERAGE : resize_average( width, height, depth ); break;
            case TO_BIGGER  : resize_bigger( width, height, depth );  break;
            case TO_SMALLER : resize_smaller( width, height, depth ); break;
        }
    }
    
    return *this;
}

template <typename T> 
inline void Matrix<T>::mask_ex_outer( Matrix& mask, T trigger ){
    #ifdef MSPDIMCHECK
    if( mask.width != this->width || mask.height != this->height ){
        throw MSPException( __FILE__, __LINE__, "Matrix size mismatch (%dx%dx%d - %dx%dx%d) .", width, height, depth, mask.width, mask.height, mask.depth );
    }
    #endif
    
    uint x, y, z;
    
    for( x = 0; x < width; x++ ){
        for( y = 0; y < height; y++ ){
            for( z = 0; z < depth; z++ ){
                if( mask[x][y][0] != trigger ){
                    data[x][y][z] = 0;
                }
            }
        }
    }
}

template <typename T> 
inline void Matrix<T>::mask_ex_inner( Matrix& mask, T trigger ){
    #ifdef MSPDIMCHECK
    if( mask.width != this->width || mask.height != this->height ){
        throw MSPException( __FILE__, __LINE__, "Matrix size mismatch (%dx%dx%d - %dx%dx%d) .", width, height, depth, mask.width, mask.height, mask.depth );
    }
    #endif
    
    uint x, y, z;
    
    for( x = 0; x < width; x++ ){
        for( y = 0; y < height; y++ ){
            for( z = 0; z < depth; z++ ){
                if( mask[x][y][0] == trigger ){
                    data[x][y][z] = 0;
                }
            }
        }
    }
}

template <typename T> 
inline void Matrix<T>::mask_ex_outer( Matrix& mask ){
    #ifdef MSPDIMCHECK
    if( mask.width != this->width || mask.height != this->height ){
        throw MSPException( __FILE__, __LINE__, "Matrix size mismatch (%dx%dx%d - %dx%dx%d) .", width, height, depth, mask.width, mask.height, mask.depth );
    }
    #endif
    
    uint x, y, z;
    
    for( x = 0; x < width; x++ ){
        for( y = 0; y < height; y++ ){
            for( z = 0; z < depth; z++ ){
                if( mask(x,y) <= 0 ){
                    data[x][y][z] = 0;
                }
            }
        }
    }
}

template <typename T> 
inline void Matrix<T>::mask_ex_inner( Matrix& mask ){
    #ifdef MSPDIMCHECK
    if( mask.width != this->width || mask.height != this->height ){
        throw MSPException( __FILE__, __LINE__, "Matrix size mismatch (%dx%dx%d - %dx%dx%d) .", width, height, depth, mask.width, mask.height, mask.depth );
    }
    #endif
    
    uint x, y, z;
    
    for( x = 0; x < width; x++ ){
        for( y = 0; y < height; y++ ){
            for( z = 0; z < depth; z++ ){
                if( mask(x,y) > 0 ){
                    data[x][y][z] = 0;
                }
            }
        }
    }
}

template <typename T> 
inline void Matrix<T>::orientation_lowpass( Matrix<double>& theta, Matrix<double>& orientation, uint filtersize, uint width, uint height ){
    uint fsize = filtersize * 2 + 1,
         dwidth = width - fsize,
         dheight = height - fsize,
            i, j, x, y;
    Matrix<double> filter( fsize, fsize ),
            phix( width, height ),
            phiy( width, height ),
            phi2x( width, height ),
            phi2y( width, height );
    double nx, ny;
    
    for( y = 0; y < height; y++ ){
        for( x = 0; x < width; x++ ){
            phix(x,y) = cos(theta(x,y));
            phiy(x,y) = sin(theta(x,y));
        }
    }
    
    nx = 0.0;
    for( j = 0; j < fsize; j++ ){
        for( i = 0; i < fsize; i++ ){
            nx += (filter(i,j) = 1.0);
        }
    }
    if( nx > 1.0 ){
        for( j = 0; j < fsize; j++ ){
            for( i = 0; i < fsize; i++ ){
                filter(i,j) /= nx;
            }
        }
    }
    
    for( y = 0; y < dheight; y++ ){
        for( x = 0; x < dwidth; x++ ){
            nx = 0.0;
            ny = 0.0;
            for( j = 0; j < fsize; j++ ){
                for( i = 0; i < fsize; i++ ){
                    nx += filter(i,j) * phix(x + i,y + j);
                    ny += filter(i,j) * phiy(x + i,y + j);
                }
            }
            phi2x(x,y) = nx;
            phi2y(x,y) = ny;
        }
    }
    
    for( y = 0; y < dheight; y++ ){
        for( x = 0; x < dwidth; x++ ){
            orientation(x,y) = atan2( phi2y(x,y), phi2x(x,y) ) * 0.5;
        }
    }
}

template <typename T> 
inline double Matrix<T>::gabor( int x, int y, double phi, double frequency, double radius ){
    double dr = 1.0 / radius,
            x2, y2,
            sphi,
            cphi;
    
    phi += H_M_PI;
    sphi = sin(phi);
    cphi = cos(phi);
    x2   = -x * sphi + y * cphi;
    y2   =  x * cphi + y * sphi;
    
    return exp(-0.5 * (x2*x2*dr + y2*y2*dr)) * cos(D_M_PI*x2*frequency);
}

template <typename T> 
inline uint Matrix<T>::entropy_empty_neighbours( Matrix& entropy, uint x, uint y, uint region ){
    uint xr, yr, 
         xs = x - region,
         xe = x + region,
         ys = y - region,
         ye = y + region,
         n = 0;
	
    for( xr = xs; xr < xe; xr++ ){
        for( yr = ys; yr < ye; yr++ ){
            if( !entropy(xr,yr) ){
                n++;	
            }
        }	
    }
    return n;
}

template <typename T> 
inline double Matrix<T>::delta( T total, T value ){
    T a = total, b = value;

    if( total < value ){
        a = value;
        b = total;
    }

    return ((double)(a - b) / (double)b) * 100.0;
}

template <typename T> 
inline Matrix<T>& Matrix<T>::mask( Matrix& mask, T trigger, MASKMETHOD method /*= MASK_EXCLUDE_OUTER*/ ){
    switch(method){
        case MASK_EXCLUDE_OUTER : mask_ex_outer( mask, trigger ); break;
        case MASK_EXCLUDE_INNER : mask_ex_inner( mask, trigger ); break;
    }
    
    return *this;
}

template <typename T> 
inline Matrix<T>& Matrix<T>::mask( Matrix& mask, MASKMETHOD method /*= MASK_EXCLUDE_OUTER*/ ){
    switch(method){
        case MASK_EXCLUDE_OUTER : mask_ex_outer( mask ); break;
        case MASK_EXCLUDE_INNER : mask_ex_inner( mask ); break;
    }
    
    return *this;
}

template <typename T> 
inline Matrix<T>& Matrix<T>::edges( Matrix& edges, uint threshold ){
    int x, y, max = 0, min = 0,
            dx, dy, z;
    double a, b;
    Matrix<T> field(*this),
            mask( width, height, 1 );
    
    /* gray scale conversion */
    if( field.depth != 1 ){
        field.resize( width, height, 1, TO_AVERAGE );
    }
    /* helper macro */
#define PIX(x,y) field[x][y][0]
    /* create the mask with the Prewitt algorithm */
    for( x = field.width - 2; x > 0; x-- ){
        for( y = field.height - 2; y > 0; y-- ){
            dx = (PIX( x - 1, y + 1 ) + PIX( x, y + 1 ) + PIX( x + 1, y + 1 )) -
                 (PIX( x - 1, y - 1 ) + PIX( x, y - 1 ) + PIX( x + 1, y - 1 ));
            dy = (PIX( x + 1, y - 1 ) + PIX( x + 1, y ) + PIX( x + 1, y + 1 )) -
                 (PIX( x - 1, y - 1 ) + PIX( x - 1, y ) + PIX( x - 1, y + 1 ));
            
            z   = (int)(sqrt( dx*dx + dy*dy ) / 3 );
            max = z > max ? z : max;
            min = z < min ? z : min;
            /* finally set the mask */
            mask[x][y][0] = z;
        }
    }
#undef PIX
    
    a = 255.0f / (max - min);
    b = a * min;
    
    /* apply the mask to the edges matrix */
    edges.resize( width, height, 1 );
    for( x = field.width - 1; x >= 0; x-- ){
        for( y = field.height - 1; y >= 0; y-- ){
            edges[x][y][0] = (a * mask[x][y][0] + b) > threshold ? 255 : 0;
        }
    }
    
    return *this;
}

template <typename T> 
inline Matrix<T>& Matrix<T>::edges( uint threshold ){
    Matrix<T> edges;
    this->edges( edges, threshold );
    assign(edges);
    
    return *this;
}

template <typename T> 
inline Matrix<T>& Matrix<T>::thin( Matrix& thinned ){
    uint x, y,
         xp, xm,
         yp, ym,
         changed = 1;
    
    
    /* gray scale conversion */
    #ifdef MSPDIMCHECK
    if( depth != 1 ){
        throw MSPException( __FILE__, __LINE__, "Thinning is possible only on grayscale images (MxNx1 Matrix) ." );
    }
    else{
    #endif
        thinned.assign(*this);
        
        /* helper macro */
#define PIX(x,y) thinned[x][y][0]
        /* compute hit-and-miss thinning */
        while(changed){
            changed = 0;
            for( y = 1; y < height - 1; y++ ){
                yp = y + 1;
                ym = y - 1;
                for( x = 1; x < width - 1; x++ ){
                    if( PIX(x, y) ){
                        xp = x + 1;
                        xm = x - 1;
                        if( PIX(xm, ym) == 0 && PIX(x, ym) == 0 && PIX(xp, ym) == 0 &&
                                PIX(xm, yp) != 0 && PIX(x, yp) != 0 && PIX(xp, yp) != 0 ){
                            PIX(x, y) = 0;
                            changed = 1;
                        }
                        if( PIX(xm, ym) != 0 && PIX(x, ym) != 0 && PIX(xp, ym) != 0 &&
                                PIX(xm, yp) == 0 && PIX(x, yp) == 0 && PIX(xp, yp) == 0 ){
                            PIX(x, y) = 0;
                            changed = 1;
                        }
                        if( PIX(xm, ym) == 0 && PIX(xm, y) == 0 && PIX(xm, yp) == 0 &&
                                PIX(xp, ym) != 0 && PIX(xp, y) != 0 && PIX(xp, yp) != 0 ){
                            PIX(x, y) = 0;
                            changed = 1;
                        }
                        if( PIX(xm, ym) != 0 && PIX(xm, y) != 0 && PIX(xm, yp) != 0 &&
                                PIX(xp, ym) == 0 && PIX(xp, y) == 0 && PIX(xp, yp) == 0 ){
                            PIX(x, y) = 0;
                            changed = 1;
                        }
                        
                        if( PIX(x, ym) == 0 && PIX(xp, ym) == 0 && PIX(xp, y) == 0 &&
                                PIX(xm, y) != 0 && PIX(x, yp) != 0 ){
                            PIX(x, y) = 0;
                            changed = 1;
                        }
                        
                        if( PIX(xm, ym) == 0 && PIX(x, ym) == 0 && PIX(xm, y) == 0 &&
                                PIX(xp, y) != 0 && PIX(x, yp) != 0 ){
                            PIX(x, y) = 0;
                            changed = 1;
                        }
                        if( PIX(xm, yp) == 0 && PIX(xm, y) == 0 && PIX(x, yp) == 0 &&
                                PIX(xp, y) != 0 && PIX(x, ym) != 0 ){
                            PIX(x, y) = 0;
                            changed = 1;
                        }
                        
                        if( PIX(xp, yp) == 0 && PIX(xp, y) == 0 && PIX(x, yp) == 0 &&
                                PIX(xm, y) != 0 && PIX(x, ym) != 0 ){
                            PIX(x, y) = 0;
                            changed = 1;
                        }
                    }
                }
            }
        }
#undef PIX
    #ifdef MSPDIMCHECK
    }
    #endif
    
    return *this;
}

template <typename T> 
inline Matrix<T>& Matrix<T>::thin(){
    Matrix<T> thinned;
    this->thin( thinned );
    assign(thinned);
    
    return *this;
}

template <typename T> 
inline Matrix<T>& Matrix<T>::orientation( Matrix<double>& orientation, uint blocksize /*= DEFAULT_ORIENT_BSIZE*/, int filtersize /*= DEFAULT_ORIENT_FSIZE*/ ){
    uint i, j, u, v, x, y, xi, yi,
            s = blocksize * 2 + 1,
            st = blocksize + 1,
            dheight = height - blocksize - 1,
            dwidth  = width - blocksize - 1;
    double nx, ny;
    Matrix<double> dx( s, s ),
                   dy( s, s ),
                   theta( width, height );
    T bi;
    
    orientation.resize( width, height, 1 );
    for( y = st; y < dheight; y++ ){
        for( x = st; x < dwidth; x++ ){
            for( j = 0; j < s; j++ ){
                yi = y + j - blocksize;
                for( i = 0; i < s; i++ ){
                    xi = x + i - blocksize;
                    bi = data[xi][yi][0];
                    dx(j,i) = (double)(bi - data[xi - 1][yi][0]);
                    dy(j,i) = (double)(bi - data[xi][yi - 1][0]);
                }
            }
            
            nx = 0.0;
            ny = 0.0;
            for( v = 0; v < s; v++ ){
                for( u = 0; u < s; u++ ){
                    nx += 2 * dx(v,u) * dy(v,u);
                    ny += dx(v,u) * dx(v,u) - dy(v,u) * dy(v,u);
                }
            }
            
            if( filtersize > 0 ){
                theta(x,y) = atan2(nx, ny);
            }
            else{
                orientation(x,y) = atan2(nx, ny) * 0.5;
            }
        }
    }
    
    if( filtersize > 0 ){
        this->orientation_lowpass( theta, orientation, filtersize, width, height );
    }
    
    return *this;
}

template <typename T> 
inline Matrix<T>& Matrix<T>::orientation( uint blocksize /*= DEFAULT_ORIENT_BSIZE*/, int filtersize /*= DEFAULT_ORIENT_FSIZE*/ ){
    Matrix<double> orientation;
    this->orientation( orientation, blocksize, filtersize );
    assign(orientation);
    
    return *this;
}

template <typename T>
inline Matrix<T>& Matrix<T>::frequency( Matrix<double>& frequency, Matrix<double>& orientation ){
    double  dir    = 0.0,
            cosdir = 0.0,
            sindir = 0.0,
            peak_freq,
            Xsig[BLOCK_L],
            pmin, pmax;
    int x, y, u, v, d, k,
            peak_pos[BLOCK_L],
            peak_cnt,
        dheight = height - BLOCK_L2,
        dwidth  = width - BLOCK_L2;
    Matrix<double> out( width, height );
    
    frequency.resize( width, height, 1 );
    
    for( y = BLOCK_L2; y < dheight; y++ ){
        for( x = BLOCK_L2; x < dwidth; x++ ){
            dir    = orientation(x + BLOCK_W2, y + BLOCK_W2);
            cosdir = -sin(dir);
            sindir = cos(dir);
            
            for( k = 0; k < BLOCK_L; k++ ){
                Xsig[k] = 0.0;
                for( d = 0; d < BLOCK_W; d++ ){
                    u = (int)(x + (d - BLOCK_W2) * cosdir + (k - BLOCK_L2) * sindir );
                    v = (int)(y + (d - BLOCK_W2) * sindir - (k - BLOCK_L2) * cosdir );
                    
                    if( u < 0 ){
                        u = 0;
                    }
                    else if( u > width - 1 ){
                        u = width - 1;
                    }
                    if( v < 0 ){
                        v = 0;
                    }
                    else if( v > height - 1 ){
                        v = height - 1;
                    }
                    Xsig[k] += data[u][v][0];
                }
                Xsig[k] /= BLOCK_W;
            }
            peak_cnt = 0;
            pmax     = pmin = Xsig[0];
            for( k = 1; k < BLOCK_L; k++ ){
                pmin = (pmin > Xsig[k]) ? Xsig[k] : pmin;
                pmax = (pmax < Xsig[k]) ? Xsig[k] : pmax;
            }
            if( (pmax - pmin) > 64.0 ){
                for( k = 1; k < BLOCK_L - 1; k++ ){
                    if( (Xsig[k - 1] < Xsig[k]) && (Xsig[k] >= Xsig[k + 1]) ){
                        peak_pos[peak_cnt++] = k;
                    }
                }
            }
            peak_freq = 0.0;
            if( peak_cnt >= 2 ){
                for( k = 0; k < peak_cnt - 1; k++ ){
                    peak_freq += peak_pos[k + 1] - peak_pos[k];
                }
                peak_freq /= peak_cnt - 1;
            }
            
            if( peak_freq > 30.0 ){
                out(x, y) = 0.0;
            }
            else if( peak_freq < 2.0 ){
                out(x, y) = 0.0;
            }
            else{
                out(x, y) = 1.0 / peak_freq;
            }
        }
    }
    
    for( y = BLOCK_L2; y < dheight; y++ ){
        for( x = BLOCK_L2; x < dwidth; x++ ){
            if( out(x, y) < EPSILON ){
                if( out(x,y - 1) > EPSILON ){
                    out(x, y) = out(x, y - 1);
                }
                else if( out(x - 1,y) > EPSILON ){
                    out(x, y) = out(x - 1, y);
                }
            }
        }
    }
    
    for( y = BLOCK_L2; y < dheight; y++ ){
        for( x = BLOCK_L2; x < dwidth; x++ ){
            peak_freq = 0.0;
            for( v = -LPSIZE; v <= LPSIZE; v++ ){
                for( u = -LPSIZE; u <= LPSIZE; u++ ){
                    peak_freq += out(x + u,y + v);
                }
            }
            frequency(x, y) = peak_freq * LPFACTOR;
        }
    }
    
    return *this;
}

template <typename T> 
inline Matrix<T>& Matrix<T>::frequency( Matrix<double>& orientation ){
    Matrix<T> frequency;
    this->frequency( frequency, orientation );
    assign(frequency);
    
    return *this;
}

template <typename T> 
inline Matrix<T>& Matrix<T>::gabor( Matrix& gabor, Matrix<double>& orientation, Matrix<double>& frequency, double radius /*= DEFAULT_GABOR_RADIUS*/ ){
    int dheight = height - WG2,
        dwidth  = width - WG2,
        x, y, xx, yy, dy;
    double sum, freq, phi;

    gabor.resize( width, height, depth );
    
    radius = radius * radius;
    
    for( y = WG2; y < dheight; y++ ){
        for( x = WG2; x < dwidth; x++ ){
            sum  = 0.0;
            phi  = orientation(x,y);
            freq = frequency(x,y);
            for( yy = -WG2; yy <= WG2; yy++ ){
                dy  = y - yy;
                for( xx = -WG2; xx <= WG2; xx++ ){
                    sum += this->gabor( xx, yy, phi, freq, radius ) * data[x - xx][dy][0];
                }
            }
            
            if( sum > 255 ){
                sum = 255;
            }
            if( sum < 0 ){
                sum = 0;
            }
            
            gabor(x,y) = (T)sum;
        }
    }
    
    return *this;
}

template <typename T> 
inline Matrix<T>& Matrix<T>::gabor( Matrix<double>& orientation, Matrix<double>& frequency, double radius /*= DEFAULT_GABOR_RADIUS*/ ){
    Matrix<T> gabor;
    this->gabor( gabor, orientation, frequency, radius );
    assign(gabor);
    
    return *this;
}

template <typename T> 
inline Matrix<T>& Matrix<T>::soften( Matrix& soften, uint size /*= DEFAULT_SOFTEN_SIZE*/ ){
    uint x, y, z,
         dheight,
         dwidth;
    int  p, q, s, a;
    T c;
    
    soften.resize( width, height, depth );
    
    s = size / 2;
    a = size * size;
    dheight = height - s;
    dwidth  = width - s;
    for( y = s; y < dheight; y++ ){
        for( x = s; x < dwidth; x++ ){
            for( z = 0; z < depth; z++ ){
                c = (T)0;
                for( q = -s; q <= s; q++ ){
                    for( p = -s; p <= s; p++ ){
                        c += data[x + p][y + q][z];
                    }
                }
                soften[x][y][z] = c / (T)a;
            }
        }
    }
    
    return *this;
}

template <typename T> 
inline Matrix<T>& Matrix<T>::soften( uint size /*= DEFAULT_SOFTEN_SIZE*/ ){
    Matrix<T> soften;
    this->soften( soften, size );
    assign(soften);
    
    return *this;
}

template <typename T> 
inline Matrix<T>& Matrix<T>::entropy( Matrix& entropy, Matrix<double>& frequency, double hvalue /*= 255.0*/, double lthreshold /*= DEFAULT_MIN_FREQ*/, double hthreshold /*= DEFAULT_MAX_FREQ*/ ){
    uint x, y;
    #ifdef MSPDIMCHECK
    if( depth != 1 || frequency.depth != 1 ){
        throw MSPException( __FILE__, __LINE__, "Entropy mask computation is possible only on grayscale images (MxNx1 Matrix) ." );
    }
    #endif
    
    entropy.resize( width, height, depth );
    
    for( y = 0; y < height; y++ ){
        for( x = 0; x < width; x++ ){
            if( frequency(x,y) >= lthreshold && frequency(x,y) <= hthreshold ){
                entropy(x,y) = hvalue;
            }
        }
    }

    T mask = (T)((unsigned int)(hvalue / (T)2));
    
    //for( y = 0; y < 4; y++ ){
    //    entropy.dilate( hvalue, mask );
    //}

    for( y = 0; y < 15; y++ ){
        entropy.erode( (T)0, hvalue, mask );
    }
    return *this;
}

template <typename T> 
inline Matrix<T>& Matrix<T>::entropy( Matrix<double>& frequency, double hvalue /*= 255.0*/, double lthreshold /*= DEFAULT_MIN_FREQ*/, double hthreshold /*= DEFAULT_MAX_FREQ*/ ){
    Matrix<T> entropy;
    this->entropy( entropy, hvalue, frequency, lthreshold, hthreshold );
    assign(entropy);
    
    return *this;
}

template <typename T> 
inline Matrix<T>& Matrix<T>::dilate( Matrix& dilated, T threshold, T mask ){
    uint x, y;
    #ifdef MSPDIMCHECK
    if( depth != 1 ){
        throw MSPException( __FILE__, __LINE__, "Dilatation is possible only on grayscale images (MxNx1 Matrix) ." );
    }
    #endif
    dilated.assign(*this);
    
    for( x = 1; x < width - 1; x++ ){
        for( y = 1; y < height - 1; y++ ){
            if( dilated(x, y) == threshold ){
                (unsigned int &)dilated( x - 1, y ) |= (unsigned int)mask;
                (unsigned int &)dilated( x + 1, y ) |= (unsigned int)mask;
                (unsigned int &)dilated( x, y - 1 ) |= (unsigned int)mask;
                (unsigned int &)dilated( x, y + 1 ) |= (unsigned int)mask;
            }
            else if( dilated(x, y) ){
                dilated(x, y) = threshold;
            }
        }
    }
}

template <typename T> 
inline Matrix<T>& Matrix<T>::dilate( T threshold, T mask ){
    Matrix<T> dilated;
    this->dilate( dilated, threshold, mask );
    assign(dilated);
    
    return *this;
}

template <typename T> 
inline Matrix<T>& Matrix<T>::erode( Matrix& eroded, T lthreshold, T hthreshold, T mask ){
    uint x, y, 
         dwidth  = width - 1,
         dheight = height - 1;
    #ifdef MSPDIMCHECK
    if( depth != 1 ){
        throw MSPException( __FILE__, __LINE__, "Erosion is possible only on grayscale images (MxNx1 Matrix) ." );
    }
    #endif
    
    eroded.assign(*this);
    
    for( x = 1; x < dwidth; x++ ){
        for( y = 1; y < dheight; y++ ){
            if( eroded(x, y) == lthreshold ){
                (unsigned int &)eroded( x - 1, y ) &= (unsigned int)mask;
                (unsigned int &)eroded( x + 1, y ) &= (unsigned int)mask;
                (unsigned int &)eroded( x, y - 1 ) &= (unsigned int)mask;
                (unsigned int &)eroded( x, y + 1 ) &= (unsigned int)mask;
            }
            else if( eroded(x, y) != hthreshold ){
                eroded(x, y) = lthreshold;
            }
        }
    }
}

template <typename T> 
inline Matrix<T>& Matrix<T>::erode( T lthreshold, T hthreshold, T mask ){
    Matrix<T> eroded;
    this->erode( eroded, lthreshold, hthreshold, mask );
    assign(eroded);
    
    return *this;
}

template <typename T> 
inline Matrix<T>& Matrix<T>::histogram( Matrix<uint>& histogram, uint x, uint y, uint width, uint height ){
    uint xx, yy, z, xe = x + width, ye = y + height;
    #ifdef MSPDIMCHECK    
    /* check if we are in a 0-255 value space */
    for( xx = x; xx < xe; xx++ ){
        for( yy = y; yy < ye; yy++ ){
            for( z = 0; z < depth; z++ ){
                if( data[xx][yy][z] < (T)0 || data[xx][yy][z] > (T)255 ){
                    throw MSPException( __FILE__, __LINE__, "Histogram computing is possible only in a 0-255 value space ." );
                }
            }
        }
    }
    #endif
    
    /* after those checks compute histogram */
    histogram.resize( 256, 1, depth );
    for( xx = x; xx < xe; xx++ ){
        for( yy = y; yy < ye; yy++ ){
            for( z = 0; z < depth; z++ ){
                histogram( data[xx][yy][z], 0, z )++;
            }
        }
    }
    
    return *this;
}

template <typename T> 
inline Matrix<T>& Matrix<T>::histogram( Matrix<uint>& histogram ){
    uint x, y, z;
    #ifdef MSPDIMCHECK    
    /* check if we are in a 0-255 value space */
    for( x = 0; x < width; x++ ){
        for( y = 0; y < height; y++ ){
            for( z = 0; z < depth; z++ ){
                if( data[x][y][z] < (T)0 || data[x][y][z] > (T)255 ){
                    throw MSPException( __FILE__, __LINE__, "Histogram computing is possible only in a 0-255 value space ." );
                }
            }
        }
    }
    #endif
    
    /* after those checks compute histogram */
    histogram.resize( 256, 1, depth );
    for( x = 0; x < width; x++ ){
        for( y = 0; y < height; y++ ){
            for( z = 0; z < depth; z++ ){
                histogram( data[x][y][z], 0, z )++;
            }
        }
    }
    
    return *this;
}

template <typename T> 
inline Matrix<T>& Matrix<T>::histogram(){
    Matrix<uint> histogram;
    this->histogram( histogram );
    assign(histogram);
    
    return *this;
}

template <typename T> 
inline double Matrix<T>::mean( Matrix<uint>& histogram ){
    #ifdef MSPDIMCHECK
    if( histogram.width != 256 ){
        throw MSPException( __FILE__, __LINE__, "Invalid histogram size %d, should be 256 .", histogram.width );
    }
    #endif
    
    uint i;
    double mean = 0.0f;
    
    for( i = 1; i < 255; i++ ){
        mean += i * histogram(i);
    }
    
    mean /= width * height;
    
    return mean;
}

template <typename T> 
inline double Matrix<T>::variance( Matrix<uint>& histogram ){
    #ifdef MSPDIMCHECK
    if( histogram.width != 256 ){
        throw MSPException( __FILE__, __LINE__, "Invalid histogram size %d, should be 256 .", histogram.width );
    }
    #endif
    
    uint i;
    double mean = 0.0f, variance = 0.0f;
    
    mean = this->mean(histogram);
    for( i = 0; i < 255; i++ ){
        variance += histogram(i) * (i - mean) * (i - mean);
    }
    
    variance /= width * height;
    
    return variance;
}

template <typename T> 
inline Matrix<T>& Matrix<T>::binarize( Matrix& binarized, T lthreshold /*= DEFAULT_L_VAL*/, T hthreshold /*= DEFAULT_H_VAL*/ ){
    uint x, y, z, val, low = 0;
    
    binarized.assign(*this);
    
    for( x = 0; x < width; x++ ){
        for( y = 0; y < height; y++ ){
            low = 0;
            for( z = 0; z < depth; z++ ){
                if( binarized[x][y][z] == lthreshold ){
                    low = 1;
                }
                else{
                    low = 0;
                    break;
                }
            }
            val = (low == 1 ? hthreshold : lthreshold);
            for( z = 0; z < depth; z++ ){
                binarized[x][y][z] = val;
            }
        }
    }
    
    return *this;
}

template <typename T> 
inline Matrix<T>& Matrix<T>::binarize( T lthreshold /*= DEFAULT_L_VAL*/, T hthreshold /*= DEFAULT_H_VAL*/ ){
    Matrix<T> binarized;
    this->binarize( binarized, lthreshold, hthreshold );
    assign(binarized);
    
    return *this;
}

template <typename T> 
inline Matrix<T>& Matrix<T>::normalize( Matrix& normalized, double mean, double variance ){
    uint x, y, z;
    double fmean,
           fmean0,
           fsigma,
           fsigma0,
           fgray,
           fcoeff = 0.0;
    Matrix<int> histogram;
    
    normalized.assign(*this);
    normalized.histogram(histogram);
    
    fmean0  = mean;
    fmean   = normalized.mean(histogram);
    fsigma0 = sqrt(variance);
    fsigma  = sqrt( normalized.variance(histogram) );
    
    if( fsigma > 0.0 ){
        fcoeff = fsigma0 / fsigma;
    }
    
    for( y = 0; y < height; y++ ){
        for( x = 0; x < width; x++ ){
            for( z = 0; z < depth; z++ ){
                fgray = (double)normalized[x][y][z];
                fgray = fmean0 + fcoeff * (fgray - mean);
                if( fgray < 0.0 ){
                    fgray = 0.0;
                }
                else if( fgray > 255.0 ){
                    fgray = 255.0;
                }
                
                normalized[x][y][z] = (T)fgray;
            }
        }
    }
    return *this;
}

template <typename T> 
inline Matrix<T>& Matrix<T>::normalize( double mean, double variance ){
    Matrix<T> normalized;
    this->normalize( normalized, mean, variance );
    assign(normalized);
    
    return *this;
}

template <typename T> 
inline Matrix<T>& Matrix<T>::transpose( Matrix& transposed ){
    uint x, y, z;
    
    transposed.resize( height, width, depth );
    for( x = 0; x < width; x++ ){
        for( y = 0; y < height; y++ ){
            for( z = 0; z < depth; z++ ){
                transposed[y][x][z] = data[x][y][z];
            }
        }
    }
    
    return *this;
}

template <typename T> 
inline Matrix<T>& Matrix<T>::transpose(){
    Matrix<T> transposed;
    this->transpose( transposed );
    assign(transposed);
    
    return *this;
}

template <typename T> 
inline Matrix<T>& Matrix<T>::integral( Matrix& integral ){
    #ifdef MSPDIMCHECK
    if( depth != 1 ){
        throw MSPException( __FILE__, __LINE__, "Integral computation is possible only on grayscale images (MxNx1 Matrix) ." );
    }
    #endif
    
    uint x, y;
    
    integral.assign(*this);
    for( x = 1; x < width; x++ ){
        integral(x,0) += integral(x - 1, 0);
    }
    for( y = 1; y < height; y++ ){
        integral(0,y) += integral(0, y - 1);
    }
    for( x = 1; x < width; x++ ){ 
        for( y = 1; y < height; y++ ){ 
            integral(x,y) += (integral(x,y - 1) + integral(x - 1,y)) - integral(x - 1,y - 1);
        }    
    }
    
    return *this;
}

template <typename T> 
inline Matrix<T>& Matrix<T>::integral(){
    Matrix<T> integral;
    this->integral( integral );
    assign(integral);
    
    return *this;
}

template <typename T> 
inline Matrix<T>& Matrix<T>::features( vector< Matrix<T> * >& features, 
                                       Matrix& orientation,
                                       Matrix& entropy,
                                       uint minneighbours,
                                       uint maxneighbours ){
    uint x, y, neighbours;
    T angle;
    Matrix<T> *feature;
    
    features.clear();
    
    for( x = 1; x < width - 1; x++ ){
        for( y = 1; y < height - 1; y++ ){
            /* check if we are in a informative region */
            if( entropy(x,y) && data[x][y][0] ){
                /* check if we have an edge on the entropy mask */
                if( entropy_empty_neighbours( entropy, x, y, 8 ) == 0 ){
                    neighbours = 0;
                    if( data[x][y-1][0] ){
                        neighbours++;
                    }
                    if( data[x+1][y-1][0] ){
                        neighbours++;
                    }
                    if( data[x+1][y][0] ){
                        neighbours++;
                    }
                    if( data[x+1][y+1][0] ){
                        neighbours++;
                    }
                    if( data[x][y+1][0] ){
                        neighbours++;
                    }
                    if( data[x-1][y+1][0] ){
                        neighbours++;
                    }
                    if( data[x-1][y][0] ){
                        neighbours++;
                    }
                    if( data[x-1][y-1][0] ){
                        neighbours++;
                    }
                    
                    if( neighbours >= minneighbours && neighbours <= maxneighbours ){
                        angle         = orientation(x,y);
                        feature       = new Matrix<T>( 4 );
                        (*feature)(0) = x;
                        (*feature)(1) = y;
                        (*feature)(2) = neighbours;
                        (*feature)(3) = angle;
                        features.push_back(feature);
                    }
                }
            }
        }
    }
    
    return *this;
}

template <typename T> 
inline double Matrix<T>::delta( Matrix& m, uint x, uint y, uint widht, uint height ){
    uint xx, yy, z, xe = x + width, ye = y + height;
    double tmp = 0.0, delta = 0.0;
    
    #ifdef MSPDIMCHECK
    if( depth != m.depth ){
        throw MSPException( __FILE__, __LINE__, "Delta computation is possible only between matrices with the same depth ." );
    }
    #endif
    
    xe = (xe < this->width  ? xe : this->width);
    ye = (ye < this->height ? ye : this->height);
    for( xx = x; xx < xe; xx++ ){
        for( yy = y; yy < ye; yy++ ){
            tmp = 0.0;
            for( z = 0; z < depth; z++ ){
                tmp += this->delta( data[xx][yy][z], m[xx][yy][z] );
            }
            delta += (tmp / (double)depth);
        }
    }

    return delta;
}

}
#endif
