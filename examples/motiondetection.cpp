#include "../msp.hpp"
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <malloc.h>
#include <linux/videodev.h>

using namespace MSP;

typedef unsigned char byte;
typedef unsigned int  uint;

typedef struct{
    int  fd;
    byte * mmap;
    char device[0xFF];
    char name[0xFF];
    uint width;
    uint height;
    uint depth;
}
device_t;

typedef struct{
    byte * data;
    uint   width;
    uint   height;
    uint   depth;
    uint   size;
}
video_buffer_t;

int cam_open( char *devname, device_t * dev ){
    struct video_capability vcap;
    struct video_channel    vchan;
    
    if( (dev->fd = open( devname, O_RDWR )) <= 0 ){
        perror("open");
        return -1;
    }
    
    strcpy( dev->device, devname );
    
    if( ioctl( dev->fd, VIDIOCGCAP, &vcap ) < 0 ){
        perror("VIDIOCGCAP");
        return -1;
    }
    
    strcpy( dev->name, vcap.name );
    
    dev->width  = vcap.maxwidth;
    dev->height = vcap.maxheight;
    dev->depth  = 3;
    
    vchan.channel = 1;
    if( ioctl( dev->fd, VIDIOCGCHAN, &vchan ) < 0 ){
        perror("VIDIOCGCHAN");
        return -1;
    }
    vchan.norm = 1;
    if( ioctl( dev->fd, VIDIOCSCHAN, &vchan ) < 0 ){
        perror("VIDIOCSCHAN");
        return -1;
    }
    
    dev->mmap = (byte *)mmap( 0, dev->width * dev->height * dev->depth, PROT_READ|PROT_WRITE, MAP_SHARED, dev->fd, 0 );
    
    return 1;
}

int cam_capture( device_t * dev, video_buffer_t * vbuffer ){
    struct video_mmap vmap;
    
    vmap.width     = vbuffer->width  = dev->width;
    vmap.height    = vbuffer->height = dev->height;
    vmap.frame     = 0;
    vmap.format    = VIDEO_PALETTE_RGB24;
    vbuffer->depth = dev->depth;
    vbuffer->size  = vmap.width * vmap.height * vbuffer->depth;
    
    if( ioctl( dev->fd, VIDIOCMCAPTURE, &vmap ) < 0 ){
        perror("VIDIOCMCAPTURE");
        return -1;
    }
    
    if( ioctl( dev->fd, VIDIOCSYNC, &vmap.frame ) < 0 ){
        perror("VIDIOCSYNC");
        return -1;
    }
    
    vbuffer->data = dev->mmap;
    
    return 1;
}

void cam_close( device_t * dev ){
    munmap( dev->mmap, dev->width * dev->height * dev->depth );
    close( dev->fd);
}


int main(int argc, char** argv) {
    try{
        Matrix<byte>  *frame, *lastframe = NULL;
        device_t       device;
        video_buffer_t video;
        double         delta;
        uint           x, y,
                bsize = 8, sectors = 0, total;
        
        if( cam_open( "/dev/video0", &device ) < 0 ){
            return -1;
        }
        
        while(true){
            if( cam_capture( &device, &video ) < 0 ){
                return -1;
            }
            frame = new Matrix<byte>( video.data, video.width, video.height, video.depth );
            
            if( lastframe == NULL ){
                lastframe = new Matrix<byte>(*frame);
            }
            else{
                sectors = 0;
                for( x = 0; x < frame->width; x += bsize ){
                    for( y = 0; y < frame->height; y += bsize ){          
                        if( lastframe->delta( *frame, x, y, bsize, bsize ) >= 3000.0f ){
                            sectors++;
                        }
                    }
                }
                
                lastframe->assign(*frame);
                
                total = (frame->width * frame->height) / bsize;
                delta = (sectors * 100.0f) / total;
                printf( "MOTION : %f%%\n", delta );
            }
            
            delete frame;
        }
        
        cam_close(&device);
    }
    catch( MSPException me ){
        me.what();
    }
    
    return (EXIT_SUCCESS);
}

