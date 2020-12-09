#define _GNU_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <pthread.h>
#include <sched.h>
#include <time.h>
#include <semaphore.h>

#include <syslog.h>
#include <sys/time.h>
#include <sys/sysinfo.h>
#include <errno.h>

#include <signal.h>


#include <string.h>
#include <assert.h>


#include <getopt.h>             /* getopt_long() */

#include <fcntl.h>              /* low-level i/o */
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

#include <linux/videodev2.h>


#define USEC_PER_MSEC (1000)
#define NANOSEC_PER_MSEC (1000000)
#define NANOSEC_PER_SEC (1000000000)
#define NUM_CPU_CORES (4)
#define TRUE (1)
#define FALSE (0)

#define RT_CORE (2)

#define NUM_THREADS (3)


#define MY_CLOCK_TYPE CLOCK_MONOTONIC_RAW

#define CLEAR(x) memset(&(x), 0, sizeof(x))

#define MAX_PIXEL_SIZE (3)

#define HRES (640)
#define VRES (480)
#define PIXEL_SIZE (2)
#define HRES_STR "640"
#define VRES_STR "480"


#define STARTUP_FRAMES (30)
#define LAST_FRAMES (1)
#define CAPTURE_FRAMES (1800+LAST_FRAMES)
#define FRAMES_TO_ACQUIRE (CAPTURE_FRAMES + STARTUP_FRAMES + LAST_FRAMES)

#define FRAMES_PER_SEC (1)  

#define DUMP_FRAMES
#define COLOR_CONVERT_GRAY

#define DRIVER_MMAP_BUFFERS (6)  // request buffers for delay


int abortTest=FALSE;
int abortS1=FALSE, abortS2=FALSE, abortS3=FALSE;
sem_t semS1, semS2, semS3;
struct timespec start_time_val;
double start_realtime;

static timer_t timer_1;
static struct itimerspec itime = {{1,0}, {1,0}};
static struct itimerspec last_itime;

static unsigned long long seqCnt=0;

typedef struct
{
    int threadIdx;
} threadParams_t;


void Sequencer(int id);

void *Service_1_frame_acquisition(void *threadp);
void *Service_2_frame_process(void *threadp);
void *Service_3_frame_storage(void *threadp);

int seq_frame_read(void);
int seq_frame_process(void);
int seq_frame_store(void);

double getTimeMsec(void);
double realtime(struct timespec *tsptr);
void print_scheduler(void);


int v4l2_frame_acquisition_initialization(char *dev_name);
int v4l2_frame_acquisition_shutdown(void);
int v4l2_frame_acquisition_loop(char *dev_name);



// Format is used by a number of functions, so made as a file global
static struct v4l2_format fmt;
struct v4l2_buffer frame_buf;

struct buffer 
{
        void   *start;
        size_t  length;
};


struct save_frame_t
{
    unsigned char   frame[HRES*VRES*PIXEL_SIZE];
    struct timespec time_stamp;
    char identifier_str[80];
};

struct ring_buffer_t
{
    unsigned int ring_size;

    int tail_idx;
    int head_idx;
    int count;

    struct save_frame_t save_frame[3*FRAMES_PER_SEC];
};

static  struct ring_buffer_t	ring_buffer;

static int              camera_device_fd = -1;
struct buffer          *buffers;
static unsigned int     n_buffers;
static int              force_format=1;


static double fnow=0.0, fstart=0.0, fstop=0.0;
static struct timespec time_now, time_start, time_stop;

static void errno_exit(const char *s)
{
        fprintf(stderr, "%s error %d, %s\n", s, errno, strerror(errno));
        exit(EXIT_FAILURE);
}


static int xioctl(int fh, int request, void *arg)
{
    int rc;

    do 
    {
        rc = ioctl(fh, request, arg);

    } while (-1 == rc && EINTR == errno);

    return rc;
}



char pgm_header[]="P5\n#9999999999 sec 9999999999 msec \n"HRES_STR" "VRES_STR"\n255\n";
char pgm_dumpname[]="frames/test0000.pgm";

static void dump_pgm(const void *p, int size, unsigned int tag, struct timespec *time)
{
    int written, i, total, dumpfd;
   
    snprintf(&pgm_dumpname[11], 9, "%04d", tag);
    strncat(&pgm_dumpname[15], ".pgm", 5);
    dumpfd = open(pgm_dumpname, O_WRONLY | O_NONBLOCK | O_CREAT, 00666);

    snprintf(&pgm_header[4], 11, "%010d", (int)time->tv_sec);
    strncat(&pgm_header[14], " sec ", 5);
    snprintf(&pgm_header[19], 11, "%010d", (int)((time->tv_nsec)/1000000));
    strncat(&pgm_header[29], " msec \n"HRES_STR" "VRES_STR"\n255\n", 19);

    // subtract 1 from sizeof header because it includes the null terminator for the string
    written=write(dumpfd, pgm_header, sizeof(pgm_header)-1);

    total=0;

    do
    {
        written=write(dumpfd, p, size);
        total+=written;
    } while(total < size);

    clock_gettime(CLOCK_MONOTONIC, &time_now);
    fnow = (double)time_now.tv_sec + (double)time_now.tv_nsec / 1000000000.0;
    printf("Frame written to flash at %lf, %d, bytes\n", (fnow-fstart), total);

    close(dumpfd);
    
}


void yuv2rgb_float(float y, float u, float v, 
                   unsigned char *r, unsigned char *g, unsigned char *b)
{
    float r_temp, g_temp, b_temp;

    // R = 1.164(Y-16) + 1.1596(V-128)
    r_temp = 1.164*(y-16.0) + 1.1596*(v-128.0);  
    *r = r_temp > 255.0 ? 255 : (r_temp < 0.0 ? 0 : (unsigned char)r_temp);

    // G = 1.164(Y-16) - 0.813*(V-128) - 0.391*(U-128)
    g_temp = 1.164*(y-16.0) - 0.813*(v-128.0) - 0.391*(u-128.0);
    *g = g_temp > 255.0 ? 255 : (g_temp < 0.0 ? 0 : (unsigned char)g_temp);

    // B = 1.164*(Y-16) + 2.018*(U-128)
    b_temp = 1.164*(y-16.0) + 2.018*(u-128.0);
    *b = b_temp > 255.0 ? 255 : (b_temp < 0.0 ? 0 : (unsigned char)b_temp);
}



void yuv2rgb(int y, int u, int v, unsigned char *r, unsigned char *g, unsigned char *b)
{
   int r1, g1, b1;

   // replaces floating point coefficients
   int c = y-16, d = u - 128, e = v - 128;       

   // Conversion that avoids floating point
   r1 = (298 * c           + 409 * e + 128) >> 8;
   g1 = (298 * c - 100 * d - 208 * e + 128) >> 8;
   b1 = (298 * c + 516 * d           + 128) >> 8;

   // Computed values may need clipping.
   if (r1 > 255) r1 = 255;
   if (g1 > 255) g1 = 255;
   if (b1 > 255) b1 = 255;

   if (r1 < 0) r1 = 0;
   if (g1 < 0) g1 = 0;
   if (b1 < 0) b1 = 0;

   *r = r1 ;
   *g = g1 ;
   *b = b1 ;
}


// always ignore STARTUP_FRAMES while camera adjusts to lighting, focuses, etc.
int read_framecnt=-STARTUP_FRAMES;
int process_framecnt=0;
int save_framecnt=0;

unsigned char scratchpad_buffer[HRES*VRES*MAX_PIXEL_SIZE];


static int save_image(const void *p, int size, struct timespec *frame_time)
{
    int i, newi, newsize=0;
    unsigned char *frame_ptr = (unsigned char *)p;

    save_framecnt++;
    printf("save frame %d: ", save_framecnt);
    
#ifdef DUMP_FRAMES	

    if(fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_GREY)
    {
        printf("Dump graymap as-is size %d\n", size);
        dump_pgm(frame_ptr, size, save_framecnt, frame_time);
    }

    else if(fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_YUYV)
    {

#if defined(COLOR_CONVERT_GRAY)
        if(save_framecnt > 0)
        {
            dump_pgm(frame_ptr, (size/2), process_framecnt, frame_time);
            printf("Dump YUYV converted to YY size %d\n", size);
        }
#endif

    }

    else
    {
        printf("ERROR - unknown dump format\n");
    }
#endif

    return save_framecnt;
}


static int process_image(const void *p, int size)
{
    int i, newi, newsize=0;
    int y_temp, y2_temp, u_temp, v_temp;
    unsigned char *frame_ptr = (unsigned char *)p;

    process_framecnt++;
    printf("process frame %d: ", process_framecnt);
    
    if(fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_GREY)
    {
        printf("NO PROCESSING for graymap as-is size %d\n", size);
    }

    else if(fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_YUYV)
    {
#if defined(COLOR_CONVERT_GRAY)
        // Pixels are YU and YV alternating, so YUYV which is 4 bytes
        // We want Y, so YY which is 2 bytes
        //
        for(i=0, newi=0; i<size; i=i+4, newi=newi+2)
        {
            // Y1=first byte and Y2=third byte
            scratchpad_buffer[newi]=frame_ptr[i];
            scratchpad_buffer[newi+1]=frame_ptr[i+2];
        }
#endif
    }

    else if(fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_RGB24)
    {
        printf("NO PROCESSING for RGB as-is size %d\n", size);
    }
    else
    {
        printf("NO PROCESSING ERROR - unknown format\n");
    }

    return process_framecnt;
}


static int read_frame(void)
{
    CLEAR(frame_buf);

    frame_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    frame_buf.memory = V4L2_MEMORY_MMAP;

    if (-1 == xioctl(camera_device_fd, VIDIOC_DQBUF, &frame_buf))
    {
        switch (errno)
        {
            case EAGAIN:
                return 0;

            case EIO:
                /* Could ignore EIO, but drivers should only set for serious errors, although some set for
                   non-fatal errors too.
                 */
                return 0;


            default:
                printf("mmap failure\n");
                errno_exit("VIDIOC_DQBUF");
        }
    }

    read_framecnt++;

    //printf("frame %d ", read_framecnt);

    if(read_framecnt == 0) 
    {
        clock_gettime(CLOCK_MONOTONIC, &time_start);
        fstart = (double)time_start.tv_sec + (double)time_start.tv_nsec / 1000000000.0;
    }

    assert(frame_buf.index < n_buffers);

    return 1;
}


int seq_frame_read(void)
{
    fd_set fds;
    struct timeval tv;
    int rc;

    FD_ZERO(&fds);
    FD_SET(camera_device_fd, &fds);

    /* Timeout */
    tv.tv_sec = 2;
    tv.tv_usec = 0;

    rc = select(camera_device_fd + 1, &fds, NULL, NULL, &tv);

    read_frame();

    // save off copy of image with time-stamp here
    //printf("memcpy to %p from %p for %d bytes\n", (void *)&(ring_buffer.save_frame[ring_buffer.tail_idx].frame[0]), buffers[frame_buf.index].start, frame_buf.bytesused);
    //syslog(LOG_CRIT, "memcpy to %p from %p for %d bytes\n", (void *)&(ring_buffer.save_frame[ring_buffer.tail_idx].frame[0]), buffers[frame_buf.index].start, frame_buf.bytesused);
    memcpy((void *)&(ring_buffer.save_frame[ring_buffer.tail_idx].frame[0]), buffers[frame_buf.index].start, frame_buf.bytesused);

    ring_buffer.tail_idx = (ring_buffer.tail_idx + 1) % ring_buffer.ring_size;
    ring_buffer.count++;

    clock_gettime(CLOCK_MONOTONIC, &time_now);
    fnow = (double)time_now.tv_sec + (double)time_now.tv_nsec / 1000000000.0;

    if(read_framecnt > 0)
    {	
        //printf("read_framecnt=%d, rb.tail=%d, rb.head=%d, rb.count=%d at %lf and %lf FPS", read_framecnt, ring_buffer.tail_idx, ring_buffer.head_idx, ring_buffer.count, (fnow-fstart), (double)(read_framecnt) / (fnow-fstart));

        //syslog(LOG_CRIT, "read_framecnt=%d, rb.tail=%d, rb.head=%d, rb.count=%d at %lf and %lf FPS", read_framecnt, ring_buffer.tail_idx, ring_buffer.head_idx, ring_buffer.count, (fnow-fstart), (double)(read_framecnt) / (fnow-fstart));
        syslog(LOG_CRIT, "read_framecnt=%d at %lf and %lf FPS", read_framecnt, (fnow-fstart), (double)(read_framecnt) / (fnow-fstart));
    }
    else 
    {
        printf("at %lf\n", fnow);
    }

    if (-1 == xioctl(camera_device_fd, VIDIOC_QBUF, &frame_buf))
        errno_exit("VIDIOC_QBUF");
}



int seq_frame_process(void)
{
    int cnt;

    printf("processing rb.tail=%d, rb.head=%d, rb.count=%d\n", ring_buffer.tail_idx, ring_buffer.head_idx, ring_buffer.count);

    ring_buffer.head_idx = (ring_buffer.head_idx + 2) % ring_buffer.ring_size;

    cnt=process_image((void *)&(ring_buffer.save_frame[ring_buffer.head_idx].frame[0]), HRES*VRES*PIXEL_SIZE);

    ring_buffer.head_idx = (ring_buffer.head_idx + 3) % ring_buffer.ring_size;
    ring_buffer.count = ring_buffer.count - 5;

     	
    printf("rb.tail=%d, rb.head=%d, rb.count=%d ", ring_buffer.tail_idx, ring_buffer.head_idx, ring_buffer.count);
       
    if(process_framecnt > 0)
    {	
        clock_gettime(CLOCK_MONOTONIC, &time_now);
        fnow = (double)time_now.tv_sec + (double)time_now.tv_nsec / 1000000000.0;
                printf(" processed at %lf, @ %lf FPS\n", (fnow-fstart), (double)(process_framecnt+1) / (fnow-fstart));
    }
    else 
    {
        printf("at %lf\n", fnow-fstart);
    }

    return cnt;
}


int seq_frame_store(void)
{
    int cnt;

    cnt=save_image(scratchpad_buffer, HRES*VRES*PIXEL_SIZE, &time_now);
    printf("save_framecnt=%d ", save_framecnt);


    if(save_framecnt > 0)
    {	
        clock_gettime(CLOCK_MONOTONIC, &time_now);
        fnow = (double)time_now.tv_sec + (double)time_now.tv_nsec / 1000000000.0;
                printf(" saved at %lf, @ %lf FPS\n", (fnow-fstart), (double)(process_framecnt+1) / (fnow-fstart));
    }
    else 
    {
        printf("at %lf\n", fnow-fstart);
    }

    return cnt;
}


static void mainloop(void)
{
    unsigned int count;
    struct timespec read_delay;
    struct timespec time_error;

    // Replace this with a delay designed for your rate
    // of frame acquitision and storage.
    //
  
#if (FRAMES_PER_SEC  == 1)
    printf("Running at 1 frame/sec\n");
    read_delay.tv_sec=0;
    read_delay.tv_nsec=920000000;
#elif (FRAMES_PER_SEC == 10)
    printf("Running at 10 frames/sec\n");
    read_delay.tv_sec=0;
    read_delay.tv_nsec=100000000;
#elif (FRAMES_PER_SEC == 20)
    printf("Running at 20 frames/sec\n");
    read_delay.tv_sec=0;
    read_delay.tv_nsec=50000000;
#elif (FRAMES_PER_SEC == 25)
    printf("Running at 25 frames/sec\n");
    read_delay.tv_sec=0;
    read_delay.tv_nsec=40000000;
#elif (FRAMES_PER_SEC == 30)
    printf("Running at 30 frames/sec\n");
    read_delay.tv_sec=0;
    read_delay.tv_nsec=33333333;
#else
    printf("Running at 1 frame/sec\n");
    read_delay.tv_sec=1;
    read_delay.tv_nsec=0;
#endif

    count = FRAMES_TO_ACQUIRE;

    while (count > 0)
    {
        for (;;)
        {
            fd_set fds;
            struct timeval tv;
            int rc;

            FD_ZERO(&fds);
            FD_SET(camera_device_fd, &fds);

            /* Timeout */
            tv.tv_sec = 2;
            tv.tv_usec = 0;

            rc = select(camera_device_fd + 1, &fds, NULL, NULL, &tv);

            if (-1 == rc)
            {
                if (EINTR == errno)
                    continue;
                errno_exit("select");
            }

            if (0 == rc)
            {
                fprintf(stderr, "select timeout\n");
                exit(EXIT_FAILURE);
            }

            if (read_frame())
            {
                if(nanosleep(&read_delay, &time_error) != 0)
                    perror("nanosleep");
                else
		{
		    clock_gettime(CLOCK_MONOTONIC, &time_now);
		    fnow = (double)time_now.tv_sec + (double)time_now.tv_nsec / 1000000000.0;

		    if(read_framecnt > 1)
	            {	
                        printf(" read at %lf, @ %lf FPS\n", (fnow-fstart), (double)(read_framecnt+1) / (fnow-fstart));

                        memcpy((void *)&(ring_buffer.save_frame[ring_buffer.tail_idx].frame[0]), buffers[frame_buf.index].start, frame_buf.bytesused);
			printf("memcpy to rb.tail=%d, rb.head=%d, ptr=%p\n", ring_buffer.tail_idx, ring_buffer.head_idx, (void *)&(ring_buffer.save_frame[ring_buffer.tail_idx].frame[0]));

                        // advance ring buffer for next read
                        ring_buffer.tail_idx = (ring_buffer.tail_idx + 1) % ring_buffer.ring_size;
                        ring_buffer.count++;


                        process_image((void *)&(ring_buffer.save_frame[ring_buffer.head_idx].frame[0]), HRES*VRES*PIXEL_SIZE);
                        //process_image(buffers[frame_buf.index].start, frame_buf.bytesused);
			printf("bytesused=%d, hxvxp=%d\n", frame_buf.bytesused, HRES*VRES*PIXEL_SIZE);
                        process_image((void *)&(ring_buffer.save_frame[ring_buffer.head_idx].frame[0]), HRES*VRES*PIXEL_SIZE);

			printf("process from rb.tail=%d, rb.head=%d, ptr=%p\n", ring_buffer.tail_idx, ring_buffer.head_idx, (void *)&(ring_buffer.save_frame[ring_buffer.head_idx].frame[0]));
                        save_image(scratchpad_buffer, HRES*VRES*PIXEL_SIZE, &time_now);

                        // advance ring buffer for next write
                        ring_buffer.head_idx = (ring_buffer.head_idx + 1) % ring_buffer.ring_size;
                        ring_buffer.count--;

		    }
		    else 
		    {
                        printf("at %lf\n", (fnow-fstart));
		    }
		}

                if (-1 == xioctl(camera_device_fd, VIDIOC_QBUF, &frame_buf))
                        errno_exit("VIDIOC_QBUF");
                count--;
                break;
            }

            /* EAGAIN - continue select loop unless count done. */
            if(count <= 0) break;
        }

        if(count <= 0) break;
    }

}


static void stop_capturing(void)
{
    enum v4l2_buf_type type;

    clock_gettime(CLOCK_MONOTONIC, &time_stop);
    fstop = (double)time_stop.tv_sec + (double)time_stop.tv_nsec / 1000000000.0;

    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if(-1 == xioctl(camera_device_fd, VIDIOC_STREAMOFF, &type))
		    errno_exit("VIDIOC_STREAMOFF");

    printf("capture stopped\n");
}


static void start_capturing(void)
{
        unsigned int i;
        enum v4l2_buf_type type;

	printf("will capture to %d buffers\n", n_buffers);

        for (i = 0; i < n_buffers; ++i) 
        {
                printf("allocated buffer %d\n", i);

                CLEAR(frame_buf);
                frame_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                frame_buf.memory = V4L2_MEMORY_MMAP;
                frame_buf.index = i;

                if (-1 == xioctl(camera_device_fd, VIDIOC_QBUF, &frame_buf))
                        errno_exit("VIDIOC_QBUF");
        }

        type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

        if (-1 == xioctl(camera_device_fd, VIDIOC_STREAMON, &type))
                errno_exit("VIDIOC_STREAMON");

}


static void uninit_device(void)
{
        unsigned int i;

        for (i = 0; i < n_buffers; ++i)
                if (-1 == munmap(buffers[i].start, buffers[i].length))
                        errno_exit("munmap");

        free(buffers);
}


static void init_mmap(char *dev_name)
{
        struct v4l2_requestbuffers req;

        CLEAR(req);

        req.count = DRIVER_MMAP_BUFFERS;
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;

	printf("init_mmap req.count=%d\n",req.count);

	ring_buffer.tail_idx=0;
	ring_buffer.head_idx=0;
	ring_buffer.count=0;
	ring_buffer.ring_size=3*FRAMES_PER_SEC;

        if (-1 == xioctl(camera_device_fd, VIDIOC_REQBUFS, &req)) 
        {
                if (EINVAL == errno) 
                {
                        fprintf(stderr, "%s does not support "
                                 "memory mapping\n", dev_name);
                        exit(EXIT_FAILURE);
                } else 
                {
                        errno_exit("VIDIOC_REQBUFS");
                }
        }

        if (req.count < 2) 
        {
                fprintf(stderr, "Insufficient buffer memory on %s\n", dev_name);
                exit(EXIT_FAILURE);
        }
	else
	{
	    printf("Device supports %d mmap buffers\n", req.count);

	    // allocate tracking buffers array for those that are mapped
            buffers = calloc(req.count, sizeof(*buffers));


	    // set up double buffer for frames to be safe with one time malloc her or just declare

	}

        if (!buffers) 
        {
                fprintf(stderr, "Out of memory\n");
                exit(EXIT_FAILURE);
        }

        for (n_buffers = 0; n_buffers < req.count; ++n_buffers) 
	{
                CLEAR(frame_buf);

                frame_buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                frame_buf.memory      = V4L2_MEMORY_MMAP;
                frame_buf.index       = n_buffers;

                if (-1 == xioctl(camera_device_fd, VIDIOC_QUERYBUF, &frame_buf))
                        errno_exit("VIDIOC_QUERYBUF");

                buffers[n_buffers].length = frame_buf.length;
                buffers[n_buffers].start =
                        mmap(NULL /* start anywhere */,
                              frame_buf.length,
                              PROT_READ | PROT_WRITE /* required */,
                              MAP_SHARED /* recommended */,
                              camera_device_fd, frame_buf.m.offset);

                if (MAP_FAILED == buffers[n_buffers].start)
                        errno_exit("mmap");

                printf("mappped buffer %d\n", n_buffers);
        }
}


static void init_device(char *dev_name)
{
    struct v4l2_capability cap;
    struct v4l2_cropcap cropcap;
    struct v4l2_crop crop;
    unsigned int min;

    if (-1 == xioctl(camera_device_fd, VIDIOC_QUERYCAP, &cap))
    {
        if (EINVAL == errno) {
            fprintf(stderr, "%s is no V4L2 device\n",
                     dev_name);
            exit(EXIT_FAILURE);
        }
        else
        {
                errno_exit("VIDIOC_QUERYCAP");
        }
    }

    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
    {
        fprintf(stderr, "%s is no video capture device\n",
                 dev_name);
        exit(EXIT_FAILURE);
    }

    if (!(cap.capabilities & V4L2_CAP_STREAMING))
    {
        fprintf(stderr, "%s does not support streaming i/o\n",
                 dev_name);
        exit(EXIT_FAILURE);
    }


    /* Select video input, video standard and tune here. */


    CLEAR(cropcap);

    cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (0 == xioctl(camera_device_fd, VIDIOC_CROPCAP, &cropcap))
    {
        crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        crop.c = cropcap.defrect; /* reset to default */

        if (-1 == xioctl(camera_device_fd, VIDIOC_S_CROP, &crop))
        {
            switch (errno)
            {
                case EINVAL:
                    /* Cropping not supported. */
                    break;
                default:
                    /* Errors ignored. */
                        break;
            }
        }

    }
    else
    {
        /* Errors ignored. */
    }


    CLEAR(fmt);

    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (force_format)
    {
        printf("FORCING FORMAT\n");
        fmt.fmt.pix.width       = HRES;
        fmt.fmt.pix.height      = VRES;

        // Specify the Pixel Coding Formate here

        // This one works for Logitech C200
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;

        //fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY;
        //fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_VYUY;

        // Would be nice if camera supported
        //fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_GREY;
        //fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24;

        //fmt.fmt.pix.field       = V4L2_FIELD_INTERLACED;
        fmt.fmt.pix.field       = V4L2_FIELD_NONE;

        if (-1 == xioctl(camera_device_fd, VIDIOC_S_FMT, &fmt))
                errno_exit("VIDIOC_S_FMT");

        /* Note VIDIOC_S_FMT may change width and height. */
    }
    else
    {
        printf("ASSUMING FORMAT\n");
        /* Preserve original settings as set by v4l2-ctl for example */
        if (-1 == xioctl(camera_device_fd, VIDIOC_G_FMT, &fmt))
                    errno_exit("VIDIOC_G_FMT");
    }

    /* Buggy driver paranoia. */
    min = fmt.fmt.pix.width * 2;
    if (fmt.fmt.pix.bytesperline < min)
            fmt.fmt.pix.bytesperline = min;
    min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
    if (fmt.fmt.pix.sizeimage < min)
            fmt.fmt.pix.sizeimage = min;

    init_mmap(dev_name);
}


static void close_device(void)
{
        if (-1 == close(camera_device_fd))
                errno_exit("close");

        camera_device_fd = -1;
}


static void open_device(char *dev_name)
{
        struct stat st;

        if (-1 == stat(dev_name, &st)) {
                fprintf(stderr, "Cannot identify '%s': %d, %s\n",
                         dev_name, errno, strerror(errno));
                exit(EXIT_FAILURE);
        }

        if (!S_ISCHR(st.st_mode)) {
                fprintf(stderr, "%s is no device\n", dev_name);
                exit(EXIT_FAILURE);
        }

        camera_device_fd = open(dev_name, O_RDWR /* required */ | O_NONBLOCK, 0);

        if (-1 == camera_device_fd) {
                fprintf(stderr, "Cannot open '%s': %d, %s\n",
                         dev_name, errno, strerror(errno));
                exit(EXIT_FAILURE);
        }
}


int v4l2_frame_acquisition_loop(char *dev_name)
{

    // initialization of V4L2
    open_device(dev_name);
    init_device(dev_name);

    start_capturing();

    // service loop frame read
    mainloop();

    // shutdown of frame acquisition service
    stop_capturing();

    printf("Total capture time=%lf, for %d frames, %lf FPS\n", (fstop-fstart), read_framecnt, ((double)read_framecnt / (fstop-fstart)));

    uninit_device();
    close_device();
    fprintf(stderr, "\n");
    return 0;
}


int v4l2_frame_acquisition_initialization(char *dev_name)
{
    // initialization of V4L2
    open_device(dev_name);
    init_device(dev_name);

    start_capturing();
}


int v4l2_frame_acquisition_shutdown(void)
{
    // shutdown of frame acquisition service
    stop_capturing();

    printf("Total capture time=%lf, for %d frames, %lf FPS\n", (fstop-fstart), read_framecnt+1, ((double)read_framecnt / (fstop-fstart)));

    uninit_device();
    close_device();
    fprintf(stderr, "\n");
    return 0;
}






void main(void)
{
    struct timespec current_time_val, current_time_res;
    double current_realtime, current_realtime_res;

    char *dev_name="/dev/video0";

    int i, rc, scope, flags=0;

    cpu_set_t threadcpu;
    cpu_set_t allcpuset;

    pthread_t threads[NUM_THREADS];
    threadParams_t threadParams[NUM_THREADS];
    pthread_attr_t rt_sched_attr[NUM_THREADS];
    int rt_max_prio, rt_min_prio, cpuidx;

    struct sched_param rt_param[NUM_THREADS];
    struct sched_param main_param;

    pthread_attr_t main_attr;
    pid_t mainpid;

    v4l2_frame_acquisition_initialization(dev_name);

    // required to get camera initialized and ready
    seq_frame_read();

    printf("Starting High Rate Sequencer Demo\n");
    clock_gettime(MY_CLOCK_TYPE, &start_time_val); start_realtime=realtime(&start_time_val);
    clock_gettime(MY_CLOCK_TYPE, &current_time_val); current_realtime=realtime(&current_time_val);
    clock_getres(MY_CLOCK_TYPE, &current_time_res); current_realtime_res=realtime(&current_time_res);
    printf("START High Rate Sequencer @ sec=%6.9lf with resolution %6.9lf\n", (current_realtime - start_realtime), current_realtime_res);
    syslog(LOG_CRIT, "START High Rate Sequencer @ sec=%6.9lf with resolution %6.9lf\n", (current_realtime - start_realtime), current_realtime_res);


   printf("System has %d processors configured and %d available.\n", get_nprocs_conf(), get_nprocs());

   CPU_ZERO(&allcpuset);

   for(i=0; i < NUM_CPU_CORES; i++)
       CPU_SET(i, &allcpuset);

   printf("Using CPUS=%d from total available.\n", CPU_COUNT(&allcpuset));


    // initialize the sequencer semaphores
    //
    if (sem_init (&semS1, 0, 0)) { printf ("Failed to initialize S1 semaphore\n"); exit (-1); }
    if (sem_init (&semS2, 0, 0)) { printf ("Failed to initialize S2 semaphore\n"); exit (-1); }
    if (sem_init (&semS3, 0, 0)) { printf ("Failed to initialize S2 semaphore\n"); exit (-1); }

    mainpid=getpid();

    rt_max_prio = sched_get_priority_max(SCHED_FIFO);
    rt_min_prio = sched_get_priority_min(SCHED_FIFO);

    rc=sched_getparam(mainpid, &main_param);
    main_param.sched_priority=rt_max_prio;
    rc=sched_setscheduler(getpid(), SCHED_FIFO, &main_param);
    if(rc < 0) perror("main_param");
    print_scheduler();


    pthread_attr_getscope(&main_attr, &scope);

    if(scope == PTHREAD_SCOPE_SYSTEM)
      printf("PTHREAD SCOPE SYSTEM\n");
    else if (scope == PTHREAD_SCOPE_PROCESS)
      printf("PTHREAD SCOPE PROCESS\n");
    else
      printf("PTHREAD SCOPE UNKNOWN\n");

    printf("rt_max_prio=%d\n", rt_max_prio);
    printf("rt_min_prio=%d\n", rt_min_prio);


    for(i=0; i < NUM_THREADS; i++)
    {

      // run ALL threads on core RT_CORE
      CPU_ZERO(&threadcpu);
      cpuidx=(RT_CORE);
      CPU_SET(cpuidx, &threadcpu);

      rc=pthread_attr_init(&rt_sched_attr[i]);
      rc=pthread_attr_setinheritsched(&rt_sched_attr[i], PTHREAD_EXPLICIT_SCHED);
      rc=pthread_attr_setschedpolicy(&rt_sched_attr[i], SCHED_FIFO);
      rc=pthread_attr_setaffinity_np(&rt_sched_attr[i], sizeof(cpu_set_t), &threadcpu);

      rt_param[i].sched_priority=rt_max_prio-i;
      pthread_attr_setschedparam(&rt_sched_attr[i], &rt_param[i]);

      threadParams[i].threadIdx=i;
    }
   
    printf("Service threads will run on %d CPU cores\n", CPU_COUNT(&threadcpu));

    // Create Service threads which will block awaiting release for:
    //

    // Servcie_1 = RT_MAX-1	@ 25 Hz
    //
    rt_param[0].sched_priority=rt_max_prio-1;
    pthread_attr_setschedparam(&rt_sched_attr[0], &rt_param[0]);
    rc=pthread_create(&threads[0],               // pointer to thread descriptor
                      &rt_sched_attr[0],         // use specific attributes
                      //(void *)0,               // default attributes
                      Service_1_frame_acquisition,                 // thread function entry point
                      (void *)&(threadParams[0]) // parameters to pass in
                     );
    if(rc < 0)
        perror("pthread_create for service 1 - V4L2 video frame acquisition");
    else
        printf("pthread_create successful for service 1\n");


    // Service_2 = RT_MAX-2	@ 1 Hz
    //
    rt_param[1].sched_priority=rt_max_prio-2;
    pthread_attr_setschedparam(&rt_sched_attr[1], &rt_param[1]);
    rc=pthread_create(&threads[1], &rt_sched_attr[1], Service_2_frame_process, (void *)&(threadParams[1]));
    if(rc < 0)
        perror("pthread_create for service 2 - flash frame storage");
    else
        printf("pthread_create successful for service 2\n");


    // Service_3 = RT_MAX-3	@ 1 Hz
    //
    rt_param[2].sched_priority=rt_max_prio-3;
    pthread_attr_setschedparam(&rt_sched_attr[2], &rt_param[2]);
    rc=pthread_create(&threads[2], &rt_sched_attr[2], Service_3_frame_storage, (void *)&(threadParams[2]));
    if(rc < 0)
        perror("pthread_create for service 3 - flash frame storage");
    else
        printf("pthread_create successful for service 3\n");


    // Wait for service threads to initialize and await relese by sequencer.
    //
    // Note that the sleep is not necessary of RT service threads are created with 
    // correct POSIX SCHED_FIFO priorities compared to non-RT priority of this main
    // program.
    //
    // sleep(1);
 
    // Create Sequencer thread, which like a cyclic executive, is highest prio
    printf("Start sequencer\n");

    // Sequencer = RT_MAX	@ 100 Hz
    //
    /* set up to signal SIGALRM if timer expires */
    timer_create(CLOCK_REALTIME, NULL, &timer_1);

    signal(SIGALRM, (void(*)()) Sequencer);


    /* arm the interval timer */
    itime.it_interval.tv_sec = 0;
    itime.it_interval.tv_nsec = 10000000;
    itime.it_value.tv_sec = 0;
    itime.it_value.tv_nsec = 10000000;
    //itime.it_interval.tv_sec = 1;
    //itime.it_interval.tv_nsec = 0;
    //itime.it_value.tv_sec = 1;
    //itime.it_value.tv_nsec = 0;

    timer_settime(timer_1, flags, &itime, &last_itime);


    for(i=0;i<NUM_THREADS;i++)
    {
        if(rc=pthread_join(threads[i], NULL) < 0)
		perror("main pthread_join");
	else
		printf("joined thread %d\n", i);
    }

   v4l2_frame_acquisition_shutdown();

   printf("\nTEST COMPLETE\n");
}



void Sequencer(int id)
{
    struct timespec current_time_val;
    double current_realtime;
    int rc, flags=0;

    // received interval timer signal
    if(abortTest)
    {
        // disable interval timer
        itime.it_interval.tv_sec = 0;
        itime.it_interval.tv_nsec = 0;
        itime.it_value.tv_sec = 0;
        itime.it_value.tv_nsec = 0;
        timer_settime(timer_1, flags, &itime, &last_itime);
	printf("Disabling sequencer interval timer with abort=%d and %llu\n", abortTest, seqCnt);

	// shutdown all services
        abortS1=TRUE; abortS2=TRUE; abortS3=TRUE;
        sem_post(&semS1); sem_post(&semS2); sem_post(&semS3);

    }
           
    seqCnt++;

    //clock_gettime(MY_CLOCK_TYPE, &current_time_val); current_realtime=realtime(&current_time_val);
    //printf("Sequencer on core %d for cycle %llu @ sec=%6.9lf\n", sched_getcpu(), seqCnt, current_realtime-start_realtime);
    //syslog(LOG_CRIT, "Sequencer on core %d for cycle %llu @ sec=%6.9lf\n", sched_getcpu(), seqCnt, current_realtime-start_realtime);


    // Release each service at a sub-rate of the generic sequencer rate

    // Servcie_1 @ 5 Hz
    if((seqCnt % 20) == 0) sem_post(&semS1);

    // Service_2 @ 1 Hz
    if((seqCnt % 100) == 0) sem_post(&semS2);

    // Service_3 @ 1 Hz
    if((seqCnt % 100) == 0) sem_post(&semS3);
}




void *Service_1_frame_acquisition(void *threadp)
{
    struct timespec current_time_val;
    double current_realtime;
    unsigned long long S1Cnt=0;
    threadParams_t *threadParams = (threadParams_t *)threadp;

    // Start up processing and resource initialization
    clock_gettime(MY_CLOCK_TYPE, &current_time_val); current_realtime=realtime(&current_time_val);
    syslog(LOG_CRIT, "S1 thread @ sec=%6.9lf\n", current_realtime-start_realtime);
    printf("S1 thread @ sec=%6.9lf\n", current_realtime-start_realtime);

    while(!abortS1) // check for synchronous abort request
    {
	// wait for service request from the sequencer, a signal handler or ISR in kernel
        sem_wait(&semS1);

	if(abortS1) break;
        S1Cnt++;

	// DO WORK - acquire V4L2 frame here or OpenCV frame here
	seq_frame_read();

	// on order of up to milliseconds of latency to get time
        clock_gettime(MY_CLOCK_TYPE, &current_time_val); current_realtime=realtime(&current_time_val);
        syslog(LOG_CRIT, "S1 at 25 Hz on core %d for release %llu @ sec=%6.9lf\n", sched_getcpu(), S1Cnt, current_realtime-start_realtime);

	if(0) {abortTest=TRUE;};
    }

    // Resource shutdown here
    //
    pthread_exit((void *)0);
}


void *Service_2_frame_process(void *threadp)
{
    struct timespec current_time_val;
    double current_realtime;
    unsigned long long S2Cnt=0;
    int process_cnt;
    threadParams_t *threadParams = (threadParams_t *)threadp;

    clock_gettime(MY_CLOCK_TYPE, &current_time_val); current_realtime=realtime(&current_time_val);
    syslog(LOG_CRIT, "S2 thread @ sec=%6.9lf\n", current_realtime-start_realtime);
    printf("S2 thread @ sec=%6.9lf\n", current_realtime-start_realtime);

    while(!abortS2)
    {
        sem_wait(&semS2);

	if(abortS2) break;
        S2Cnt++;

	// DO WORK - transform frame
	process_cnt=seq_frame_process();

        clock_gettime(MY_CLOCK_TYPE, &current_time_val); current_realtime=realtime(&current_time_val);
        syslog(LOG_CRIT, "S2 at 1 Hz on core %d for release %llu @ sec=%6.9lf\n", sched_getcpu(), S2Cnt, current_realtime-start_realtime);
    }

    pthread_exit((void *)0);
}


void *Service_3_frame_storage(void *threadp)
{
    struct timespec current_time_val;
    double current_realtime;
    unsigned long long S3Cnt=0;
    int store_cnt;
    threadParams_t *threadParams = (threadParams_t *)threadp;

    clock_gettime(MY_CLOCK_TYPE, &current_time_val); current_realtime=realtime(&current_time_val);
    syslog(LOG_CRIT, "S3 thread @ sec=%6.9lf\n", current_realtime-start_realtime);
    printf("S3 thread @ sec=%6.9lf\n", current_realtime-start_realtime);

    while(!abortS3)
    {
        sem_wait(&semS3);

	if(abortS3) break;
        S3Cnt++;

	// DO WORK - store frame
	store_cnt=seq_frame_store();

        clock_gettime(MY_CLOCK_TYPE, &current_time_val); current_realtime=realtime(&current_time_val);
        syslog(LOG_CRIT, "S3 at 1 Hz on core %d for release %llu @ sec=%6.9lf\n", sched_getcpu(), S3Cnt, current_realtime-start_realtime);

	// after last write, set synchronous abort
	if(store_cnt == 1800) {abortTest=TRUE;};
    }

    pthread_exit((void *)0);
}



double getTimeMsec(void)
{
  struct timespec event_ts = {0, 0};

  clock_gettime(MY_CLOCK_TYPE, &event_ts);
  return ((event_ts.tv_sec)*1000.0) + ((event_ts.tv_nsec)/1000000.0);
}


double realtime(struct timespec *tsptr)
{
    return ((double)(tsptr->tv_sec) + (((double)tsptr->tv_nsec)/1000000000.0));
}


void print_scheduler(void)
{
   int schedType;

   schedType = sched_getscheduler(getpid());

   switch(schedType)
   {
       case SCHED_FIFO:
           printf("Pthread Policy is SCHED_FIFO\n");
           break;
       case SCHED_OTHER:
           printf("Pthread Policy is SCHED_OTHER\n"); exit(-1);
         break;
       case SCHED_RR:
           printf("Pthread Policy is SCHED_RR\n"); exit(-1);
           break;
       //case SCHED_DEADLINE:
       //    printf("Pthread Policy is SCHED_DEADLINE\n"); exit(-1);
       //    break;
       default:
           printf("Pthread Policy is UNKNOWN\n"); exit(-1);
   }
}

